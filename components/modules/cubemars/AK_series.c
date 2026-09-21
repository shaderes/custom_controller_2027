#include "AK_series.h"
#include "can_bsp.h"
//当前电机型号ak10-9_kv60
//电机在不同位置上电得到的零点不同，像是只得到了内部位置，但上电从当前位置开始可记录走过的角度，即可正常使用
//多种尝试未能实现上电得到当前的绝对位置，可能这电机个不具备双编码功能

/*当前AK70-10的id:93*/
AK_Joint_Motor_t AK70_10_motor = {0};

void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void buffer_append_int16(uint8_t* buffer, int16_t number, int16_t *index) {
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void AK_joint_motor_init(AK_Joint_Motor_t *motor, uint16_t id)
{
  motor->id=id;
  motor->enable=0;
  AK_joint_motor_nonforce_ctrl(motor);

  
}

void AK_joint_motor_enable(AK_Joint_Motor_t * motor)
{
  motor->enable = 1;
}

void AK_joint_motor_disable(AK_Joint_Motor_t * motor)
{
  /*todo:disable的时候需要电机进入失力状态*/
  motor->enable = 0;
}


void AK_joint_motor_current_ctrl(AK_Joint_Motor_t * motor, float current)
{
  motor->mode = CAN_PACKET_SET_CURRENT;
  int32_t send_index = 0;
  buffer_append_int32((motor->tx_buffer), (int32_t)(current * 100.0f), &send_index);
}

void AK_joint_motor_speed_ctrl(AK_Joint_Motor_t * motor, float rpm)
{
  motor->mode = CAN_PACKET_SET_RPM;
  int32_t send_index = 0;
  buffer_append_int32(motor->tx_buffer, (int32_t)rpm, &send_index);
}


void AK_joint_motor_pos_speed_ctrl(AK_Joint_Motor_t * motor, float pos, float spd, float RPA)
{
  motor->mode = CAN_PACKET_SET_POS_SPD;
  int32_t send_index = 0;
  int16_t send_index1 = 4;
  buffer_append_int32(motor->tx_buffer, (int32_t)(pos * 10000.0f), &send_index);
  buffer_append_int16(motor->tx_buffer, spd / 10.0f, &send_index1);
  buffer_append_int16(motor->tx_buffer, RPA / 10.0f, &send_index1);
}

void AK_joint_motor_nonforce_ctrl(AK_Joint_Motor_t * motor)
{
  AK_joint_motor_current_ctrl(motor, 0.0f);
}

void AK_joint_motor_lock_ctrl(AK_Joint_Motor_t *motor)
{
  AK_joint_motor_pos_speed_ctrl(motor, motor->pos, 3500, 5500);
}

void __AK_joint_motor_feedback_hook(AK_Joint_Motor_t *motor, uint8_t *rx_message)
{
  // void motor_receive(float* motor_pos,float* motor_spd,float* cur,int_8* temp,int_8* error,rx_message)
  // float motor_pos,motor_spd,cur;
  // int8_t temp,error;
  int16_t pos_int = (rx_message[0] << 8) | rx_message[1];
  int16_t spd_int = (rx_message[2] << 8) | rx_message[3];
  int16_t cur_int = (rx_message[4] << 8) | rx_message[5];
  motor->pos = (float)(pos_int * 0.1f);      // 电机位置
  motor->spd = (float)(spd_int * 10.0f);     // 电机速度
  motor->current = (float)(cur_int * 0.01f); // 电机电流
  motor->temp = rx_message[6];               // 电机温度
  motor->error = rx_message[7];              // 电机故障码
}

void AK_joint_motor_set_zero_pos(AK_Joint_Motor_t * motor)
{
  // int32_t send_index = 0;
  // uint8_t buffer;
  motor->mode = CAN_PACKET_SET_ORIGIN_HERE;
  motor->tx_buffer[0] = 0x00;
  // comm_can_transmit_eid(controller_id |((uint32_t) CAN_PACKET_SET_ORIGIN_HERE << 8), &buffer, send_index);
}

// 实测并没有掉电不丢失位置作用，效果和AK_joint_motor_set_zero_pos一样
void AK_joint_motor_set_forever_zero_pos(AK_Joint_Motor_t *motor)
{

  motor->mode = CAN_PACKET_SET_ORIGIN_HERE;
  motor->tx_buffer[0] = 0x01;
}

void __AK_joint_motor_ctrl_hook(AK_Joint_Motor_t * motor, AK_hcan_t * can)
{
  static uint32_t data_len;
  if (motor->enable)
  {
    switch (motor->mode)
    {
    case CAN_PACKET_SET_ORIGIN_HERE:
      data_len=CAN_DATA_SIZE_1_BYTES;
      break;
    case CAN_PACKET_SET_CURRENT:
    case CAN_PACKET_SET_RPM:
      data_len=CAN_DATA_SIZE_4_BYTES;
      break;
    case CAN_PACKET_SET_POS_SPD:
      data_len=CAN_DATA_SIZE_8_BYTES;
      break;
    default:
      AK_joint_motor_nonforce_ctrl(motor);
      data_len=CAN_DATA_SIZE_4_BYTES;
  }
  fdcanx_send_data_ex_mode(can
    ,(motor->id)|((uint32_t)(motor->mode) << 8)
    ,motor->tx_buffer,data_len);
  }
}


//这个电机油大饼啊，各种非人设计，设置永久零点为什么零点不在当前位置,上电后后位置也是混乱的，这根本就不是双编码电机
//调试该电机请认真阅读书册，最好拆下来尝试
//上位机写参数前一定要读参数

