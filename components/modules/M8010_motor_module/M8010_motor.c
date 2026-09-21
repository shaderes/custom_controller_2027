#include "M8010_motor.h"
#include "bsp_usart.h"
#include <string.h>

#define SATURATE(_IN, _MIN, _MAX) {\
 if (_IN < _MIN)\
 _IN = _MIN;\
 else if (_IN > _MAX)\
 _IN = _MAX;\
 } 

#define __M8010_SET_OUTPUT_KP_KD(motor_ptr,KP,KD) \
{ \
  motor_ptr->send_data.GM_Send_Kp_Pos=KP; \
  motor_ptr->send_data.GM_Send_Kd_Speed=KD; \
} \

M8010_motor_t joint1_motor;

void M8010_motor_init(M8010_motor_t* motor,uint8_t id,float Kp,float Kd)
{
  motor->send_data.id=id;
  motor->Kp=Kp;
  motor->Kd=Kd;
}

void M8010_motor_change_param(M8010_motor_t* motor,float Kp,float Kd)
{
  motor->Kp=Kp;
  motor->Kd=Kd;
}

void M8010_motor_set_angle(M8010_motor_t* motor,float angle)
{
  M8010_motor_set_output(motor,
    1,
    0,
    angle,
    0,
    motor->Kp,
    motor->Kd
  );
}

void M8010_motor_set_angle_speed(M8010_motor_t* motor, float angle, float speed)
{
  M8010_motor_set_output(motor,
    1,
    speed,
    angle,
    0,
    motor->Kp,
    motor->Kd 
  );
}

void M8010_motor_lock(M8010_motor_t* motor)
{
  M8010_motor_set_output(motor,
    0,
    0,
    0,
    0,
    0,
    0
  );
}

void M8010_motor_nonforce(M8010_motor_t* motor)
{
  M8010_motor_set_output(motor,
    1,
    0,
    0,
    0,
    0,
    0
  );
}

void M8010_motor_set_output(M8010_motor_t* motor,unsigned short mode,float speed,float pos,float torque,float Kp,float Kd)
{
  motor->send_data.mode=mode;
  motor->send_data.GM_Send_speed=speed;
  motor->send_data.GM_Send_Pos=pos;
  motor->send_data.GM_Send_Effort=torque;
  motor->send_data.GM_Send_Kp_Pos=Kp;
  motor->send_data.GM_Send_Kd_Speed=Kd;
}

MOTOR_recv* SERVO_Recv(M8010_motor_t* motor,uint8_t* rx_data)
{
  memcpy((void*)&(motor->recv_data.motor_recv_data),(void*)rx_data,sizeof(MotorData_t));
  __extract_data(&(motor->recv_data));
  return &(motor->recv_data);
}

HAL_StatusTypeDef SERVO_Send(M8010_motor_t* motor)
{
  __modify_data(&(motor->send_data));
  if(motor==&joint1_motor)
  {
    USART2_Send((uint8_t*)(void*)&(motor->send_data.motor_send_data),
      sizeof(motor->send_data.motor_send_data));
  }
  return HAL_OK;
}

/**
 * @brief 将发送结构体数据封装成数据包，得到直接用于发送的数据
 */
int __modify_data(MOTOR_send *motor_s)
{
    motor_s->hex_len = 17;
    motor_s->motor_send_data.head[0] = 0xFE;
    motor_s->motor_send_data.head[1] = 0xEE;
	
		SATURATE(motor_s->GM_Send_Kp_Pos,  0.0f,   25.599f);
		SATURATE(motor_s->GM_Send_Kd_Speed,  0.0f,   25.599f);
		SATURATE(motor_s->GM_Send_Effort,   -127.99f,  127.99f);
		SATURATE(motor_s->GM_Send_speed,   -804.00f,  804.00f);
		SATURATE(motor_s->GM_Send_Pos, -411774.0f,  411774.0f);

    motor_s->motor_send_data.mode.id   = motor_s->id;
    motor_s->motor_send_data.mode.status  = motor_s->mode;
    motor_s->motor_send_data.comd.k_pos  = motor_s->GM_Send_Kp_Pos/25.6f*32768;
    motor_s->motor_send_data.comd.k_spd  = motor_s->GM_Send_Kd_Speed/25.6f*32768;
    motor_s->motor_send_data.comd.pos_des  = motor_s->GM_Send_Pos/6.2832f*32768;
    motor_s->motor_send_data.comd.spd_des  = motor_s->GM_Send_speed/6.2832f*256;
    motor_s->motor_send_data.comd.tor_des  = motor_s->GM_Send_Effort*256;
    motor_s->motor_send_data.CRC16 = crc_ccitt(0, (uint8_t *)&motor_s->motor_send_data, 15);
    return 0;
}

/**
 * @brief 解包数据函数，负责解包数据并存在结构体中
 */
int __extract_data(MOTOR_recv *motor_r)
{
    if(motor_r->motor_recv_data.CRC16 !=
        crc_ccitt(0, (uint8_t *)&motor_r->motor_recv_data, 14)){
        // printf("[WARNING] Receive data CRC error");
        motor_r->correct = 0;
        return motor_r->correct;
    }
    else
		{
        motor_r->motor_id = motor_r->motor_recv_data.mode.id;
        motor_r->mode = motor_r->motor_recv_data.mode.status;
        motor_r->Temp = motor_r->motor_recv_data.fbk.temp;
        motor_r->MError = motor_r->motor_recv_data.fbk.MError;
        motor_r->W = ((float)motor_r->motor_recv_data.fbk.speed/256)*6.2832f ;
        motor_r->T = ((float)motor_r->motor_recv_data.fbk.torque) / 256;
        motor_r->Pos = 6.2832f*((float)motor_r->motor_recv_data.fbk.pos) / 32768;
		    motor_r->footForce = motor_r->motor_recv_data.fbk.force;
		    motor_r->correct = 1;
        return motor_r->correct;
    }
}

void __M8010_motor_control_hook(void)
{
  SERVO_Send(&joint1_motor);
}
