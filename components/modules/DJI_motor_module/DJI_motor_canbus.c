#include "DJI_motor_canbus.h"
#include "CAN_receive.h"

// CANBus宏函数
#define __DJI_CANBus_get_motor_instance(bus_ptr,index) ((bus_ptr->mounted_motor)[index])
#define __DJI_CANBus_get_motor_count(bus_ptr) (bus_ptr->mounted_motor_count)

DJI_Motor_Bus_t DJI_CAN1_Bus_ctrl;
DJI_Motor_Bus_t DJI_CAN2_Bus_ctrl;
DJI_Motor_Bus_t DJI_CAN3_Bus_ctrl;

extern FDCAN_HandleTypeDef hcan1;
extern FDCAN_HandleTypeDef hcan2;
extern FDCAN_HandleTypeDef hcan3;

CAN_SuperCapTXDataTypeDef super_tx;

/**
 * @brief 预定义的CANBus初始化
 */
void DJI_CANBus_init_all(void)
{
  DJI_CANBus_init(&DJI_CAN1_Bus_ctrl,&hcan1);
  DJI_CANBus_init(&DJI_CAN2_Bus_ctrl,&hcan2);
  DJI_CANBus_init(&DJI_CAN3_Bus_ctrl,&hcan3);
}

inline void DJI_CANBus_enable_bus(DJI_Motor_Bus_t* DJI_CAN_Bus_ctrl)
{
  DJI_CAN_Bus_ctrl->enable=1;
}

/**
 * @brief CANBus初始化
 */
void DJI_CANBus_init(DJI_Motor_Bus_t* bus,FDCAN_HandleTypeDef* can)
{
  bus->can=can;
  bus->mounted_motor_count=0;
  bus->enable=0;
}

void DJI_CANBus_config_init(DJI_Motor_Bus_t* bus, DJI_Motor_Config_t* config)
{

}
void DJI_CANBus_add_motor(DJI_Motor_Bus_t* bus,DJI_Motor_Ctrl_t* motor)
{
  if(__DJI_CANBus_get_motor_count(bus)>6)
    return;
  __DJI_CANBus_get_motor_instance(bus,__DJI_CANBus_get_motor_count(bus))=motor;

  motor->mounted_bus = bus;
  bus->mounted_motor_count++;
}

/**
 * @brief 总线电机控制循环
 * 执行pid计算和电流发送的操作
 */
void __DJI_CANBus_ctrl_loop(DJI_Motor_Bus_t* bus)
{
  static uint16_t index;

  for(index=0;index<__DJI_CANBus_get_motor_count(bus);index++)
  {
    DJI_Motor_Ctrl_t* motor = __DJI_CANBus_get_motor_instance(bus,index);
    __DJI_Motor_ctrl_loop(motor);
  }
  if(bus->enable)
  {
    CanSendMess(bus->can,0x200,bus->output_current200H);
    CanSendMess(bus->can,0x1ff,bus->output_current1FFH);
  }
    /********************超电通信***********************/
    // if (bus->can == &hcan3)
    // {
    //   Set_CAN_cmd_SuperCap(&super_tx);
    //   CAN_cmd_SuperCap(&super_tx);
    // }
    /********************超电通信***********************/
}

/**
 * @brief CAN总线数据更新钩子
 */
int8_t __DJI_CANBus_feedback_update(DJI_Motor_Bus_t* bus, uint8_t* rx_data, uint16_t rx_id)
{
  static uint16_t index;

  for(index=0;index<__DJI_CANBus_get_motor_count(bus);index++)
  {
    if(rx_id==__DJI_CANBus_get_motor_instance(bus,index)->id)
    {
      //memcpy(&(__DJI_CANBus_get_motor_instance(bus,index)->recv_pack),rx_data,sizeof(uint8_t)*8);
      __DJI_Motor_get_feedback(__DJI_CANBus_get_motor_instance(bus,index),rx_data);
      return 1;
    }
  }
  return 0;
}
