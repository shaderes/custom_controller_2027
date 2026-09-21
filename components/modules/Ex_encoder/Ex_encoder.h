#ifndef __EX_ECD__
#define __EX_ECD__

#include "main.h"
#include "DJI_motor.h"
#include "pid.h"
#include "struct_typedef.h"

#define ABS(X) ((X) > 0 ? (X) : -(X))

typedef struct __External_ecd_handler_t External_ecd_handler_t;
typedef struct __External_ecd_bus_handler_t External_ecd_bus_handler_t;

//原先是有两个编码器,一个走can，一个走uart(uart的未使用)
typedef enum
{
  NONE_ECD = 0,
  ECD_USART_COMMUNICATION,
  ECD_CAN_COMMUNICATION,
}
ecd_protocol_type_e;

//可放置不同牌子有不同协议的编码器更根据对于的类型启用对应协议的函数
typedef enum{
  /*can通信*/
  OID_ECD=0,
  can_ecd_count, // 用于遍历存于can_communication_ecd中的数据来通过can发送
  /**********分离线**********/
  /*usart通信*/ 
  //usart_ecd_count,//同上
} ecd_type_e;

struct __External_ecd_handler_t{
  ecd_protocol_type_e protocol_type;//协议类型
  ecd_type_e type;//编码器数量类型
  
  uint32_t current_ecd;
  uint32_t min_ecd;
  uint32_t max_ecd;
  uint32_t process_ecd;
  //uint32_t target_ecd;

  PidTypeDef pid_oid_loop;

  /*CAN总线专用*/
  uint16_t bus_id;
  uint16_t device_id;//can回调传回来的数据中含有的id
  //uint16_t display_id;//测试使用

  /*usart专用*/
  // 可以指向有相同类型的输入出的函数，可以根据条件选择不同指向函数来实现
  unsigned int (*USART_Recv)(uint8_t *, unsigned short);
};

struct __External_ecd_bus_handler_t{
  External_ecd_handler_t *can_communication_ecd[can_ecd_count-1];
  //External_ecd_handler_t *usart_communication_ecd[usart_ecd_count - 1 - can_ecd_count];

};//编码器汇总结构体

/*只能有一个*/
extern External_ecd_bus_handler_t Ex_ecd_bus;//总的句柄

void External_can_ecd_init(External_ecd_handler_t *ecd, uint32_t max_ecd, uint32_t min_ecd, ecd_type_e type, ecd_protocol_type_e protocol_type, uint16_t device_id, uint16_t bus_id);
void External_uart_ecd_init(void);

uint32_t External_ecd_get_value(External_ecd_handler_t *ecd);
//fp32 External_ecd_get_angle(External_ecd_handler_t* ecd);
//fp32* External_ecd_get_angle_pointer(External_ecd_handler_t* ecd);
void __External_ecd_get_process_value(External_ecd_handler_t *ecd);

void External_ecd_oid_set_id(FDCAN_HandleTypeDef* CANx,uint16_t oid_id,uint16_t new_id);
void External_ecd_oid_set_mode(FDCAN_HandleTypeDef *CANx, uint16_t oid_id, uint16_t commmand, uint16_t set_data, uint16_t set_other_ata);

void DJI_Motor_Oid_PID_init(External_ecd_handler_t *ecd, enum PID_MODE pid_mode,
                            fp32 Kp, fp32 Ki, fp32 Kd,
                            fp32 max_out, fp32 max_iout);
fp32 __ADD_OID_LENGTH_OUTPUT(uint32_t value, External_ecd_handler_t *ecd);

// void __External_ecd_usart_feedback_hook(void);
void __External_ecd_can_feedback_hook(uint16_t id, uint8_t *msg);

#endif
