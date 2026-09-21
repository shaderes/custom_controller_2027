/**
 * @brief DJI电机CAN总线模块
 * @note 与DJI_motor强耦合，只是为了方便才进行拆分
 */
#ifndef __DJI_MOTOR_CANBUS_MODULE__
#define __DJI_MOTOR_CANBUS_MODULE__

#include "DJI_motor.h"

#define MAX_MOTOR_MOUNTED 6

typedef struct __DJI_Motor_Ctrl_t DJI_Motor_Ctrl_t;

/**
 * @brief dji电机总线句柄结构体
 * @date 24/11/8
 */
typedef struct __DJI_Motor_Bus_t
{
  FDCAN_HandleTypeDef *can;
  DJI_Motor_Ctrl_t *mounted_motor[MAX_MOTOR_MOUNTED];
  uint8_t enable; // 总线输出使能
  uint16_t mounted_motor_count;

  int16_t output_current200H[4];
  int16_t output_current1FFH[4];
} DJI_Motor_Bus_t;

extern DJI_Motor_Bus_t DJI_CAN1_Bus_ctrl;
extern DJI_Motor_Bus_t DJI_CAN2_Bus_ctrl;
extern DJI_Motor_Bus_t DJI_CAN3_Bus_ctrl;

/*preset_interface*/
void DJI_CANBus_init_all(void);
void DJI_CANBus_enable_bus(DJI_Motor_Bus_t *DJI_CAN_Bus_ctrl);

/*DJI_CANBus*/
void DJI_CANBus_init(DJI_Motor_Bus_t *bus, FDCAN_HandleTypeDef *can);
// void DJI_CANBus_config_init(DJI_Motor_Bus_t* bus, DJI_Motor_Config_t* config);
void DJI_CANBus_add_motor(DJI_Motor_Bus_t *bus, DJI_Motor_Ctrl_t *motor);

/*DJI_CANBus循环控制接口*/
void __DJI_CANBus_ctrl_loop(DJI_Motor_Bus_t *bus);
int8_t __DJI_CANBus_feedback_update(DJI_Motor_Bus_t *bus, uint8_t *rx_data, uint16_t rx_id);

#endif
