#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"
#include "pid.h"
#include "main.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

/*C610¡¢C620*/
#define SEND_ID201_204 0x200
#define SEND_ID205_208 0X1FF
/*6020*/
#define _6020_SEND_ID201_204 s
/*5-7*/
#define _6020_SEND_ID205_208 0X2FF

#define M2006_REDUCTIONRTION 36
#define M3510_REDUCTIONRTION  27

/*C610¡¢C620*/
#define SEND_ID201_204 0x200
#define SEND_ID205_208 0X1FF
/*6020*/
// #define _6020_SEND_ID201_204 0X1FF
// /*5-7*/
// #define _6020_SEND_ID205_208 0X2FF

#define M2006_REDUCTIONRTION 36
#define M3510_REDUCTIONRTION  27

/*??????*/
#define hcan1 hfdcan1
#define hcan2 hfdcan2
#define hcan3 hfdcan3

typedef enum
{

  CAN_3508_M1_ID                = 0x201,
  CAN_3508_M2_ID                = 0x202,
  CAN_3508_M3_ID                = 0x203,
  CAN_3508_M4_ID                = 0x204,

} can_msg_id_e;

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
	  int64_t crc;
} motor_measure_t;

/********************????***********************/
//

// ????????????????????????????
typedef enum
{
  DISCHARGE = 0,                    // ????
  CHARGE = 1,                       // ????
  WAIT = 2,                         // ????
  SOFTSTART_PROTECTION = 3,         // ???????
  OVER_LOAD_PROTECTION = 4,         // ????????
  BAT_OVER_VOLTAGE_PROTECTION = 5,  // ??????
  BAT_UNDER_VOLTAGE_PROTECTION = 6, // ?????????????????
  CAP_UNDER_VOLTAGE_PROTECTION = 7, // ?????????????????????????
  OVER_TEMPERATURE_PROTECTION = 8,  // ????????
} SuperCapStateEnum;


typedef struct
{
  FunctionalState Enable;   // ???????1???0??
  SuperCapStateEnum Charge; // ???????1???0????PLUS?????????????????????
  uint8_t PowerLimint;      // ????????
  uint8_t ChargePower;      // ??????????PLUS???????????????????????
  uint8_t PowerOffset;
} CAN_SuperCapTXDataTypeDef;

void Set_CAN_cmd_SuperCap(CAN_SuperCapTXDataTypeDef *TX_Temp);
void CAN_cmd_SuperCap(CAN_SuperCapTXDataTypeDef *TX_Temp);
//
/********************????***********************/

const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);
void CAN_RX_hook(FDCAN_HandleTypeDef* CANx, FDCAN_RxHeaderTypeDef* rx_header,uint8_t* rx_message); 
void CanSendMsg(FDCAN_HandleTypeDef* CANx,uint32_t SendID,uint8_t *message);
void CanSendMess(FDCAN_HandleTypeDef* CANx,uint32_t SendID,int16_t *message);
void CanSendMoreMess(FDCAN_HandleTypeDef* CANx, uint32_t SendID, uint8_t *message, uint8_t messageLength);

#endif
