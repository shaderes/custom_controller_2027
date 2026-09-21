#ifndef __DM4310_DRV_H__
#define __DM4310_DRV_H__
#include "main.h"
#include "can_bsp.h"

#define MIT_MODE 			0x000
#define POS_MODE			0x100
#define SPEED_MODE		0x200

//以下是DM4310的参数，用其他电机需要更改下面参数
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

#pragma pack(1)
typedef struct 
{
	uint16_t id;
	uint16_t state;
	int p_int;
	int v_int;
	int t_int;

	float pos;
	float vel;
	float tor;

	float Tmos;
	float Tcoil;
}motor_fbpara_t;
#pragma pack()

typedef struct
{
	uint16_t mode;
  uint16_t output_id;
  uint8_t output[8];
  float Kp;
  float Kd;
  float lock_angle;
  uint8_t enable;
	motor_fbpara_t para;
}Joint_Motor_t ;

typedef FDCAN_HandleTypeDef hcan_t;
/*preset_motor_handler*/
extern Joint_Motor_t DM_Motor_J2;
extern Joint_Motor_t DM_Motor_J3;
extern Joint_Motor_t DM_Motor_J5;
extern Joint_Motor_t DM_Motor_gripper;
extern void dm4310_fbdata(Joint_Motor_t *motor, uint8_t *rx_data, uint32_t data_len);

extern void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
extern void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);

//关节电机
extern void mit_ctrl(Joint_Motor_t* motor_ptr, float pos, float vel,float kp, float kd, float torq);
extern void mit_nonforce_ctrl(Joint_Motor_t* motor_ptr);
extern void pos_speed_ctrl(Joint_Motor_t* motor_ptr, float pos, float vel);
extern void speed_ctrl(hcan_t* hcan,uint16_t motor_id, float _vel);

void mit_ctrl_pos_speed(Joint_Motor_t* motor_ptr,float pos,float vel);
void mit_ctrl_lock(Joint_Motor_t* motor_ptr);
void __dm4310_mit_output_ctrl(hcan_t* hcan,Joint_Motor_t* motor_ptr);

extern void joint_motor_init(Joint_Motor_t *motor,uint16_t id,uint16_t mode,float Kp,float Kd);

	
extern float Hex_To_Float(uint32_t *Byte,int num);//十六进制到浮点数
extern uint32_t FloatTohex(float HEX);//浮点数到十六进制转换

extern float uint_to_float(int x_int, float x_min, float x_max, int bits);
extern int float_to_uint(float x_float, float x_min, float x_max, int bits);
void dm_set_pos(Joint_Motor_t *motor_ptr, float angle);
void DM_Motor_Offline_Handler(Joint_Motor_t *motor_ptr);

#endif /* __DM4310_DRV_H__ */

