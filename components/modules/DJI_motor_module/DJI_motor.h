/**
 * @brief DJI电机控制
 * @date 24/11/9
 * @note DJI_Motor_Bus_t仅针对CAN端口
 * 上层代码仅对电机句柄操作
 * @details 
 * 使用方法:
 * ||添加电机
 * 1. 声明DJI_Motor_Ctrl_t变量
 * 2. 调用DJI_Motor_init初始化
 * 3. 调用DJI_Motor_<Item>_PID_init初始化pid
 * 
 * 模块工作流程图见obsidian文档
 * @todo 添加电机输出转轴位置环控制
 */
#ifndef __DJI_MOTOR_MODULE__
#define __DJI_MOTOR_MODULE__

#include "main.h"
#include "DJI_motor_canbus.h"
#include "struct_typedef.h"
#include "general_motor_module.h"
#include "pid.h"

typedef enum{
  M2006=0,
  M3508,
  ANY_MOTOR,
} DJI_Motor_Type_e;

// 表示电机初始化状态，
//限制条件不足时电机的控制
typedef enum{
  MOTOR_SPEED_PID_INIT       =0x01<<0,
  MOTOR_POS_PID_INIT         =0x01<<1,
  EXTERN_ENCODER_INIT        =0x01<<2,
  MOTOR_ANGLE_LIMIT_INIT     =0x01<<3,
  MOTOR_SPEED_LIMIT_INIT     =0x01<<4,
  MOTOR_CURRENT_LIMIT_INIT   =0x01<<5,
  MOTOR_STALL_DETECT_INIT    =0x01<<6,
  MOTOR_OFFLINE_DETECT_INIT  =0x01<<7
} Motor_Ctrl_init_state_e;

typedef struct __DJI_Motor_Bus_t DJI_Motor_Bus_t;

/*反馈报文*/
#pragma pack(1)
typedef struct{
  uint16_t ecd;
  int16_t speed_rpm;
  int16_t torque;
  uint8_t temp;
  uint8_t REMAIN;
} dji_recv_pack;
#pragma pack()

typedef struct{

} DJI_Motor_Config_t;

typedef struct __DJI_Motor_Ctrl_t{
/*电机结构体*/
  DJI_Motor_Bus_t* mounted_bus;
  PidTypeDef pid_speed_loop;
  PidTypeDef pid_pos_loop;
  dji_recv_pack recv_pack;

/*电机属性*/
  DJI_Motor_Type_e type;
  uint16_t id;

/*标志*/
  Motor_Ctrl_mode_e mode;
  Motor_Ctrl_init_state_e init_state;
  //电机反转标志,会使发送的电流与反馈的速度，转矩，角度置为负值
  uint8_t reverse_flag;

/*目标值*/
  fp32 set_speed;//目标速度 range:0~2PI 下同
  fp32 set_angle;//目标角度
  int16_t set_current;
  fp32 set_lock_angle;

/*限制*/
  //除了braking_angle其它都在设置目标值时使用
  fp32 max_angle;
  fp32 min_angle;
  fp32 braking_angle;
  fp32 speed_limit;
  fp32 current_limit;

/*角度反馈*/
  fp32 *ref_ptr;// 反馈变量指针,可以为该结构体的angle成员
  uint16_t last_ecd;
  fp32 ecd_angle;// 转子角度反馈
  fp32 last_ecd_angle;// 上一转子角度反馈

/*WARNING:不要同时使用两种测量方式*/
/*多圈角度测量:转子圈数*/
uint8_t circle_count_flag;//电机转子计数标志,启用圈速计数反馈
fp32 circle_count;//多圈角度:转子圈数
/*多圈角度测量:角度累加*/
uint8_t angle_sum_flag;
int32_t ecd_sum;
fp32 angle_sum;

/*堵转检测*/
uint16_t stall_loop_count;
uint16_t stall_loop_count_compare;
fp32 stall_delta_angle;
uint8_t stall_current;
uint8_t stall_flag;

/*掉电检测*/
  uint16_t non_feedback_loop_count;
  uint16_t non_feedback_counter_compare;/* 掉电检测计数器比较值*/
  uint8_t offline_flag;
  uint8_t offline_recover;
} DJI_Motor_Ctrl_t;

/*preset_bus_handler*/
extern DJI_Motor_Bus_t DJI_CAN1_Bus_ctrl;
extern DJI_Motor_Bus_t DJI_CAN2_Bus_ctrl;
extern DJI_Motor_Bus_t DJI_CAN3_Bus_ctrl;

/**********DJI_Motor*********/
/*DJI_Motor初始化设置*/
void DJI_Motor_init(DJI_Motor_Ctrl_t* motor,DJI_Motor_Bus_t* bus,DJI_Motor_Type_e motor_type,uint16_t id);
void DJI_Motor_set_angle_limit(DJI_Motor_Ctrl_t* motor,fp32 max_angle,fp32 min_angle,fp32 braking_angle);
void DJI_Motor_set_speed_limit(DJI_Motor_Ctrl_t* motor,fp32 max_speed);
void DJI_Motor_set_current_limit(DJI_Motor_Ctrl_t* motor,uint16_t max_current);
void DJI_Motor_set_reverse(DJI_Motor_Ctrl_t* motor);
void DJI_Motor_set_angle_feedback(DJI_Motor_Ctrl_t* motor,fp32* feedback_angle);
void DJI_Motor_Pos_PID_init(DJI_Motor_Ctrl_t* motor,enum PID_MODE pid_mod,
  fp32 Kp,fp32 Ki,fp32 Kd,
  fp32 max_out,fp32 max_iout);
void DJI_Motor_Speed_PID_init(DJI_Motor_Ctrl_t* motor,enum PID_MODE pid_mod,
  fp32 Kp,fp32 Ki,fp32 Kd,
  fp32 max_out,fp32 max_iout);
void DJI_Motor_set_multiple_circle_angle(DJI_Motor_Ctrl_t* motor);
void DJI_Motor_set_sum_angle(DJI_Motor_Ctrl_t* motor);
void DJI_Motor_PID_set_deadband(DJI_Motor_Ctrl_t* motor,fp32 deadband);
void DJI_Motor_set_stall_detect(DJI_Motor_Ctrl_t* motor);
void DJI_Motor_set_offline_detect(DJI_Motor_Ctrl_t* motor,uint16_t counter,uint8_t recoverable);

/*电机控制接口*/
void DJI_Motor_set_angle(DJI_Motor_Ctrl_t* motor, fp32 angle);//
void DJI_Motor_set_speed(DJI_Motor_Ctrl_t* motor, fp32 speed_rpm);//速度设置
void DJI_Motor_set_current(DJI_Motor_Ctrl_t* motor, int16_t current);
void DJI_Motor_set_nonforce(DJI_Motor_Ctrl_t* motor);
void DJI_Motor_lockup(DJI_Motor_Ctrl_t* motor);//电机自锁

/*电机反馈接口*/
void DJI_Motor_get_feedback(DJI_Motor_Ctrl_t* motor,fp32* torque,fp32* speed,fp32* angle);
uint8_t DJI_Motor_get_stall_flag(DJI_Motor_Ctrl_t* motor);
uint8_t DJI_Motor_get_offline_flag(DJI_Motor_Ctrl_t* motor);
void DJI_Motor_set_offline(DJI_Motor_Ctrl_t *motor);

/*电机状态控制接口*/
void DJI_Motor_clear_offline_flag(DJI_Motor_Ctrl_t *motor);
void DJI_Motor_clear_circle_count(DJI_Motor_Ctrl_t* motor);

/*DJI_Motor循环控制接口*/
void __DJI_Motor_ctrl_loop(DJI_Motor_Ctrl_t* motor);
void __DJI_Motor_speed_ctrl_loop(DJI_Motor_Ctrl_t* motor);
void __DJI_Motor_pos_ctrl_loop(DJI_Motor_Ctrl_t* motor);
void __DJI_Motor_current_ctrl_loop(DJI_Motor_Ctrl_t* motor);
void __DJI_Motor_get_feedback(DJI_Motor_Ctrl_t* motor,uint8_t* rx_msg);

/**********PRIVATE**********/
/*预设pid值*/
static void __DJI_Motor_preset_pid_m2006(DJI_Motor_Ctrl_t* motor);
static void __DJI_Motor_preset_pid_m3508(DJI_Motor_Ctrl_t* motor);

static fp32 __DJI_Motor_speed_loop_calc(DJI_Motor_Ctrl_t* motor);
static fp32 __DJI_Motor_angle_loop_calc(DJI_Motor_Ctrl_t* motor);

/*DJI_Motor在线掉线*/
void DJI_Motor_set_online(DJI_Motor_Ctrl_t *motor);


static void __DJI_Motor_warning(void);
//static void __DJI_Motor_circle_

#endif
