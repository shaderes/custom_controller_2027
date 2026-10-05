#ifndef __ZDT_X42_V2_H
#define __ZDT_X42_V2_H

#include "main.h"
#include "bsp_can.h"
#include "stdbool.h"
/**********************************************************
*** ZDT_X42_V2.0步进闭环控制例程
*** 编写作者：ZHANGDATOU
*** 技术支持：张大头闭环伺服
*** 淘宝店铺：https://zhangdatou.taobao.com
*** CSDN博客：http s://blog.csdn.net/zhangdatou666
*** qq交流群：262438510
**********************************************************/

#define		ABS(x)		((x) > 0 ? (x) : -(x)) 

#define ZDT_TORQUE_MAX_MA    4000.0f

#define ZDT_TORQUE_RAMP_MA_S 30000

#define ZDT_40MM_TORQUE_CONST 0.22f
#define ZDT_23MM_TORQUE_CONST 0.12f

#define ZDT_40MM_MIT_KP 0.0f
#define ZDT_40MM_MIT_KD 0.0f
#define ZDT_40MM_MIT_SET_ANGLE 0.0f
#define ZDT_40MM_MIT_SET_SPEED 0.0f

#define ZDT_23MM_MIT_KP 0.0f
#define ZDT_23MM_MIT_KD 0.0f
#define ZDT_23MM_MIT_SET_ANGLE 0.0f
#define ZDT_23MM_MIT_SET_SPEED 0.0f

typedef enum {
  S_VER   = 0,      /* 读取固件版本和对应的硬件版本 */
  S_RL    = 1,      /* 读取读取相电阻和相电感 */
  S_PID   = 2,      /* 读取PID参数 */
  S_ORG   = 3,      /* 读取回零参数 */
  S_VBUS  = 4,      /* 读取总线电压 */
  S_CBUS  = 5,      /* 读取总线电流 */
  S_CPHA  = 6,      /* 读取相电流 */
  S_ENC   = 7,      /* 读取编码器原始值 */
  S_CPUL  = 8,      /* 读取实时脉冲数（根据实时位置计算得到的脉冲数） */
  S_ENCL  = 9,      /* 读取经过线性化校准后的编码器值 */
  S_TPUL  = 10,     /* 读取输入脉冲数 */
  S_TPOS  = 11,     /* 读取电机目标位置 */
  S_OPOS  = 12,     /* 读取电机实时设定的目标位置（开环模式的实时位置） */
  S_VEL   = 13,     /* 读取电机实时转速 */
  S_CPOS  = 14,     /* 读取电机实时位置（基于角度编码器累加的电机实时位置） */
  S_PERR  = 15,     /* 读取电机位置误差 */
  S_TEMP  = 16,     /* 读取电机实时温度 */
  S_SFLAG = 17,     /* 读取状态标志位 */
  S_OFLAG = 18,     /* 读取回零状态标志位 */
  S_Conf  = 19,     /* 读取驱动参数 */
  S_State = 20,     /* 读取系统状态参数 */
}SysParams_t;

typedef struct {
  uint8_t id;                 // 电机ID (1~6)
  int16_t target_current;     // 目标电流值
  int16_t actual_current;     // 实际电流值
  uint16_t voltage;           // 电压值 (mV)
  int8_t temperature;         // 温度值 (℃)
  uint8_t error_code;         // 错误码
  uint8_t enabled;            // 使能状态
  float position;             // 当前角度位置 (度)
  float rad;                  // 当前角度位置 (rad)
	uint8_t dir;
	float dq;              // 当前关节速度（单位：rad/s）
  float theta_prev;      // 上一时刻的关节位置（单位：°）
  uint32_t timestamp;    // 数据更新时间戳
  uint8_t valid;         // 数据是否有效
} MotorCurrentInfo_t;

/* 电机数据本体：下标 = 电机地址 - 1，由 bsp_can.c 的 CAN 接收中断刷新 */
extern MotorCurrentInfo_t MotorCurrents[ZDT_MOTOR_NUM];
/* 关节角度：单位 rad，由 Get_theta() 根据电机角度换算得到 */
extern float theta[6];

extern float zdt_torque_const[3];
extern float zdt_speed_target[3];
extern float zdt_angle_target[3];
/* 用 MotorCurrents[index] 的电机角度刷新关节角 theta[index]（index = 电机地址 - 1） */
void Get_theta(const MotorCurrentInfo_t *motor_currents, uint8_t index);

void ZDT_X42_V2_Reset_CurPos_To_Zero(uint8_t addr); // 将当前位置清零
void ZDT_X42_V2_Reset_Clog_Pro(uint8_t addr); // 解除堵转保护
void ZDT_X42_V2_Read_Sys_Params(uint8_t addr, SysParams_t s); // 读取参数
void ZDT_X42_V2_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode); // 发送命令切换开环/闭环控制模式
void ZDT_X42_V2_En_Control(uint8_t addr, bool state, uint8_t snF); // 电机使能控制
void ZDT_X42_V2_Torque_Control(uint8_t addr, uint8_t sign, uint16_t t_ramp, uint16_t torque, uint8_t snF); // 力矩模式控制
void ZDT_X42_V2_Velocity_Control(uint8_t addr, uint8_t dir, uint16_t v_ramp, float velocity, uint8_t snF); // 速度模式控制
void ZDT_X42_V2_Bypass_Position_LV_Control(uint8_t addr, uint8_t dir, float velocity, float position, uint8_t raf, uint8_t snF); // 直通限速位置模式控制
void ZDT_X42_V2_Traj_Position_Control(uint8_t addr, uint8_t dir, uint16_t acc, uint16_t dec, float velocity, float position, uint8_t raf, uint8_t snF); // 梯形曲线加减速位置模式控制
void ZDT_X42_V2_Stop_Now(uint8_t addr, uint8_t snF); // 让电机立即停止运动
void ZDT_X42_V2_Synchronous_motion(uint8_t addr); // 触发多机同步开始运动
void ZDT_X42_V2_Origin_Set_O(uint8_t addr, bool svF); // 设置单圈回零的零点位置
void ZDT_X42_V2_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF); // 修改回零参数
void ZDT_X42_V2_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF); // 发送命令触发回零
void ZDT_X42_V2_Origin_Interrupt(uint8_t addr); // 强制中断并退出回零
void ZDT_X42_V2_Receive_Data(uint8_t *rxCmd, uint8_t *rxCount); // 返回数据接收函数

void USER_ZDT_X42_V2_Read_Sys_Params(CAN_HandleTypeDef *hcan, uint8_t addr, SysParams_t s); // 读取参数
void USER_ZDT_X42_V2_Torque_Control(CAN_HandleTypeDef *hcan, uint8_t addr, uint8_t sign, uint16_t t_ramp, uint16_t torque, uint8_t snF); // 力矩模式控制
#endif
