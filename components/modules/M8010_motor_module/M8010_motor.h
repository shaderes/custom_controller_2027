/**
 * @brief M8010电机驱动模块
 * @note 因为肥肠烂usart的bsp，该模块复用性差，但没必要重改
 * @todo 添加过压监测
 */
#ifndef __M8010_MOTOR_MODULE
#define __M8010_MOTOR_MODULE

#include <stdint.h>
#include "main.h"
#include "ris_protocol.h"
#include "crc_ccitt.h"

#pragma pack(1)
typedef struct
{
    uint8_t head[2];    // 包头         2Byte
    RIS_Mode_t mode;    // 电机控制模式  1Byte
    RIS_Fbk_t   fbk;    // 电机反馈数据 11Byte
    uint16_t  CRC16;    // CRC          2Byte
} MotorData_t;  //返回数据

typedef struct
{
    // 定义 电机控制命令数据包
    uint8_t head[2];    // 包头         2Byte
    RIS_Mode_t mode;    // 电机控制模式  1Byte
    RIS_Comd_t comd;    // 电机期望数据 12Byte
    uint16_t   CRC16;   // CRC          2Byte
} ControlData_t;     //电机控制命令数据包
#pragma pack()

typedef struct
{
    // 定义 发送格式化数据
    ControlData_t motor_send_data;   //电机控制数据结构体
    int hex_len;                        //发送的16进制命令数组长度, 34
    long long send_time;                //发送该命令的时间, 微秒(us)
    // 待发送的各项数据
    unsigned short id;                  //电机ID，0代表全部电机
    unsigned short mode;                // 0:空闲, 5:开环转动, 10:闭环FOC控制
    //实际给FOC的指令力矩为：
    // K_P*delta_Pos + K_W*delta_W + T
    float GM_Send_Effort;                            //期望关节的输出力矩（电机本身的力矩）（Nm）
    float GM_Send_speed;                            //期望关节速度（电机本身的速度）(rad/s)
    float GM_Send_Pos;                          //期望关节位置（rad）
    float GM_Send_Kp_Pos;                          //关节刚度系数
    float GM_Send_Kd_Speed;                          //关节速度系数
} MOTOR_send;

typedef struct
{
    // 定义 接收数据
    MotorData_t motor_recv_data;    //电机接收数据结构体，详见motor_msg.h
    int hex_len;                        //接收的16进制命令数组长度, 78
    long long resv_time;                //接收该命令的时间, 微秒(us)
    int correct;                        //接收数据是否完整（1完整，0不完整）
    //解读得出的电机数据
    unsigned char motor_id;             //电机ID
    unsigned char mode;                 // 0:空闲, 5:开环转动, 10:闭环FOC控制
    int Temp;                           //温度
    unsigned char MError;               //错误码
    float T;                            // 当前实际电机输出力矩
		float W;														// speed
    float Pos;                          // 当前电机位置（主控0点修正，电机关节还是以编码器0点为准）
		float footForce;												// 足端气压传感器数据 12bit (0-4095)
} MOTOR_recv;

typedef struct{
  MOTOR_recv recv_data;
  MOTOR_send send_data;

  float Kp;
  float Kd;
 
  float MAX_SPEED;

  float MAX_POS;
  float MIN_POS;

  float MAX_TORQUE;
} M8010_motor_t;

extern M8010_motor_t joint1_motor;

void M8010_motor_init(M8010_motor_t* motor,uint8_t id,float Kp,float Kd);
void M8010_motor_change_param(M8010_motor_t* motor,float Kp,float Kd);

void M8010_motor_set_angle(M8010_motor_t* motor,float angle);
void M8010_motor_set_angle_speed(M8010_motor_t* motor, float angle, float speed);
void M8010_motor_lock(M8010_motor_t* motor);
void M8010_motor_nonforce(M8010_motor_t* motor);
void M8010_motor_set_output(M8010_motor_t* motor,unsigned short mode,float speed,float pos,float torque,float Kp,float Kd);

MOTOR_recv* SERVO_Recv(M8010_motor_t* motor,uint8_t* rx_data);
HAL_StatusTypeDef SERVO_Send(M8010_motor_t* motor);

void __M8010_motor_control_hook(void);
int __modify_data(MOTOR_send *motor_s);
int __extract_data(MOTOR_recv *motor_r);
#endif
