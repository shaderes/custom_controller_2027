#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "main.h"

/* ==========================================================================
 *  张大头(ZDT) X42_V2 闭环步进电机 CAN 通信
 *  数组下标统一使用 (地址 - 1)：
 *      ZDT_MOTOR_NUM    数组/协议容量（6）
 *      ZDT_MOTOR_ONLINE 实际在线的电机台数（3，地址 1~3）—— 轮询按它来
 *
 *  电机数据本体用的是 ZDT 模块的 MotorCurrentInfo_t（见 ZDT_X42_V2.h）：
 *      MotorCurrents[]  定义在 ZDT_X42_V2.c，本文件的 CAN 中断负责刷新
 *      theta[]          关节角(rad)，由 bsp_can.c 的 zdt_update_theta() 按
 *                       "地址→关节"映射换算（映射表见下面的 ZDT_JOINT_OFFSET）
 * ========================================================================*/
#define ZDT_MOTOR_NUM       6U       /* 数组/协议容量上限：下标统一用 (地址 - 1)，最多 6 台 */
#define ZDT_MOTOR_ONLINE    3U       /* ★ 实际挂在 CAN1 上的电机台数（地址 1 ~ 3）。
                                        只有这个范围内的电机会被轮询/参与有效位图；
                                        6 是容量，不要为了"少发几帧"把上面那个数改小 ——
                                        MotorCurrents[]/theta[] 都是按容量开的。
                                        以后加电机只改这一个数（≤ ZDT_MOTOR_NUM）。 */
#define ZDT_FUNC_CPOS       0x36U    /* 功能码：读取电机实时位置(角度) */
#define ZDT_FUNC_VEL        0x35U    /* 功能码：读取电机实时转速 */
#define ZDT_FRAME_TAIL      0x6BU    /* 帧尾校验字节 */

/* ===== 电机地址 → 关节号的映射（很重要，别改错）=====
   实际挂的 3 台步进电机（地址 1/2/3）驱动的是机械臂的 3/4/5 号关节：

        地址 1 → 关节 3        地址 2 → 关节 4        地址 3 → 关节 5

   关节号 = 地址 + ZDT_JOINT_OFFSET
   下面这些都是"关节下标"（= 关节号 - 1）：
       theta[]、arm_param.arm_theta[]、arm_param.joint_speed[]、arm_param.joint_current[]、
       joint_speed_rad_s[]（main.c 里 watch 用的副本）
   而下面这些一律是"电机下标"（= 地址 - 1）：
       MotorCurrents[]、MotorSpeedRPM*[]、can_get_motor_*()、can_zdt_read_*()
   两边别混。

   换装配（比如把电机挪到别的关节）改这一个数就行；如果三个地址不连续，
   就得改成查表，告诉我。 */
#define ZDT_JOINT_OFFSET    2U

extern void can_filter_init(void);

/* 发送一条电机命令；can_SendCmd 固定走 CAN1，USER_can_SendCmd 可指定总线 */
void can_SendCmd(uint8_t *cmd, uint32_t len);
void USER_can_SendCmd(CAN_HandleTypeDef *hcan, uint8_t *cmd, uint32_t len);

/* 请求角度：发送 0x36 读取命令，电机应答后由 CAN 中断解析并更新 MotorCurrents[] 和 theta[] */
void can_zdt_read_position(uint8_t addr);
void can_zdt_read_position_all(void);

/* 请求转速：发送 0x35 读取命令，应答后更新 MotorSpeedRPM_Meas[] 和 MotorCurrents[].dq */
void can_zdt_read_speed(uint8_t addr);
void can_zdt_read_speed_all(void);

/* 读取解析结果 */
float    can_get_motor_position(uint8_t addr);                      /* 电机角度，单位：度  (MotorCurrents[].position) */
float    can_get_motor_theta(uint8_t addr);                         /* 关节角，单位：弧度  (theta[]，Get_theta 换算) */
uint8_t  can_motor_data_valid(uint8_t addr, uint32_t timeout_ms);   /* 1 = timeout_ms 内收到过应答；电机数据本体直接读 MotorCurrents[] */

/* 电机转速（RPM，带符号），两种来源都保留、都在 watch 窗口里可直接看（下标 = 地址 - 1）：
     MotorSpeedRPM[]      0x36 位置差分估的：50Hz刷新、带低通滞后，电机不回 0x35 时的备用
     MotorSpeedRPM_Meas[] 0x35 直接问电机要的实测值：准、无差分滞后，推荐用它做限速 */
float    can_get_motor_speed_rpm(uint8_t addr);          /* 差分估计值(RPM) */
float    can_get_motor_speed_meas_rpm(uint8_t addr);     /* 0x35 实测值(RPM) */
uint8_t  can_motor_speed_valid(uint8_t addr, uint32_t timeout_ms);   /* 1 = timeout_ms 内收到过 0x35 应答 */
extern float    MotorSpeedRPM[ZDT_MOTOR_NUM];
extern float    MotorSpeedRPM_Meas[ZDT_MOTOR_NUM];
extern uint32_t MotorSpeedTim[ZDT_MOTOR_NUM];            /* 实测值的时间戳(ms) */
extern uint8_t  MotorSpeedValid[ZDT_MOTOR_NUM];          /* 收到过 0x35 应答就置 1 */
/* 关节角速度(rad/s)在 MotorCurrents[addr-1].dq（已按各关节方向取号，可直接当关节速度用） */

/* ===== CAN1 自诊断计数器（详见 bsp_can.c 里的说明）=====
   联调时把下面这些加进 Keil 的 watch 窗口：
     tx_frame_ok 每 20ms +7；一直涨但 tx_acked 一直是 0 → 帧发出去了没人 ACK
     tx_terr ≈ tx_frame_ok → 每帧都发失败（典型：没人应答 / 线没接通）
     tx_acked 也在涨 → 总线上有人应答，通信是通的
     tx_fail 一直涨 → HAL 就没发出去（外设没 Start / 邮箱卡死）
     rx_frames 涨 → 收到帧了（电机在回话）                            */
extern volatile uint32_t can1_tx_frame_ok;
extern volatile uint32_t can1_tx_acked;
extern volatile uint32_t can1_tx_terr;
extern volatile uint32_t can1_tx_fail;
extern volatile uint32_t can1_tx_slow;
extern volatile uint32_t can1_last_hal_err;
extern volatile uint32_t can1_last_tsr;
extern volatile uint32_t can1_last_esr;
extern volatile uint32_t can1_rx_frames;

/* ===== “theta[] 为什么读不到角度”专用分层计数（详见 bsp_can.c）=====
   theta[] 只有 CAN 接收中断里的 Get_theta() 会写，所以：
     rx_frames      = 0            → 帧没进 CAN1 的 FIFO（滤波器/CAN2SB、中断、总线）
     rx_data_frames = 0 而上面在涨 → 收到的都是远程帧
     rej_func / rej_tail / rej_addr / rej_len 在涨 → 帧进来了但被解析层丢掉（看 last_rx_*）
     rx_ok 在涨                    → 位置已写进 MotorCurrents[]/theta[]（theta 就有值）
     rx_ok_vel 在涨                → 转速已写进 MotorSpeedRPM_Meas[] / MotorCurrents[].dq
     last_rx_id / last_rx_dlc / last_rx_buf[] ← 最后一帧的原始样子 */
extern volatile uint32_t can1_rx_data_frames;
extern volatile uint32_t can1_rx_rej_len;
extern volatile uint32_t can1_rx_rej_func;
extern volatile uint32_t can1_rx_rej_addr;
extern volatile uint32_t can1_rx_rej_tail;
extern volatile uint32_t can1_rx_ok;
extern volatile uint32_t can1_rx_ok_vel;
extern volatile uint32_t can1_last_rx_id;
extern volatile uint32_t can1_last_rx_dlc;
extern volatile uint8_t  can1_last_rx_buf[8];

#endif
