#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "main.h"

/* ==========================================================================
 *  张大头(ZDT) X42_V2 闭环步进电机 CAN 通信
 *  电机地址范围 1 ~ ZDT_MOTOR_NUM，所有数组下标统一使用 (地址 - 1)
 *
 *  电机数据本体用的是 ZDT 模块的 MotorCurrentInfo_t（见 ZDT_X42_V2.h）：
 *      MotorCurrents[]  定义在 ZDT_X42_V2.c，本文件的 CAN 中断负责刷新
 *      theta[]          关节角(rad)，由 Get_theta() 按各关节映射换算
 * ========================================================================*/
#define ZDT_MOTOR_NUM       6U       /* 挂载的张大头电机数量 */
#define ZDT_FUNC_CPOS       0x36U    /* 功能码：读取电机实时位置(角度) */
#define ZDT_FUNC_VEL        0x35U    /* 功能码：读取电机实时转速 */
#define ZDT_FRAME_TAIL      0x6BU    /* 帧尾校验字节 */

extern void can_filter_init(void);

/* 发送一条电机命令；can_SendCmd 固定走 CAN1，USER_can_SendCmd 可指定总线 */
void can_SendCmd(uint8_t *cmd, uint32_t len);
void USER_can_SendCmd(CAN_HandleTypeDef *hcan, uint8_t *cmd, uint32_t len);

/* 请求角度：发送 0x36 读取命令，电机应答后由 CAN 中断解析并更新 MotorCurrents[] 和 theta[] */
void can_zdt_read_position(uint8_t addr);
void can_zdt_read_position_all(void);

/* 读取解析结果 */
float    can_get_motor_position(uint8_t addr);                      /* 电机角度，单位：度  (MotorCurrents[].position) */
float    can_get_motor_theta(uint8_t addr);                         /* 关节角，单位：弧度  (theta[]，Get_theta 换算) */
uint8_t  can_motor_data_valid(uint8_t addr, uint32_t timeout_ms);   /* 1 = timeout_ms 内收到过应答；电机数据本体直接读 MotorCurrents[] */

#endif
