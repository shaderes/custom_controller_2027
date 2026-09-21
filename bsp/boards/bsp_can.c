#include "bsp_can.h"
#include "main.h"
#include "ZDT/ZDT_X42_V2.h"
#include "string.h"

/* ============================================================================
 *  张大头(ZDT) X42_V2 闭环步进电机 CAN 通信 —— 获取电机角度
 *
 *  主机发送（扩展帧，ID = (地址 << 8) | 包序号）：
 *      data[0] = 0x36   功能码（读取电机实时位置）
 *      data[1] = 0x6B   校验字节
 *
 *  电机应答（数据帧，共 7 字节）：
 *      buf[0]    = 0x36            功能码
 *      buf[1]    = 0x00 正 / 0x01 负
 *      buf[2..5] = 位置值，大端序，实际角度放大 10 倍（单位 0.1°）
 *      buf[6]    = 0x6B            校验字节
 *
 *  角度(度) = ±(int32)buf[2..5] / 10
 *
 *  解析结果写进 ZDT 模块的电机数据结构（下标 = 电机地址 - 1）：
 *      MotorCurrents[index].position   ← 电机角度，单位：度
 *      MotorCurrents[index].dir        ← 方向字节（0x00 正 / 0x01 负）
 *      MotorCurrents[index].timestamp  ← 数据更新时间
 *      MotorCurrents[index].valid      ← 收到过应答就置 1
 *      然后调用 Get_theta(MotorCurrents, index) 刷新关节角 theta[]（单位：rad）
 *
 *  关于 CAN ID：电机地址就是 1 ~ ZDT_MOTOR_NUM（1~6）。取地址时三种约定都兼容，
 *  不用按固件版本改代码：
 *      标准帧，ID 直接是地址
 *      扩展帧，ID 直接是地址（也是 1~6）
 *      扩展帧，ID = (地址 << 8) | 包序号
 *  （滤波器掩码全 0，标准帧/扩展帧一律接收，见 can_filter_init）
 *
 *  注意：HAL 库只有 HAL_CAN_RxFifo0MsgPendingCallback 一个接收入口
 *  （USE_HAL_CAN_REGISTER_CALLBACKS 为 0），本工程由 bsp_can.c 独占，
 *  原来定义该回调的 application/CAN_receive.c（DJI 旧代码）已从 Keil 工程移除。
 * ==========================================================================*/

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#define ZDT_TX_TIMEOUT_MS   5      /* 等待发送邮箱空闲/发送完成的最长时间(ms) */

/* 主机命令帧的 CAN ID 约定（电机地址范围 1 ~ ZDT_MOTOR_NUM，即 1~6）：
 *   0 = 扩展帧，ID = (地址 << 8) | 包序号   ← ZDT 官方 X42_V2 例程的写法，默认
 *   1 = 标准帧，ID = 地址(1~6)
 * 发完收不到应答时，就改这个数再试。 */
#define ZDT_CMD_ID_IS_ADDR  0

/* 电机数据全部存在 MotorCurrents[]（ZDT 模块的 MotorCurrentInfo_t），本文件不再另开数组 */

/* 地址合法性检查：电机地址 1 ~ ZDT_MOTOR_NUM 才能映射到数组下标 */
static uint8_t zdt_addr_check(uint8_t addr)
{
    return ((addr >= 1) && (addr <= ZDT_MOTOR_NUM)) ? 1 : 0;
}

void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;   /* 掩码全 0：标准帧/扩展帧全部接收 */
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

}

/* 等待发送邮箱空闲（bxCAN 只有 3 个邮箱，满了会直接失败） */
static uint8_t can_wait_mailbox(CAN_HandleTypeDef *hcan, uint32_t timeout_ms)
{
    uint32_t tick = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
    {
        if ((HAL_GetTick() - tick) > timeout_ms) return 1;   /* 超时 */
    }
    return 0;
}

/* 等待本包真正发完，保证多包顺序（ZDT 也要求包间留间隔） */
static void can_wait_tx_done(CAN_HandleTypeDef *hcan, uint32_t mailbox, uint32_t timeout_ms)
{
    uint32_t tick = HAL_GetTick();
    while (HAL_CAN_IsTxMessagePending(hcan, mailbox))
    {
        if ((HAL_GetTick() - tick) > timeout_ms) return;
    }
}

/**
  * @brief    在指定 CAN 总线上发送一条张大头电机命令
  * @param    hcan ：CAN 句柄
  * @param    cmd  ：地址 + 功能码 + 数据 + 校验字节(0x6B)
  * @param    len  ：cmd 的有效长度
  * @retval   none
  * @note     cmd[0] 是电机地址，cmd[1] 是功能码；超长命令自动拆成多包，
  *           扩展帧 ID = (地址 << 8) | 包序号。
  */
static void can_send_cmd_on(CAN_HandleTypeDef *hcan, const uint8_t *cmd, uint32_t len)
{
    uint32_t i = 0, l = 0, packNum = 0;
    uint32_t j, k;
    uint8_t  send_buffer[8] = {0};
    CAN_TxHeaderTypeDef pTxHeader = {0};
    uint32_t mailbox;

    if (len < 2) return;                /* 至少要有 地址 + 功能码 */
    j = len - 2;                        /* 要搬运的数据字节数 */

    while (i < j)
    {
        k = j - i;                       /* 本包开始前剩余数据量 */

        /* ---- 帧头：见文件顶部的 ZDT_CMD_ID_IS_ADDR 说明 ---- */
#if (ZDT_CMD_ID_IS_ADDR == 1)
        pTxHeader.StdId              = (uint32_t)cmd[0];      /* 标准帧，ID 就是地址 */
        pTxHeader.IDE                = CAN_ID_STD;
#else
        pTxHeader.ExtId              = ((uint32_t)cmd[0] << 8) | packNum;  /* 高字节地址，低字节包序号 */
        pTxHeader.IDE                = CAN_ID_EXT;            /* 扩展帧 */
#endif
        pTxHeader.RTR                = CAN_RTR_DATA;    /* 数据帧 */
        pTxHeader.TransmitGlobalTime = DISABLE;

        send_buffer[0] = cmd[1];         /* 每包都带功能码 */

        if (k < 8)                      /* 最后一包 */
        {
            for (l = 0; l < k; l++, i++) send_buffer[l + 1] = cmd[i + 2];
            pTxHeader.DLC = k + 1;      /* DLC 直接就是字节数 0~8 */
        }
        else                             /* 满包：7 数据 + 1 功能码 */
        {
            for (l = 0; l < 7U; l++, i++) send_buffer[l + 1] = cmd[i + 2];
            pTxHeader.DLC = 8U;
        }

        if (can_wait_mailbox(hcan, ZDT_TX_TIMEOUT_MS) != 0) {break; }

        if (HAL_CAN_AddTxMessage(hcan, &pTxHeader, send_buffer, &mailbox) != HAL_OK)
        {
            break;                       /* 失败就整条放弃，避免发出半条命令 */
        }

        can_wait_tx_done(hcan, mailbox, ZDT_TX_TIMEOUT_MS);  /* 等这一包上总线，再发下一包 */

        packNum++;
        memset(send_buffer, 0, sizeof(send_buffer));
    }
}

/* 默认使用 CAN1 发送（张大头电机接在 CAN1：PD0=RX / PD1=TX） */
void can_SendCmd(uint8_t *cmd, uint32_t len)
{
    can_send_cmd_on(&hcan1, cmd, len);
}

/* 需要走其它总线时用这个（ZDT_X42_V2.c 里的 USER_ 系列命令会调用） */
void USER_can_SendCmd(CAN_HandleTypeDef *hcan, uint8_t *cmd, uint32_t len)
{
    can_send_cmd_on(hcan, cmd, len);
}

/**
  * @brief    解析一帧电机应答：写进 MotorCurrents[] 并刷新关节角 theta[]
  * @param    id  ：应答帧 CAN ID
  * @param    ide ：CAN_ID_STD / CAN_ID_EXT
  * @param    buf ：应答数据
  * @param    dlc ：应答数据长度
  * @retval   none
  */
static void zdt_parse_position(uint32_t id, uint32_t ide, const uint8_t *buf, uint32_t dlc)
{
    uint8_t  addr;
    uint8_t  index;
    uint32_t addr32;
    int32_t  raw;

    if (dlc != 7) return;                                 /* 0x36 应答固定 7 字节 */
    if (buf[0] != ZDT_FUNC_CPOS || buf[6] != ZDT_FRAME_TAIL) return;   /* 无效帧 */

    /* 取电机地址：电机 CAN ID 就在 1 ~ ZDT_MOTOR_NUM（1~6）范围内。
       标准帧：ID 直接就是地址；
       扩展帧：ID 直接是地址(1~6)，或者 ID = (地址 << 8) | 包序号，此时取高字节。
       地址超出 1~6 的帧一律丢掉，不截断，避免和总线上其它设备撞号。 */
    if (ide == CAN_ID_STD)
    {
        addr32 = id;
    }
    else
    {
        addr32 = (id <= ZDT_MOTOR_NUM) ? id : (id >> 8);
    }
    if ((addr32 < 1) || (addr32 > ZDT_MOTOR_NUM)) return;

    addr = (uint8_t)addr32;

    /* 4 字节位置值，大端序（高位在前） */
    raw = ((int32_t)buf[2] << 24) |
          ((int32_t)buf[3] << 16) |
          ((int32_t)buf[4] << 8)  |
           (int32_t)buf[5];
    if (buf[1] != 0) raw = -raw;                           /* 0x01 表示负方向 */

    index = (uint8_t)(addr - 1);

    /* 写入 ZDT 模块的电机数据结构（MotorCurrentInfo_t） */
    MotorCurrents[index].id        = addr;
    MotorCurrents[index].dir       = buf[1];
    MotorCurrents[index].position  = (float)raw / 10.0f;   /* 协议把角度放大了 10 倍 */
    MotorCurrents[index].rad       = MotorCurrents[index].position * DEG_TO_RAD;
    MotorCurrents[index].timestamp = HAL_GetTick();
    MotorCurrents[index].valid     = 1;

    /* 电机角度 → 关节角 theta[]（单位：rad） */
    Get_theta(MotorCurrents, index);
}

/**
  * @brief    HAL CAN 接收中断回调
  * @param    hcan ：产生中断的 CAN 句柄
  * @retval   none
  * @note     一次中断可能积压多帧，全部取完，避免 FIFO 溢出丢帧。
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t  rx_buf[8];
    uint32_t id;

    if ((hcan->Instance != CAN1) && (hcan->Instance != CAN2)) return;

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_buf) != HAL_OK) break;

        if (rx_header.RTR != CAN_RTR_DATA) continue;         /* 只处理数据帧 */

        id = (rx_header.IDE == CAN_ID_EXT) ? rx_header.ExtId : rx_header.StdId;
        zdt_parse_position(id, rx_header.IDE, rx_buf, rx_header.DLC);
    }
}

/**
  * @brief    发送一条“读取电机实时位置”命令
  * @param    addr ：电机地址(1 ~ ZDT_MOTOR_NUM)
  * @retval   none
  * @note     交给 ZDT 模块拼命令：S_CPOS 是“读取实时位置”的枚举选择子，
  *           模块内部会把它换成线上功能码 0x36（注意 S_CPOS 本身等于 14，
  *           不能直接当成 cmd[1] 发出去）。
  */
void can_zdt_read_position(uint8_t addr)
{
    if (zdt_addr_check(addr) == 0) return;

    USER_ZDT_X42_V2_Read_Sys_Params(&hcan1, addr, S_CPOS);
}

/**
  * @brief    依次读取 1 ~ ZDT_MOTOR_NUM 号电机的实时位置
  * @param    none
  * @retval   none
  */
void can_zdt_read_position_all(void)
{
    for (uint8_t addr = 1; addr <= ZDT_MOTOR_NUM; addr++)
    {
        can_zdt_read_position(addr);
    }
}

/**
  * @brief    获取电机角度
  * @param    addr ：电机地址(1 ~ ZDT_MOTOR_NUM)
  * @retval   电机角度，单位：度（MotorCurrents[].position）；地址非法时返回 0
  */
float can_get_motor_position(uint8_t addr)
{
    if (zdt_addr_check(addr) == 0) return 0.0f;

    return MotorCurrents[addr - 1].position;
}

/**
  * @brief    获取关节角
  * @param    addr ：电机地址(1 ~ ZDT_MOTOR_NUM)
  * @retval   关节角，单位：弧度（theta[]，由 Get_theta 按各关节映射换算）；地址非法时返回 0
  */
float can_get_motor_theta(uint8_t addr)
{
    if (zdt_addr_check(addr) == 0) return 0.0f;

    return theta[addr - 1];
}

/**
  * @brief    判断电机数据是否新鲜
  * @param    addr       ：电机地址(1 ~ ZDT_MOTOR_NUM)
  * @param    timeout_ms ：超时时间(ms)
  * @retval   1 = timeout_ms 内收到过应答，0 = 掉线或地址非法
  * @note     用的是 MotorCurrents[].valid 和 .timestamp：
  *           valid 一直为 0 → 从没收到过应答（命令没发出去或接线/波特率有问题）；
  *           valid 为 1 但这里返回 0 → 收到过，但已经超过 timeout_ms 没再更新。
  */
uint8_t can_motor_data_valid(uint8_t addr, uint32_t timeout_ms)
{
    uint8_t index;

    if (zdt_addr_check(addr) == 0) return 0;

    index = (uint8_t)(addr - 1);
    if (MotorCurrents[index].valid == 0) return 0;

    return (HAL_GetTick() - MotorCurrents[index].timestamp) <= timeout_ms;
}
