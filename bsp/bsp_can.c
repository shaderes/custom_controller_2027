#include "bsp_can.h"
#include "main.h"
#include "ZDT/ZDT_X42_V2.h"
#include "string.h"

/* ============================================================================
 *  张大头(ZDT) X42_V2 闭环步进电机 CAN 通信 —— 读取电机角度 / 转速
 *
 *  主机发送（扩展帧，ID = (地址 << 8) | 包序号）：
 *      data[0] = 0x36   功能码（读取电机实时位置，S_CPOS）
 *      data[1] = 0x6B   校验字节
 *  或：
 *      data[0] = 0x35   功能码（读取电机实时转速，S_VEL）
 *      data[1] = 0x6B   校验字节
 *
 *  电机应答（数据帧）：
 *      0x36 位置：功能码 + 符号(1B) + 位置(4B 大端, 单位 0.1°) + 0x6B   → 7 字节
 *      0x35 转速：功能码 + 符号(1B) + 转速(2B 大端, 单位 RPM)  + 0x6B   → 5 字节
 *  角度(度) = ±(int32)位置 / 10        转速(RPM) = ±(int16)转速
 *
 *  为了不因为固件差异白烧一遍板子，解析层做成"宽容"的（见 zdt_parse_frame）：
 *      · 功能码在 buf[0]（数据段不带地址）或 buf[1]（数据段带一个地址字节）都能认
 *      · 有没有那个符号字节都能认（4B 位置/2B 转速按有符号补码解释）
 *      · 帧尾按最后一字节判，末尾多几个补齐的 0 也不影响
 *  万一还是对不上，就看 can1_last_rx_dlc / can1_last_rx_buf[] 里电机的原样，
 *  以及 can1_rx_rej_* 是哪一个在涨。
 *
 *  解析结果写进 ZDT 模块的电机数据结构（下标 = 电机地址 - 1）：
 *      MotorCurrents[index].position   ← 电机角度，单位：度        （0x36）
 *      MotorCurrents[index].dir        ← 方向字节（0x00 正 / 0x01 负）
 *      MotorCurrents[index].timestamp  ← 数据更新时间             （0x36）
 *      MotorCurrents[index].valid      ← 收到过位置应答就置 1      （0x36）
 *      MotorCurrents[index].dq         ← 关节角速度(rad/s)         （0x35 换算）
 *      然后调用 zdt_update_theta() 按“地址→关节”映射刷新 theta[]
 *      （关节下标 = 电机下标 + ZDT_JOINT_OFFSET，见 zdt_update_theta）
 *
 *  两种转速来源，都在 watch 窗口里可直接看（下标 = 地址 - 1）：
 *      MotorSpeedRPM[index]      差分估计：拿 0x36 位置前后两帧差分算的，50Hz 更新、
 *                                自带一阶低通，有量化噪声；电机不回 0x35 时的备用
 *      MotorSpeedRPM_Meas[index] 实测：0x35 直接问电机要的，电机自己算的转速，
 *                                更准、没有差分滞后，推荐用它做限速/阻尼
 *
 *  关于 CAN ID：电机地址就是 1 ~ ZDT_MOTOR_ONLINE（本工程 = 1~3）。取地址时三种约定都兼容，
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

/* ============================================================================
 *  CAN1 自诊断计数器 —— 联调时把下面几个变量加进 Keil 的 watch 窗口看数字，
 *  不用示波器、不用分析仪也能判断“到底卡在哪一层”：
 *
 *    can1_tx_frame_ok  已经交给外设并等到发送结束的帧数（每 20ms 应该 +7）
 *    can1_tx_acked     其中“被总线上某个节点 ACK 了”的帧数（TSR.TXOK 置位）
 *                      → tx_frame_ok 一直涨、acked 一直是 0：帧确实发上总线了，
 *                        但没人应答 = 波特率不对 / 没接上 / 电机没上电 / A-CAN 线反了
 *                      → acked 也在涨：总线上有人应答（分析仪或电机），
 *                        那么“分析仪没数据”就是分析仪那边设置的事
 *    can1_tx_terr      发送失败的帧数（TSR.TERR 置位）。跟 acked 对照着看：
 *                        frame_ok ≈ acked + terr，terr 全占 = 帧一帧都没被 ACK 过
 *    can1_tx_fail      HAL_CAN_AddTxMessage 返回非 HAL_OK 的次数
 *                      → 一直涨说明外设根本没在发（没 Start / 邮箱卡死）
 *    can1_tx_slow      等发送邮箱空闲超时的次数
 *    can1_last_hal_err 最后一次失败的返回值：1=HAL_ERROR 2=HAL_BUSY 3=HAL_TIMEOUT
 *    can1_last_tsr     最后一次发送后的 TSR 寄存器快照
 *                        bit0 RQCP0(请求完成) / bit1 TXOK0(发送成功) / bit3 TERR0(发送错误)
 *                        bit26~28 TME0~2（=1 表示邮箱全空）
 *    can1_last_esr     最后一次发送后的 ESR 寄存器快照：
 *                        bit4 BOFF=1 总线关闭；TEC 在 bit23:16；REC 在 bit31:24
 *                        TEC 涨到 128/255 = 一直没人 ACK
 *    can1_rx_frames    CAN1 收到的帧数（电机在回话就会涨；
 *                      开了自环自测(CAN1_SELFTEST_LOOPBACK)时收到的是自己发的帧）
 * ==========================================================================*/
volatile uint32_t can1_tx_frame_ok  = 0;
volatile uint32_t can1_tx_acked     = 0;
volatile uint32_t can1_tx_terr      = 0;
volatile uint32_t can1_tx_fail      = 0;
volatile uint32_t can1_tx_slow      = 0;
volatile uint32_t can1_last_hal_err = 0;
volatile uint32_t can1_last_tsr     = 0;
volatile uint32_t can1_last_esr     = 0;
volatile uint32_t can1_rx_frames    = 0;

/* ============================================================================
 *  CAN1 接收侧分层计数 —— 用来回答“theta[] 为什么是 0”
 *
 *  theta[] 只有 zdt_parse_frame() 里那一句 Get_theta() 会写，
 *  转速只有同一个函数里 0x35 那个分支会写，
 *  所以 theta 全 0 / 转速不动 ⇔ 一帧都没走到 zdt_parse_frame() 的最后一步。
 *  下面这几个数把“卡在哪一层”直接摊开：
 *
 *    can1_rx_frames        CAN1 收到任何帧（含远程帧/别的 ID）就 +1
 *                          = 0 → 帧根本没进 FIFO：参考 CAN2SB/滤波器（见 can_filter_init）、
 *                                没开 RX0 中断、或总线上压根没有帧
 *    can1_rx_data_frames   其中是数据帧的数量
 *    can1_rx_rej_func      buf[0]/buf[1] 都不是 0x35 / 0x36（见 can1_last_rx_buf[]）
 *    can1_rx_rej_tail      最后一字节不是 0x6B（见 can1_last_rx_buf[]）
 *    can1_rx_rej_addr      CAN ID（和数据段里的地址字节）都换不出 1~6 的地址
 *    can1_rx_rej_len       功能码认出来了，但数据段长度对不上这个功能码
 *    can1_rx_ok            成功解析的位置帧(0x36)数 —— ★ 这个在涨，theta[] 就一定有值
 *    can1_rx_ok_vel        成功解析的转速帧(0x35)数 —— ★ 这个在涨，
 *                          MotorSpeedRPM_Meas[] / MotorCurrents[].dq 就一定有值
 *
 *    can1_last_rx_id / can1_last_rx_dlc / can1_last_rx_buf[]
 *                          最后一帧的原始样子，联调时最有用：
 *                          一眼就能看出电机回的 ID 是什么、DLC 几、帧尾是不是 0x6B
 * ==========================================================================*/
volatile uint32_t can1_rx_data_frames = 0;
volatile uint32_t can1_rx_rej_len     = 0;
volatile uint32_t can1_rx_rej_func    = 0;
volatile uint32_t can1_rx_rej_tail    = 0;
volatile uint32_t can1_rx_rej_addr    = 0;
volatile uint32_t can1_rx_ok          = 0;
volatile uint32_t can1_rx_ok_vel      = 0;
volatile uint32_t can1_last_rx_id     = 0xFFFFFFFFu;
volatile uint32_t can1_last_rx_dlc    = 0;
volatile uint8_t  can1_last_rx_buf[8] = {0};

/* ============================================================================
 *  电机转速（RPM，带符号；下标 = 电机地址 - 1）
 *
 *  MotorSpeedRPM[]      差分估计：不是问电机要的，而是拿 0x36 回读的位置做差分
 *                       算出来的（见 zdt_parse_frame），所以它和 MotorCurrents[]
 *                       一样是 50Hz 刷新的、并且带一点低通滞后。
 *  MotorSpeedRPM_Meas[] 实测：0x35（S_VEL，读取电机实时转速）让电机自己报的，
 *                       没有差分量化噪声和滞后。掉线判断看 MotorSpeedValid/Tim。
 *
 *  限速/阻尼用哪一个都行：要快、要准用 Meas，要"哪怕电机不回 0x35 也有值"用差分。
 * ==========================================================================*/
float MotorSpeedRPM[ZDT_MOTOR_NUM] = {0};

float    MotorSpeedRPM_Meas[ZDT_MOTOR_NUM] = {0};   /* 0x35 回读的实测转速 */
uint32_t MotorSpeedTim[ZDT_MOTOR_NUM]      = {0};   /* 该实测值的更新时间戳(ms) */
uint8_t  MotorSpeedValid[ZDT_MOTOR_NUM]    = {0};   /* 收到过 0x35 应答就置 1 */

/* 关节角速度的符号：和下面 zdt_update_theta() 里各关节的公式保持一致。
   现在三台电机装在关节 3/4/5，电机正转就是关节正方向（公式都是 +角度），
   所以三个都是 +1。哪个关节以后改成取负（-角度），对应项就改 -1。 */
static const float zdt_joint_dir[ZDT_MOTOR_ONLINE] = {1.0f, 1.0f, 1.0f};

/* ============================================================================
 *  电机角度(度) → 关节角 theta[](rad)
 *
 *  映射：地址 1 → 关节 3，地址 2 → 关节 4，地址 3 → 关节 5（见 bsp_can.h 的
 *        ZDT_JOINT_OFFSET）。关节下标 = 电机下标 + ZDT_JOINT_OFFSET。
 *
 *  为什么不直接调 ZDT 模块里的 Get_theta()：
 *      那个函数是按"电机下标 == 关节下标"写的（1 号电机 → 关节 1、3 号 → 关节 3…），
 *      本工程的三台电机装的是关节 3/4/5，映射完全不同。这里在映射表旁边重新实现，
 *      免得每次改装配都要去动 GBK 编码的厂商文件。
 *      （ZDT_X42_V2.c 里的 Get_theta() 现在没有调用者了，留着当参考。）
 *
 *  各关节的偏置/方向沿用原来那份标定：
 *      关节 3：+PI/2 - 2π 偏置    关节 4：无偏置    关节 5：无偏置
 * ==========================================================================*/
static void zdt_update_theta(uint8_t motor_index)
{
    float m = MotorCurrents[motor_index].position * PI / 180.0f;   /* 电机角度(度) → rad */

    switch (motor_index)
    {
        case 0: theta[ZDT_JOINT_OFFSET + 0U] = PI / 2.0f + m - 2.0f * PI; break;  /* 1 号电机 → 关节 3 */
        case 1: theta[ZDT_JOINT_OFFSET + 1U] = m;                         break;  /* 2 号电机 → 关节 4 */
        case 2: theta[ZDT_JOINT_OFFSET + 2U] = m;                         break;  /* 3 号电机 → 关节 5 */
        default: break;
    }
}

/* 地址合法性检查：只有实际在线的电机（地址 1 ~ ZDT_MOTOR_ONLINE）能映射到数组下标。
   数组本身是按容量 ZDT_MOTOR_NUM 开的，所以这里用 ONLINE 只是"少发无用请求/少算空关节"，
   不会越界（ONLINE <= NUM）。 */
static uint8_t zdt_addr_check(uint8_t addr)
{
    return ((addr >= 1) && (addr <= ZDT_MOTOR_ONLINE)) ? 1 : 0;
}

/* ============================================================================
 *  滤波器初始化
 *
 *  【必须两个成员都显式赋值，别删】HAL_CAN_ConfigFilter() 每次调用都会把
 *  SlaveStartFilterBank 写进 CAN_FMR 的 CAN2SB 位（见 stm32f4xx_hal_can.c:
 *  CLEAR_BIT(FMR, CAN_FMR_CAN2SB); SET_BIT(FMR, SlaveStartFilterBank << 8);）。
 *  CAN2SB 决定 28 个滤波器组里哪些归 CAN1、哪些归 CAN2：
 *      CAN1 用 0 ~ CAN2SB-1，CAN2 用 CAN2SB ~ 27
 *
 *  原来这个结构体是 `CAN_FilterTypeDef can_filter_st;`（没初始化），
 *  而 SlaveStartFilterBank 是在 CAN2 那一段才赋值的 —— 也就是说配置 CAN1 时
 *  传进去的是栈上的垃圾值。垃圾值低 6 位一旦为 0，CAN2SB=0，CAN1 就一个
 *  滤波器组都不剩：**发送照常（can1_tx_acked 会涨，电机也真的回话了），
 *  但 CAN1 一帧都收不到** → zdt_parse_frame 不被调用 → theta[] 永远是 0。
 *  这是个随栈内容变化的偶发故障，所以这里改成 = {0} 并显式写 14。
 * ==========================================================================*/
void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st = {0};

    /* CAN1：用 0 ~ 13 号滤波组，全接收（掩码 0 = 标准帧/扩展帧都要） */
    can_filter_st.SlaveStartFilterBank = 14;   /* CAN2 起始组，必须在第一次 ConfigFilter 之前写 */
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

    /* CAN2：用 14 ~ 27 号滤波组（SlaveStartFilterBank 保持 14） */
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

        if (can_wait_mailbox(hcan, ZDT_TX_TIMEOUT_MS) != 0)
        {
            if (hcan->Instance == CAN1) can1_tx_slow++;
            break;
        }

        {
            HAL_StatusTypeDef tx_st = HAL_CAN_AddTxMessage(hcan, &pTxHeader, send_buffer, &mailbox);

            if (tx_st != HAL_OK)
            {
                if (hcan->Instance == CAN1)
                {
                    can1_tx_fail++;
                    can1_last_hal_err = (uint32_t)tx_st;   /* 1=ERROR 2=BUSY 3=TIMEOUT */
                }
                break;                   /* 失败就整条放弃，避免发出半条命令 */
            }
        }

        can_wait_tx_done(hcan, mailbox, ZDT_TX_TIMEOUT_MS);  /* 等这一包上总线，再发下一包 */

        if (hcan->Instance == CAN1)
        {
            /* 快照寄存器。TSR 里每个邮箱有 4 个标志位：
                 RQCP(请求完成) / TXOK(发送成功) / ALST(仲裁丢失) / TERR(发送错误)
               位号：邮箱0 = bit0~3，邮箱1 = bit8~11，邮箱2 = bit16~19
               （注意 TXOK0 是 bit1、RQCP0 才是 bit0，别搞混） */
            static const uint32_t txok_bit[3] = { CAN_TSR_TXOK0, CAN_TSR_TXOK1, CAN_TSR_TXOK2 };
            static const uint32_t terr_bit[3] = { CAN_TSR_TERR0, CAN_TSR_TERR1, CAN_TSR_TERR2 };
            uint32_t mb = (mailbox < 3U) ? mailbox : 0U;

            can1_last_tsr = hcan->Instance->TSR;
            can1_last_esr = hcan->Instance->ESR;
            can1_tx_frame_ok++;
            if ((can1_last_tsr & txok_bit[mb]) != 0U) can1_tx_acked++;   /* 发出去且被 ACK 了 */
            if ((can1_last_tsr & terr_bit[mb]) != 0U) can1_tx_terr++;    /* 这一帧发失败了 */
        }

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

/* ============================================================================
 *  应答帧的宽容解析（角度 + 转速共用）
 * ==========================================================================*/

/**
  * @brief    找应答帧里的功能码位置
  * @param    buf  ：应答数据
  * @param    dlc  ：应答长度
  * @param    func ：输出，找到的功能码（0x35 / 0x36）
  * @retval   功能码所在的下标(0 或 1)；不认识的帧返回 0xFF
  * @note     正常固件的数据段是"功能码 + 数据 + 0x6B"（功能码在下标 0）；
  *           有些固件的 CAN 应答数据段里还带一个地址字节，
  *           变成"地址 + 功能码 + 数据 + 0x6B"（功能码在下标 1）。
  *           0x35/0x36 都远大于地址范围 1~6，所以这两种情况不会判歧义。
  */
static uint8_t zdt_find_func(const uint8_t *buf, uint32_t dlc, uint8_t *func)
{
    if (dlc < 2U) return 0xFFU;

    if (((buf[1] == ZDT_FUNC_CPOS) || (buf[1] == ZDT_FUNC_VEL)) &&
        (buf[0] >= 1U) && (buf[0] <= ZDT_MOTOR_NUM))
    {
        *func = buf[1];
        return 1U;                                  /* 数据段 = 地址 + 功能码 + ... */
    }

    if ((buf[0] == ZDT_FUNC_CPOS) || (buf[0] == ZDT_FUNC_VEL))
    {
        *func = buf[0];
        return 0U;                                  /* 数据段 = 功能码 + ... */
    }

    return 0xFFU;
}

/**
  * @brief    从 CAN ID 抠出电机地址
  * @param    id  ：应答帧 CAN ID
  * @param    ide ：CAN_ID_STD / CAN_ID_EXT
  * @retval   地址(1 ~ ZDT_MOTOR_NUM)；抠不出来返回 0
  * @note     标准帧：ID 直接就是地址；
  *           扩展帧：ID 直接是地址(1~6)，或者 ID = (地址 << 8) | 包序号（取高字节）。
  */
static uint32_t zdt_addr_from_id(uint32_t id, uint32_t ide)
{
    uint32_t addr32 = 0;

    if (ide == CAN_ID_STD)
    {
        if ((id >= 1U) && (id <= ZDT_MOTOR_NUM)) addr32 = id;
    }
    else
    {
        if ((id >= 1U) && (id <= ZDT_MOTOR_NUM))                     addr32 = id;
        else if (((id >> 8) >= 1U) && ((id >> 8) <= ZDT_MOTOR_NUM))  addr32 = id >> 8;
    }

    return addr32;
}

/**
  * @brief    解析一帧电机应答：0x36 位置 → theta[]，0x35 转速 → dq/实测转速
  * @param    hcan：收到这帧的总线（用来决定要不要累加 can1_* 诊断计数）
  * @param    id  ：应答帧 CAN ID
  * @param    ide ：CAN_ID_STD / CAN_ID_EXT
  * @param    buf ：应答数据
  * @param    dlc ：应答数据长度
  * @retval   none
  * @note     每一次 return 都对应一个 can1_rx_rej_* 计数；theta[]/转速读不到时，
  *           把这些计数和 can1_last_rx_* 一起看就知道卡在哪一层。
  *           数据段长度判断都用 >=：末尾多几个补齐的 0 不影响取值。
  */
static void zdt_parse_frame(CAN_HandleTypeDef *hcan, uint32_t id, uint32_t ide,
                            const uint8_t *buf, uint32_t dlc)
{
    uint8_t  func  = 0U;
    uint8_t  off;
    uint8_t  index;
    uint32_t addr32;
    uint32_t payload;                              /* 功能码之后、帧尾之前的字节数 */
    uint8_t  is_can1 = (hcan->Instance == CAN1) ? 1U : 0U;

    if (is_can1 != 0U)   /* 留一份原始帧快照，联调时看这个最直观 */
    {
        can1_last_rx_id  = id;
        can1_last_rx_dlc = dlc;
        for (uint32_t i = 0; i < 8U; i++) can1_last_rx_buf[i] = buf[i];
    }

    off = zdt_find_func(buf, dlc, &func);
    if (off == 0xFFU)
    {
        if (is_can1 != 0U) can1_rx_rej_func++;
        return;
    }
    if (buf[dlc - 1U] != ZDT_FRAME_TAIL)           /* 帧尾校验字节必须在最后一字节 */
    {
        if (is_can1 != 0U) can1_rx_rej_tail++;
        return;
    }

    payload = dlc - (uint32_t)off - 2U;            /* 减掉功能码本身和帧尾 */

    addr32 = zdt_addr_from_id(id, ide);
    if ((addr32 == 0U) && (off == 1U)) addr32 = buf[0];   /* 退回数据段里的地址字节 */
    /* 只认实际在线的地址 1 ~ ZDT_MOTOR_ONLINE；别的地址（比如总线上别人的帧、
       或者没装电机的 4~6 号）一律丢掉并记一笔，方便发现地址配错 */
    if ((addr32 < 1U) || (addr32 > ZDT_MOTOR_ONLINE))
    {
        if (is_can1 != 0U) can1_rx_rej_addr++;
        return;
    }

    index = (uint8_t)(addr32 - 1U);

    if (func == ZDT_FUNC_CPOS)
    {
        /* ---------- 0x36：位置(0.1°，有符号) ---------- */
        int32_t raw;

        if (payload >= 5U)          /* 符号(1B) + 位置(4B 大端) */
        {
            raw = ((int32_t)buf[off + 2U] << 24) | ((int32_t)buf[off + 3U] << 16) |
                  ((int32_t)buf[off + 4U] << 8)  |  (int32_t)buf[off + 5U];
            if (buf[off + 1U] != 0U) raw = -raw;
            MotorCurrents[index].dir = buf[off + 1U];
        }
        else if (payload == 4U)     /* 没有符号字节：位置当有符号 32 位补码 */
        {
            raw = ((int32_t)buf[off + 1U] << 24) | ((int32_t)buf[off + 2U] << 16) |
                  ((int32_t)buf[off + 3U] << 8)  |  (int32_t)buf[off + 4U];
            MotorCurrents[index].dir = (raw < 0) ? 1U : 0U;
        }
        else
        {
            if (is_can1 != 0U) can1_rx_rej_len++;
            return;
        }

        /* ---- 顺便用这次位置和上次位置差分估一个转速（0x35 的对照/备用）----
           位置是主循环 50Hz 轮询回来的，协议分辨率 0.1°，所以差分出来的转速有量化噪声，
           这里做一阶低通压一压。转速 = Δ角度(°) ÷ 360 × 60 ÷ Δt(s) */
        {
            uint32_t now    = HAL_GetTick();
            float    newpos = (float)raw / 10.0f;

            if (MotorCurrents[index].valid != 0U)
            {
                uint32_t dt = now - MotorCurrents[index].timestamp;

                if ((dt >= 1U) && (dt <= 200U))
                {
                    float d_deg = newpos - MotorCurrents[index].position;

                    /* 位置是 32 位累加值，万一翻转会跳出一个荒唐的差值，直接丢掉 */
                    if ((d_deg > -3600.0f) && (d_deg < 3600.0f))
                    {
                        float rpm = d_deg * 60.0f / (360.0f * ((float)dt * 0.001f));
                        MotorSpeedRPM[index] += (rpm - MotorSpeedRPM[index]) * 0.5f;   /* 一阶低通 */
                    }
                }
            }
            else
            {
                MotorSpeedRPM[index] = 0.0f;
            }
        }

        /* 写入 ZDT 模块的电机数据结构（MotorCurrentInfo_t） */
        MotorCurrents[index].id        = (uint8_t)addr32;
        MotorCurrents[index].position  = (float)raw / 10.0f;   /* 协议把角度放大了 10 倍 */
        MotorCurrents[index].rad       = MotorCurrents[index].position * DEG_TO_RAD;
        MotorCurrents[index].timestamp = HAL_GetTick();
        MotorCurrents[index].valid     = 1U;

        /* 电机角度 → 关节角 theta[]（下标 = 关节号 - 1，见 zdt_update_theta） */
        zdt_update_theta(index);

        if (is_can1 != 0U) can1_rx_ok++;    /* ★ 这个在涨，theta[] 就一定有值 */
    }
    else
    {
        /* ---------- 0x35：电机自己报的实时转速(RPM，有符号) ---------- */
        int32_t v;

        if (payload >= 3U)          /* 符号(1B) + 转速(2B 大端) */
        {
            v = ((int32_t)buf[off + 2U] << 8) | (int32_t)buf[off + 3U];
            if (buf[off + 1U] != 0U) v = -v;
        }
        else if (payload == 2U)     /* 没有符号字节：转速当有符号 16 位补码 */
        {
            v = (int32_t)(int16_t)(((uint16_t)buf[off + 1U] << 8) | (uint16_t)buf[off + 2U]);
        }
        else
        {
            if (is_can1 != 0U) can1_rx_rej_len++;
            return;
        }

        MotorSpeedRPM_Meas[index] = (float)v;      /* 电机轴转速，单位 RPM */
        MotorSpeedTim[index]      = HAL_GetTick();
        MotorSpeedValid[index]    = 1U;

        /* 电机轴转速(RPM) → 关节角速度(rad/s)：×2π/60，再按关节方向取符号，
           写进 MotorCurrents[].dq（那个成员本来就是"当前关节速度 rad/s"）。
           注意这里只是"读出来给你用"，没有拿它去改变力矩输出。 */
        MotorCurrents[index].dq = (float)v * (2.0f * PI / 60.0f) * zdt_joint_dir[index];

        if (is_can1 != 0U) can1_rx_ok_vel++;   /* ★ 这个在涨，转速就一定有值 */
    }
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

        if (hcan->Instance == CAN1) can1_rx_frames++;        /* 诊断计数：收到帧就 +1 */

        if (rx_header.RTR != CAN_RTR_DATA) continue;         /* 只处理数据帧 */

        if (hcan->Instance == CAN1) can1_rx_data_frames++;

        id = (rx_header.IDE == CAN_ID_EXT) ? rx_header.ExtId : rx_header.StdId;
        zdt_parse_frame(hcan, id, rx_header.IDE, rx_buf, rx_header.DLC);
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
  * @brief    依次读取 1 ~ ZDT_MOTOR_ONLINE 号电机的实时位置
  * @param    none
  * @retval   none
  */
void can_zdt_read_position_all(void)
{
    for (uint8_t addr = 1; addr <= ZDT_MOTOR_ONLINE; addr++)
    {
        can_zdt_read_position(addr);
    }
}

/**
  * @brief    发送一条“读取电机实时转速”命令
  * @param    addr ：电机地址(1 ~ ZDT_MOTOR_NUM)
  * @retval   none
  * @note     和读位置一样交给 ZDT 模块拼命令：S_VEL 会被换成线上功能码 0x35。
  *           电机应答后由 CAN 中断解析，写进 MotorSpeedRPM_Meas[] 和
  *           MotorCurrents[].dq（关节角速度 rad/s）。
  */
void can_zdt_read_speed(uint8_t addr)
{
    if (zdt_addr_check(addr) == 0) return;

    USER_ZDT_X42_V2_Read_Sys_Params(&hcan1, addr, S_VEL);
}

/**
  * @brief    依次读取 1 ~ ZDT_MOTOR_ONLINE 号电机的实时转速
  * @param    none
  * @retval   none
  * @note     和 can_zdt_read_position_all() 一样是 50Hz 轮询用的：
  *           位置 + 转速两条命令各 ZDT_MOTOR_ONLINE 帧（3 台电机 = 6 帧/20ms，
  *           500kbps 下不到 1.5ms），主循环里连着调就行。
  */
void can_zdt_read_speed_all(void)
{
    for (uint8_t addr = 1; addr <= ZDT_MOTOR_ONLINE; addr++)
    {
        can_zdt_read_speed(addr);
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
  * @brief    获取电机转速估计值（0x36 位置差分算的）
  * @param    addr ：电机地址(1 ~ ZDT_MOTOR_NUM)
  * @retval   转速，单位：RPM（带符号，正=协议里的正方向）；地址非法时返回 0
  * @note     由 0x36 位置回读差分得到，需要电机在回话（MotorCurrents[].valid 为 1）
  *           且至少收到两帧之后才有意义；掉线时不会更新（用 can_motor_data_valid 判断）。
  *           要更准更快的值用 can_get_motor_speed_meas_rpm()（0x35 实测）。
  */
float can_get_motor_speed_rpm(uint8_t addr)
{
    if (zdt_addr_check(addr) == 0) return 0.0f;

    return MotorSpeedRPM[addr - 1];
}

/**
  * @brief    获取电机实测转速（0x35 直接读的）
  * @param    addr ：电机地址(1 ~ ZDT_MOTOR_NUM)
  * @retval   转速，单位：RPM（带符号）；地址非法时返回 0
  * @note     掉线时不会更新，用 can_motor_speed_valid() 判断新鲜度；
  *           关节角速度(rad/s)直接用 MotorCurrents[addr-1].dq。
  */
float can_get_motor_speed_meas_rpm(uint8_t addr)
{
    if (zdt_addr_check(addr) == 0) return 0.0f;

    return MotorSpeedRPM_Meas[addr - 1];
}

/**
  * @brief    判断实测转速是否新鲜
  * @param    addr       ：电机地址(1 ~ ZDT_MOTOR_NUM)
  * @param    timeout_ms ：超时时间(ms)
  * @retval   1 = timeout_ms 内收到过 0x35 应答，0 = 没收到过/掉线/地址非法
  */
uint8_t can_motor_speed_valid(uint8_t addr, uint32_t timeout_ms)
{
    uint8_t index;

    if (zdt_addr_check(addr) == 0) return 0;

    index = (uint8_t)(addr - 1);
    if (MotorSpeedValid[index] == 0) return 0;

    return (HAL_GetTick() - MotorSpeedTim[index]) <= timeout_ms;
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
