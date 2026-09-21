#include "remote_control.h"
#include "main.h"
#include "string.h"
#include "detect_task.h"
#include "bsp_usart.h"
extern UART_HandleTypeDef huart10;
extern DMA_HandleTypeDef hdma_usart10_rx;
//?????????????????
#define RC_CHANNAL_ERROR_VALUE 700

//???DMA_Stream_TypeDef?????hdma.Instance
#define __HAL_DMA_GET_INSTANCE(hdma) ((DMA_Stream_TypeDef *)(hdma.Instance))

//remote control data 
//????????????
RC_ctrl_t rc_ctrl;
//????????????18??????????36????????????DMA???????
//static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

//???????
static int16_t RC_abs(int16_t value);
/**
  * @brief          ?????Э?????
  * @param[in]      sbus_buf: ??????????
  * @param[out]     rc_ctrl: ??????????
  * @retval         none
  */
//void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
void remote_control_init(void);
const RC_ctrl_t *get_remote_control_point(void);
uint8_t RC_data_is_error(void);
void slove_RC_lost(void);
void slove_data_error(void);
static int16_t RC_abs(int16_t value);
void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
static uint16_t vt_crc16(const uint8_t *data, uint8_t len);                /* CRC-16У??     */
static const uint16_t crc16_tab[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78,
};
/**
  * @brief          ??????????
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}
/**
  * @brief          ???????????????
  * @param[in]      none
  * @retval         ????????????
  */
const inline RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}
//?ж??????????????????
uint8_t RC_data_is_error(void)
{
    //?????go to??? ?????????????????????????????
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
   /* if (rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }*/
		    if (rc_ctrl.rc.mode_sw >  2)     // C??0x00??N??0x01??S??0x10
    {
        goto error;
    }
    return 0;

error:
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
  //  rc_ctrl.rc.s[0] = RC_SW_DOWN;
   // rc_ctrl.rc.s[1] = RC_SW_DOWN;
		 rc_ctrl.rc.mode_sw = RC_GEAR_C;
		 rc_ctrl.rc.key_pause=0;
    rc_ctrl.rc.key_left=0;
    rc_ctrl.rc.key_right=0;
    rc_ctrl.rc.key_trig=0;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
		rc_ctrl.mouse.press_m = 0;//?м?
    rc_ctrl.key.v = 0;
    return 1;
}

void slove_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}
void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}
/*void USART10_IRQHandler(void)
{
    // 检测空闲中断（IDLE）
    if (__HAL_UART_GET_FLAG(&huart10, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart10);   // 清除空闲标志

        static uint16_t this_time_rx_len = 0;

        判断当前 DMA 使用的是 Buffer0 还是 Buffer1（CT位） 
        if ((DMA2_Stream1->CR & DMA_SxCR_CT) == RESET)  // 当前 Buffer0
        {
            __HAL_DMA_DISABLE(&hdma_usart10_rx);        // 你的 DMA 句柄（可能是 hdma_usart10_rx）
            this_time_rx_len = SBUS_RX_BUF_NUM - DMA2_Stream1->NDTR;
            DMA2_Stream1->NDTR = SBUS_RX_BUF_NUM;       // 重置计数
            DMA2_Stream1->CR |= DMA_SxCR_CT;            // 切换到 Buffer1
            __HAL_DMA_ENABLE(&hdma_usart10_rx);

            // 调用 VT03 解析函数（队友1的 vt_to_rc，21字节，CRC校验）
            if (SBUS_TO_RC(sbus_rx_buf[0],&rc_ctrl))
            {
                detect_hook(DBUSTOE);   // 记录数据有效
            }
        }
        else  // 当前 Buffer1
        {
            __HAL_DMA_DISABLE(&hdma_usart10_rx);
            this_time_rx_len = SBUS_RX_BUF_NUM - DMA2_Stream1->NDTR;
            DMA2_Stream1->NDTR = SBUS_RX_BUF_NUM;
            DMA2_Stream1->CR &= ~(DMA_SxCR_CT);         // 切换回 Buffer0
            __HAL_DMA_ENABLE(&hdma_usart10_rx);

            if (SBUS_TO_RC(sbus_rx_buf[1],&rc_ctrl))
            {
                detect_hook(DBUSTOE);
            }
        }
    }
}*/
/*DBUS?????ж????*/
void DBUS_IDLERX_HOOK(void)
{
    static uint16_t this_time_rx_len = 0;
    this_time_rx_len = SBUS_RX_BUF_NUM - __HAL_DMA_GET_COUNTER(&DMA_DBUS_USART_RX);
    if(this_time_rx_len == RC_FRAME_LENGTH)
    {
        SBUS_TO_RC(sbus_rx_buf[0], &rc_ctrl);
        //?????????????
        DetectHook(DBUSTOE);
        HAL_UARTEx_ReceiveToIdle_DMA(&DBUS_USART,sbus_rx_buf[0],SBUS_RX_BUF_NUM);
    }
    else
    {
        //???????????????
        memset(sbus_rx_buf[0], 0, SBUS_RX_BUF_NUM);
        HAL_UARTEx_ReceiveToIdle_DMA(&DBUS_USART,sbus_rx_buf[0],SBUS_RX_BUF_NUM);
    }
}

void DBUS_ERR_HOOK(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&DBUS_USART,sbus_rx_buf[0],SBUS_RX_BUF_NUM);
    memset(sbus_rx_buf[0], 0, SBUS_RX_BUF_NUM);
}

//???????
static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}
static uint16_t vt_crc16(const uint8_t *data, uint8_t len)
{
    uint16_t crc = 0xFFFF;
    uint8_t i;
    for (i = 0; i < len; i++)
        crc = (crc >> 8) ^ crc16_tab[(crc ^ data[i]) & 0xFF];
    return crc;
}
/**
  * @brief          ?????Э?????
  * @param[in]      sbus_buf: ??????????
  * @param[out]     rc_ctrl: ??????????
  * @retval         none
  */
void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
	 uint16_t crc_calc;      /* CRC????? */
  uint16_t crc_recv;      /* CRC????? (????β2???) */
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return ;
    }
		if(sbus_buf[0] != 0xA9 || sbus_buf[1] != 0x53)
{
    //????????
    return ;
}
    crc_calc = vt_crc16((const uint8_t *)sbus_buf, 19);             /* ????CRC: ???????0~18 */
    crc_recv = sbus_buf[19] | ((uint16_t)sbus_buf[20] << 8);              /* ????CRC: С???? (????????) */
    if (crc_calc != crc_recv)
    {
        return ;   /* CRC????? ?? ??????? (??????????) */
    }

  /*  rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &
                        0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL*/
// CH0?????bit16 ?? buf[2]??0~7 + buf[3]??0~2
rc_ctrl->rc.ch[0] = (sbus_buf[2] | (sbus_buf[3] << 8)) & 0x07ff;

// CH1?????bit27 ?? buf[3]????3 + buf[4]????5
rc_ctrl->rc.ch[1] = ((sbus_buf[3] >> 3) | (sbus_buf[4] << 5)) & 0x07ff;

// CH2?????bit38 ?? buf[4]>>6 | buf[5]<<2 | buf[6]<<10
rc_ctrl->rc.ch[2] = ((sbus_buf[4] >> 6) | (sbus_buf[5] << 2) | (sbus_buf[6] << 10)) & 0x07ff;

// CH3?????bit49 ?? buf[6]>>1 | buf[7]<<7
rc_ctrl->rc.ch[3] = ((sbus_buf[6] >> 1) | (sbus_buf[7] << 7)) & 0x07ff;

// ??λ????mode_sw??bit60??61??buf[7] >>4?????2λ??
rc_ctrl->rc.mode_sw = ((sbus_buf[7] >> 4) & 0x03);//??λ

// Pause??bit62?????bit63?????bit64 ????buf[7]
rc_ctrl->rc.key_pause = (sbus_buf[7] >> 6) & 0x01;//?????
rc_ctrl->rc.key_left  = (sbus_buf[7] >> 7) & 0x01;//fn1
// bit64??sbus_buf[8]??0λ
rc_ctrl->rc.key_right = sbus_buf[8] & 0x01;//fn2

// ????wheel??bit65~75??buf[8]>>1 | buf[9]<<7
rc_ctrl->rc.ch[4] = (((sbus_buf[8] >> 1)) | (sbus_buf[9] << 7)) & 0x07ff;//????

// ???Trig??bit76??buf[9]bit4
rc_ctrl->rc.key_trig = (sbus_buf[9] >> 4) & 0x01;//??????

//=================??????????????Э??????bit80?????buf?±??10???=================
//???X??bit80~95 16λ?з???
rc_ctrl->mouse.x = (int16_t)(sbus_buf[10] | (sbus_buf[11] << 8));
//???Y??bit96~111 16λ?з???
rc_ctrl->mouse.y = (int16_t)(sbus_buf[12] | (sbus_buf[13] << 8));
//???Z??bit112~127 16λ?з???
rc_ctrl->mouse.z = (int16_t)(sbus_buf[14] | (sbus_buf[15] << 8));

//????????bit128?????128??2bit???????1λ??
rc_ctrl->mouse.press_l = (sbus_buf[16] >> 0) & 0x01;
//????????bit130
rc_ctrl->mouse.press_r = (sbus_buf[16] >> 2) & 0x01;
//????м???bit132??????н???????????????油???壩
rc_ctrl->mouse.press_m = (sbus_buf[16] >> 4) & 0x01;

//?????????bit136~151 16bit
rc_ctrl->key.v = (uint16_t)(sbus_buf[17] | (sbus_buf[18] << 8));

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
	
}

