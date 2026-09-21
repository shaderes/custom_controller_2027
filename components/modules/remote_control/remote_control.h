/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       remote_control.c/h
  * @brief      閬ユ帶鍣ㄥ鐞嗭紝閬ユ帶鍣ㄦ槸閫氳繃绫讳技SBUS鐨勫崗璁紶杈擄紝鍒╃敤DMA浼犺緭鏂瑰紡鑺傜害CPU
  *             璧勬簮锛屽埄鐢ㄤ覆鍙ｇ┖闂蹭腑鏂潵鎷夎捣澶勭悊鍑芥暟锛屽悓鏃舵彁渚涗竴浜涙帀绾块噸鍚疍MA锛屼覆鍙?
  *             鐨勬柟寮忎繚璇佺儹鎻掓嫈鐨勭ǔ瀹氭�с�?
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include "main.h"

#include "struct_typedef.h"
#include "bsp_rc.h"

#define SBUS_RX_BUF_NUM 42u

#define RC_FRAME_LENGTH 21u

//閬ユ帶鍣ㄥ嚭閿欐暟鎹笂闄?
/*新控档位C:0 N:1 S:2*/ 
#define RC_GEAR_C               ((uint8_t)0)    /* C 挡 (低速)   — 映射为旧版 UP  */
#define RC_GEAR_N               ((uint8_t)1)    /* N 挡 (中速)   — 映射为旧版 MID */
#define RC_GEAR_S               ((uint8_t)2)    /* S 挡 (高速)   — 映射为旧版 DOWN*/

#define RC_CHANNAL_ERROR_VALUE 700
#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)
/* ----------------------- Data Struct ------------------------------------- */
typedef __packed struct
{
        __packed struct
        {
              //  int16_t ch[5];
              //  char s[2];
					   int16_t ch[5];//ch4为拨轮这样子老代码改的小一点
    uint8_t mode_sw;    // 挡位开关(2bit 0/1/2)
    uint8_t key_pause; // 暂停键
    uint8_t key_left;   // 自定义左键
    uint8_t key_right;  // 自定义右键
    uint8_t key_trig;  // 扳机按键
        } rc;
        __packed struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
					uint8_t press_m; //新增：鼠标中键
        } mouse;
        __packed struct
        {
                uint16_t v;
        } key;

} RC_ctrl_t;

//23锛?
// struct CMControlValue
// {
//    int16_t forwardOrbackward;
//    int16_t left_right;
//    int16_t horizoltal;
//    int16_t DnfanceAngle;
//    int16_t DnfanceSpeed;	
// };

// struct GimbalControlValue
// {
//    float updown;
//    float left_right;
// };

// struct ControlValue
// {
//   struct CMControlValue CMControl;
//   struct GimbalControlValue GimbalControl;
// };



// struct RemotDataPack
// {
// 	struct RC rc;
// 	struct MouseKey mousekey;
// 	u8 RC_State;
//     uint16_t	RC_onlineCnt;
// };

/* ----------------------- Internal Data ----------------------------------- */

static int16_t RC_abs(int16_t value);

void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
void remote_control_init(void);
const RC_ctrl_t *get_remote_control_point(void);
uint8_t RC_data_is_error(void);
void slove_RC_lost(void);
void slove_data_error(void);
static int16_t RC_abs(int16_t value);
//void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
void DBUS_IDLERX_HOOK(void);
void DBUS_ERR_HOOK(void);
#endif
