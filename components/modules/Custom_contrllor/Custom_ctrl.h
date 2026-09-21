#ifndef __CUSTOM_CTRL_H__
#define __CUSTOM_CTRL_H__

#include "main.h" 
#include "struct_typedef.h"
#include "referee.h"
#include "remote_control.h"

#define USE_DT7_CTRL 1

/*???*/

#define CHANNLE_OFFSET_MAX 660
#define CHANNLE_OFFSET_MIN -660

#if USE_DT7_CTRL
#define GET_KEY(KEY) ((1<<KEY)&(get_remote_control_point()->key.v))
#else
#define GET_KEY(KEY) ((1<<KEY)&(remote_data.key))
#endif
#define GET_CH_VALUE(CH) ((int64_t)(remote_data.ch_##CH-1024))
#define GET_WHEEL_VALUE() (remote_data.wheel-1024)
#define GET_SWITCH() (remote_data.mode_sw)


typedef __packed struct
{
	uint8_t data[30];
} robot_interactive_data_t;

typedef __packed struct
{
  uint8_t  up : 1;
  uint8_t  down : 1;
  uint8_t  left : 1;
	uint8_t  right : 1;
	uint8_t  k1 : 1;
	uint8_t  k2 : 1;
	uint8_t  k3 : 1;
	uint8_t  k4 : 1;
	uint8_t  k5 : 1;
	uint8_t  k6 : 1;
	uint8_t  k7 : 1;
	uint8_t  k8 : 1;
	uint8_t  k9 : 1;
	uint8_t  k10 : 1;
	uint8_t  k11 : 1;
	uint8_t  k12 : 1;
} key_t;

typedef __packed struct
{
	frame_header_t	header;
	uint16_t cmd_id;
	key_t   key;
	float CC_val_f[5];
  uint32_t CC_val_i[2];
  //uint8_t reserve[4];
	uint16_t crc16;
} data_t; 

typedef data_t CUSTOM_CTRL_RX_PACK;

typedef struct{
  CUSTOM_CTRL_RX_PACK rx_pack;
  fp32 joint_angle[5];
  int16_t CC_data[3];
  fp32 G_angle;
  int8_t get_cc_data_flag;
} CUSTOM_CTRL_T;

enum {
  KEY_W=0,
  KEY_S,
  KEY_A,
  KEY_D,
  KEY_SHIFT,
  KEY_CTRL,
  KEY_Q,
  KEY_E,
  KEY_R,
  KEY_F,
  KEY_G,
  KEY_Z,
  KEY_X,
  KEY_C,
  KEY_V,
  KEY_B,
};

/*Í¼´«Ò£¿ØÆ÷*/
typedef __packed struct
{
    uint8_t sof_1;
    uint8_t sof_2;
    uint64_t ch_0:11;
    uint64_t ch_1:11;
    uint64_t ch_2:11;
    uint64_t ch_3:11;
    uint64_t mode_sw:2;
    uint64_t pause:1;
    uint64_t fn_1:1;
    uint64_t fn_2:1;
    uint64_t wheel:11;
    uint64_t trigger:1;

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left:2;
    uint8_t mouse_right:2;
    uint8_t mouse_middle:2;
    uint16_t key;
    uint16_t crc16;
}remote_data_t;

extern remote_data_t remote_data;
extern CUSTOM_CTRL_T CC_handler;

CUSTOM_CTRL_T* Custom_Ctrl_get_ptr(void);
data_t* Custom_Ctrl_get_rx_pack_ptr(void);
void Custom_Ctrl_unpack(void);
void Custom_Ctrl_data_process(void);
void Custom_Ctrl_Task(void *para);

#endif
