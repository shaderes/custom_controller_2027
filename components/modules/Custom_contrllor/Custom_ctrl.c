#include "Custom_ctrl.h"
#include "crc.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "detect_task.h"

#include "hand_task.h"
#include "user_lib.h"

// 注意，先等自定义控制器数据稳定再进入自定义模式，否则可能直接输出未经计算的偏移量
// 或者写点标志位避免这种情况

#define HEADER 0xa5
#define CTRL_HEADER1 0xa9
#define CTRL_HEADER2 0x53
#define PACK_LENGTH (sizeof(data_t))
#define CTRL_PACK_LENGTH (sizeof(remote_data_t))
#define RX_BUF ((void *)Custom_Ctrl_get_rx_pack_ptr())
#define CTRL_RX_BUF ((void *)(&remote_data))
/**/
#define IS_HEADER(byte) (byte == HEADER)
#define IS_CTRL_HEADER(byte1, byte2) ((byte1 == CTRL_HEADER1) && (byte2 == CTRL_HEADER2))

/*自定义控制器*/
#define cc_joint_angle (Custom_Ctrl_get_rx_pack_ptr()->CC_val_f)
#define cc_int_data (Custom_Ctrl_get_rx_pack_ptr()->CC_val_i)

/*custom controller remap(custom controller -> joint_angle)*/
// hand_param
float custom_controller_K[5] = {-1, (2.289f + 2.261f) / (6.936f - 2.903f), -1, 185 / 3.14f, 1};
float custom_controller_D[5] = {0.5506f, 2.903f, 3.6662f, 1.4986f, 2.7442f};

/*crc检测*/
static uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16);
static uint16_t crc16_init = 0xffff;

CUSTOM_CTRL_T CC_handler;
remote_data_t remote_data;

inline CUSTOM_CTRL_T *Custom_Ctrl_get_ptr(void)
{
  return &CC_handler;
}

inline data_t *Custom_Ctrl_get_rx_pack_ptr(void)
{
  return &(CC_handler.rx_pack);
}

// crc16校验
//  static unsigned char CliendTxBuffer[50];
//   memcpy(
//       CliendTxBuffer + 5,
//       (uint8_t *)&data_s->cmd_id,
//       sizeof(data_t) - sizeof(frame_header_t) // 数据部分和cmd_id占用12字节
//   );
//  Verify_CRC16_Check_Sum(CliendTxBuffer, sizeof(data_t));

/**
 * @brief 解包数据
 */
void Custom_Ctrl_unpack(void)
{
  USART1_Recv(RX_BUF, sizeof(frame_header_t) + sizeof(uint16_t));
  if (((data_t *)RX_BUF)->cmd_id == 0x0302)
    USART1_Recv((void *)&(((data_t *)RX_BUF)->key), sizeof(float) * 5 + sizeof(uint32_t) * 2 + sizeof(uint16_t)); // 没有crc//实则图传已经校验过了加不加都行
  Custom_Ctrl_data_process();
  CC_handler.get_cc_data_flag = 1; // 暂不用
}

void Custom_Ctrl_data_process(void)
{
  // if (CC_handler.get_cc_data_flag)
  // {
  CC_handler.joint_angle[0] = (cc_joint_angle[0] - custom_controller_D[0]) * custom_controller_K[0];
  CC_handler.joint_angle[1] = -((cc_joint_angle[1] - custom_controller_D[1]) * custom_controller_K[1] + hand_task_handler_ptr->min_joint_angle[1]);
  CC_handler.joint_angle[2] = (cc_joint_angle[2] - custom_controller_D[2]) * custom_controller_K[2];
  CC_handler.joint_angle[3] = -(cc_joint_angle[3] - custom_controller_D[3]) * custom_controller_K[3];
  CC_handler.joint_angle[4] = -(cc_joint_angle[4] - custom_controller_D[4]) * custom_controller_K[4];
  CC_handler.G_angle = ((((uint32_t)cc_int_data[0] & 0xFFFF) - 0xC8) * ((hand_task_handler_ptr->max_joint_angle[5] - hand_task_handler_ptr->min_joint_angle[5]) / (0xE74 - 0x2DF)));
  CC_handler.CC_data[2] = int16_deadline(((((uint32_t)cc_int_data[1] >> 16) & 0xFFFF) - 0x800), -200, 200);
  CC_handler.CC_data[0] = int16_deadline((((uint32_t)cc_int_data[0] >> 16 & 0xFFFF) - 0x800), -150, 150);
  CC_handler.CC_data[1] = int16_deadline((((uint32_t)cc_int_data[1] & 0xFFFF) - 0x800), -150, 150);
  //   CC_handler.get_cc_data_flag = 0;
  // }
}
/**
 * @brief 初始化数值
 */
void Data_init(data_t *data_init)
{
  data_init->header.sof = HEADER;
  data_init->header.data_length = 30;
  data_init->header.seq = 0;
  data_init->header.crc8 = 0;
  data_init->cmd_id = 0x0302;
}

void Custom_Ctrl_Task(void *para)
{
  static uint32_t usart1_length = 0x00;

  while (1)
  {

    usart1_length = USART1_GetDataCount(); // 得出数据的长度，包括帧头、帧尾、ID和有用的数据

    if (IS_HEADER(USART1_At(0)))
    {
      if (usart1_length >= PACK_LENGTH)
        Custom_Ctrl_unpack();
    }
    else if (USART1_At(0) == CTRL_HEADER1)
    {
      if (usart1_length >= CTRL_PACK_LENGTH)
      {
        if (IS_CTRL_HEADER(USART1_At(0), USART1_At(1)))
        {
          USART1_Recv(CTRL_RX_BUF, sizeof(remote_data_t));
          DetectHook(CAMERA_TOE);
        }
        else
          USART1_Drop(1);
      }
    }
    else if (usart1_length > 3000)
    {
      USART1_Drop(usart1_length);
    }
    else
    {
      USART1_Drop(1);
    }
    // else if(usart1_length >= CTRL_PACK_LENGTH && usart1_length >= PACK_LENGTH)
    //{
    //   USART1_Drop(1);
    //   vTaskDelay(1);
    //   usart1_length = USART1_GetDataCount();
    //   if(usart1_length > 3000)
    //   {
    //       USART1_Drop(4096);
    //   }
    //   goto next_usart1;
    // }
    // else
    //{
    //   for(;;)
    //   {
    //   usart1_length =  USART1_GetDataCount();
    //    if( usart1_length > 0)
    //    {
    //        if(IS_HEADER(USART1_At(0)) &&
    //          USART1_At(0)==CTRL_HEADER1)  // Frame head
    //        {
    //          break;
    //        }
    //        else
    //        {
    //          USART1_Drop(1);
    //          vTaskDelay(1);
    //        }
    //        usart1_length =  USART1_GetDataCount();
    //        if(usart1_length > 3000)
    //        {
    //          USART1_Drop(4096);
    //          break;
    //        }
    //    }
    //    else
    //    {
    //      break;
    //    }
    //  }
    //}
    vTaskDelay(1);
  }
}

/*重复的CRC检测乐色，从例程复制的*/
static const uint16_t crc16_tab[256] =
    {
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
        0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

/**
 * @brief Get the crc16 checksum
 *
 * @param p_msg Data to check
 * @param lenData length
 * @param crc16 Crc16 initialized checksum
 * @return crc16 Crc16 checksum
 */
static uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16)
{
  uint8_t data;

  if (p_msg == NULL)
  {
    return 0xffff;
  }

  while (len--)
  {
    data = *p_msg++;
    (crc16) = ((uint16_t)(crc16) >> 8) ^ crc16_tab[((uint16_t)(crc16) ^ (uint16_t)(data)) & 0x00ff];
  }

  return crc16;
}

/**
 * @brief crc16 verify function
 *
 * @param p_msg Data to verify
 * @param len Stream length=data+checksum
 * @return bool Crc16 check result
 */
int verify_crc16_check_sum(uint8_t *p_msg, uint16_t len)
{
  uint16_t w_expected = 0;

  if ((p_msg == NULL) || (len <= 2))
  {
    return 0;
  }
  w_expected = get_crc16_check_sum(p_msg, len - 2, crc16_init);

  return ((w_expected & 0xff) == p_msg[len - 2] && ((w_expected >> 8) & 0xff) == p_msg[len - 1]);
}
