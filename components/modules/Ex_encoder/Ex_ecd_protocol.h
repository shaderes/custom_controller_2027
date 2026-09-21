#ifndef __EX_ECD_PROTOCOL_H__
#define __EX_ECD_PROTOCOL_H__
#include "struct_typedef.h"

typedef __packed struct{
  uint8_t len;
  uint8_t id;
  uint8_t cmd;
  uint32_t ecd;
  uint8_t REMAIN;
} OID_ECD_FEEDBACK_T;

typedef __packed struct {
    uint8_t data_length;   // 数据长度
    uint8_t encoder_addr;  // 编码器地址
    uint8_t command_code;  // 指令码
    uint8_t set_data;        // 数据(根据数据长度占有reserved)
    uint8_t reserved[4];   // 保留字节，用于填充
} OID_ECD_CMD_T;

#endif
