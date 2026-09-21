/**
 * @brief CRC for custom controller
 * @see Custom_ctrl.h
 */
#ifndef __CRC_H__
#define __CRC_H__

#include "main.h"
#include "string.h"
#include "stdint.h"

static void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);

static void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

static unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);

static uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

static unsigned  char  Get_CRC8_Check_Sum(unsigned  char  *pchMessage,unsigned  int dwLength,unsigned char ucCRC8);

static uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);

#endif

