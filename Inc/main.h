/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef __packed struct
{
  uint8_t  sof;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  crc8;
} frame_header_t;

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
	key_t key;
	float joint_angle[6];
	uint32_t uint32_data;
	uint16_t crc16;
} data_t;

extern data_t tx_data;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define PI 3.141592653589793238462643f
#define HPI 1.5707963267948966192313215f
#define DEG_TO_RAD 0.0174532925199432957f
#define GRAVITY 9.8f
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
