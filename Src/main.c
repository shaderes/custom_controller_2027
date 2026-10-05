/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "tim.h"
#include "gravity_comp.h"
#include "ZDT/ZDT_X42_V2.h"
#include "arm_param.h"
#include "CRC8_CRC16.h"
#include "string.h"
#include "mit.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t last_tick = 0;   

data_t data;

mit_t zdt_mit[3];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Data_init(data_t *data_init);
void send_data(data_t *data_s);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM2_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  can_filter_init();
  MX_TIM2_Init();
  Data_init(&data);
  mit_init(zdt_mit, ZDT_40MM_MIT_KP, ZDT_40MM_MIT_KD);
  mit_init(zdt_mit + 1, ZDT_23MM_MIT_KP, ZDT_23MM_MIT_KD);
  mit_init(zdt_mit + 1, ZDT_23MM_MIT_KP, ZDT_23MM_MIT_KD);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if ((HAL_GetTick() - last_tick) >= 20)
    {
      last_tick = HAL_GetTick();
      can_zdt_read_position_all();
      can_zdt_read_speed_all();

      for (uint8_t i = 0; i < 6; i++)
      {
        data.joint_angle[i] = theta[i];
        arm_param.arm_theta[i] = theta[i];
        zdt_mit[i].ref_angle = theta[i];
        arm_param.joint_speed[i + ZDT_JOINT_OFFSET] = MotorCurrents[i].dq; 
        zdt_mit[i].ref_speed = MotorCurrents[i].dq;
      }

      gravity_compensation(arm_param.arm_theta, arm_param.joint_torque);

			for (uint8_t i = 0; i < ZDT_MOTOR_ONLINE; i++)
      {
        mit_calc(zdt_mit + i, zdt_angle_target[i], zdt_speed_target[i], 0.01);

        float curr = zdt_mit->torque / zdt_torque_const[i] * 1000.0f;

        if (curr > ZDT_TORQUE_MAX_MA)  curr = ZDT_TORQUE_MAX_MA;
        if (curr < -ZDT_TORQUE_MAX_MA) curr = -ZDT_TORQUE_MAX_MA;

        /* 电机 i+1 → 关节下标 i + ZDT_JOINT_OFFSET（和上面的角度/速度映射一致） */
        arm_param.joint_current[i + ZDT_JOINT_OFFSET] = curr;
        ZDT_X42_V2_Torque_Control(i + 1, (curr < 0), ZDT_TORQUE_RAMP_MA_S,
                                  (uint16_t)((curr < 0) ? -curr : curr), false);
      }

      send_data(&data);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void Data_init(data_t *data_init)
{
	data_init->header.sof = 0xa5;
	data_init->header.data_length = 30;
	data_init->header.seq = 0;
	data_init->header.crc8 = 0;
	data_init->cmd_id = 0x0302;
}

void send_data(data_t *data_s)
{
	static unsigned char CliendTxBuffer[50];

	memcpy(CliendTxBuffer, &data_s->header, sizeof(frame_header_t));//写入帧头数据
  append_CRC8_check_sum(CliendTxBuffer,sizeof(frame_header_t)); //写入帧头校验

	
//	算好CliendTxBuffer的长度，以方便串口整包发送CliendTxBuffer
	
		memcpy(	
			CliendTxBuffer + 5, 
			(uint8_t*)&data_s->cmd_id, 
			sizeof(data_t)-sizeof(frame_header_t)				//数据部分和cmd_id占用12字节
		  );
	
  append_CRC16_check_sum(CliendTxBuffer,sizeof(data_t));

	
	HAL_UART_Transmit_DMA(&huart6,CliendTxBuffer,sizeof(data_t));
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
