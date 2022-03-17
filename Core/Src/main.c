/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "HuLa_LCD1602.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define stepsperrev 200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
char Rx_data[4] = { 0 };
uint16_t Count = 0, TiemCan = 0, Dem =0;
uint8_t  TrangThai =0, HamRot = 0;
uint8_t Mode =0, Rot =0, SoLy =0, DungLai =0, RotTroChoi =0;
uint32_t ThoiGian =0, KiemTra =0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while(__HAL_TIM_GET_COUNTER(&htim1) < us);
}
void stepper_set_rpm (int rpm)
{
	delay(60000000/stepsperrev/rpm);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	HAL_UART_Receive_IT(&huart1, (uint8_t*) Rx_data, 4);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
	if(Rx_data[0] == '1') // Vòng quay
	{
		if(Rx_data[3] == 'B')  // 1 chữ số
		{
			if(Rx_data[2] == '2' || Rx_data[2] == '6' || Rx_data[2] == '8')
			{
				RotTroChoi = 1;
				HamRot = 1;
			}
			if(Rx_data[2] == '3' || Rx_data[2] == '9')
			{
				RotTroChoi = 1;
				HamRot = 2;
			}
			if(Rx_data[2] == '7')
			{
				RotTroChoi = 1;
				HamRot = 3;
			}
			if(Rx_data[2] == '5')
			{
				RotTroChoi = 1;
				HamRot = 6;
			}
			if(Rx_data[2] == '4')
			{
				RotTroChoi = 1;
				HamRot = 10;
			}
			if(Rx_data[2] == '1')  // qua lượt
			{
				RotTroChoi = 1;
				HamRot = 9;
			}
		}
		else  // 2 chữ số
		{
			if((Rx_data[2] == '1' && Rx_data[3] == '1') || (Rx_data[2] == '1' && Rx_data[3] == '2'))
			{
				RotTroChoi = 1;
				HamRot = 1;
			}
			if((Rx_data[2] == '1' && Rx_data[3] == '0'))
			{
				RotTroChoi = 1;
				HamRot = 6;
			}
		}

	}
	else  // Ai là Triệu phú
	{
		if(Rx_data[2] == '1')
		{
			RotTroChoi = 1;
			HamRot = 9;
		}
		if(Rx_data[2] == '0')
		{
			RotTroChoi = 1;
			HamRot = 1;
		}
	}
	Count++;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == BTN1_Pin) {
		for (int i = 0; i < 50000; i++); //Delay
		while (!HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin));
		for (int i = 0; i < 50000; i++);
		EXTI->PR |= BTN1_Pin;

		Mode =!Mode;
		RotTroChoi =0;
		Rot =0;

		if(Mode == 1) // Tro Chơi
		{
			lcd_goto_XY(1, 6);
			lcd_send_string("Hu La");
			lcd_goto_XY(2, 0);
			lcd_send_string("Mode: Tro Choi   ");
		}
		else  // Rot 1 vòng
		{
			lcd_goto_XY(1, 6);
			lcd_send_string("Hu La");
			lcd_goto_XY(2, 0);
			lcd_send_string("Mode: Dieu Khien");
		}

	}
	if (GPIO_Pin == BTN2_Pin) {
		for (int i = 0; i < 50000; i++); //Delay
		while (!HAL_GPIO_ReadPin(BTN2_GPIO_Port, BTN2_Pin));
		for (int i = 0; i < 50000; i++);
		EXTI->PR |= BTN2_Pin;

		Rot = 1;

	}

}

void MotLy()
{
	if(RotTroChoi ==1 )
	{

		TrangThai = TiemCan;
		TiemCan = HAL_GPIO_ReadPin(TC_GPIO_Port, TC_Pin);

		HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 1);
		stepper_set_rpm(30);
		HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 0);
		stepper_set_rpm(30);

		if((TiemCan != TrangThai) && (TiemCan == 1) && (HAL_GetTick() - ThoiGian > 1200) && (DungLai == 0))
		{
			KiemTra = HAL_GetTick() - ThoiGian;
			ThoiGian = HAL_GetTick();

			HAL_GPIO_WritePin(BOM_GPIO_Port, BOM_Pin, 1);
			HAL_Delay(1000);
			HAL_GPIO_WritePin(BOM_GPIO_Port, BOM_Pin, 0);

			SoLy ++;
			if(SoLy == 1)
			{
				SoLy =0;
				DungLai =1;
			}
		}

		Dem ++;
		if(Dem > 200)
			{
				RotTroChoi =0;
				Dem =0;
				DungLai =0;
			}
	}
}
void HaiLy()
{
	if(RotTroChoi ==1 )
	{

		TrangThai = TiemCan;
		TiemCan = HAL_GPIO_ReadPin(TC_GPIO_Port, TC_Pin);

		HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 1);
		stepper_set_rpm(30);
		HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 0);
		stepper_set_rpm(30);

		if((TiemCan != TrangThai) && (TiemCan == 1) && (HAL_GetTick() - ThoiGian > 1200) && (DungLai == 0))
		{
			KiemTra = HAL_GetTick() - ThoiGian;
			ThoiGian = HAL_GetTick();

			HAL_GPIO_WritePin(BOM_GPIO_Port, BOM_Pin, 1);
			HAL_Delay(1000);
			HAL_GPIO_WritePin(BOM_GPIO_Port, BOM_Pin, 0);

			SoLy ++;
			if(SoLy == 2)
			{
				SoLy =0;
				DungLai =1;
			}
		}

		Dem ++;
		if(Dem > 200)
			{
				RotTroChoi =0;
				Dem =0;
				DungLai =0;
			}
	}
}
void BaLy()
{
	if(RotTroChoi ==1 )
	{

		TrangThai = TiemCan;
		TiemCan = HAL_GPIO_ReadPin(TC_GPIO_Port, TC_Pin);

		HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 1);
		stepper_set_rpm(30);
		HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 0);
		stepper_set_rpm(30);

		if((TiemCan != TrangThai) && (TiemCan == 1) && (HAL_GetTick() - ThoiGian > 1200) && (DungLai == 0))
		{
			KiemTra = HAL_GetTick() - ThoiGian;
			ThoiGian = HAL_GetTick();

			HAL_GPIO_WritePin(BOM_GPIO_Port, BOM_Pin, 1);
			HAL_Delay(1000);
			HAL_GPIO_WritePin(BOM_GPIO_Port, BOM_Pin, 0);

			SoLy ++;
			if(SoLy == 3)
			{
				SoLy =0;
				DungLai =1;
			}
		}

		Dem ++;
		if(Dem > 200)
			{
				RotTroChoi =0;
				Dem =0;
				DungLai =0;
			}
	}
}
void NuaLy()
{
	if(RotTroChoi ==1 )
	{

		TrangThai = TiemCan;
		TiemCan = HAL_GPIO_ReadPin(TC_GPIO_Port, TC_Pin);

		HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 1);
		stepper_set_rpm(30);
		HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 0);
		stepper_set_rpm(30);

		if((TiemCan != TrangThai) && (TiemCan == 1) && (HAL_GetTick() - ThoiGian > 1200) && (DungLai == 0))
		{
			KiemTra = HAL_GetTick() - ThoiGian;
			ThoiGian = HAL_GetTick();

			HAL_GPIO_WritePin(BOM_GPIO_Port, BOM_Pin, 1);
			HAL_Delay(500);
			HAL_GPIO_WritePin(BOM_GPIO_Port, BOM_Pin, 0);

			SoLy ++;
			if(SoLy == 1)
			{
				SoLy =0;
				DungLai =1;
			}
		}

		Dem ++;
		if(Dem > 200)
			{
				RotTroChoi =0;
				Dem =0;
				DungLai =0;
			}
	}
}
void SauLy()
{
	if(RotTroChoi ==1 )
	{

		TrangThai = TiemCan;
		TiemCan = HAL_GPIO_ReadPin(TC_GPIO_Port, TC_Pin);

		HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 1);
		stepper_set_rpm(30);
		HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 0);
		stepper_set_rpm(30);

		if((TiemCan != TrangThai) && (TiemCan == 1) && (HAL_GetTick() - ThoiGian > 1200) && (DungLai == 0))
		{
			KiemTra = HAL_GetTick() - ThoiGian;
			ThoiGian = HAL_GetTick();

			HAL_GPIO_WritePin(BOM_GPIO_Port, BOM_Pin, 1);
			HAL_Delay(1000);
			HAL_GPIO_WritePin(BOM_GPIO_Port, BOM_Pin, 0);

			SoLy ++;
			if(SoLy == 6)
			{
				SoLy =0;
				DungLai =1;
			}
		}

		Dem ++;
		if(Dem > 200)
			{
				RotTroChoi =0;
				Dem =0;
				DungLai =0;
			}
	}
}
void QuaLuot()
{
	if(RotTroChoi ==1 )
	{

		TrangThai = TiemCan;
		TiemCan = HAL_GPIO_ReadPin(TC_GPIO_Port, TC_Pin);

		HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 1);
		stepper_set_rpm(30);
		HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 0);
		stepper_set_rpm(30);

		SoLy =0;
		DungLai =1;

		if((TiemCan != TrangThai) && (TiemCan == 1) && (HAL_GetTick() - ThoiGian > 1200) && (DungLai == 0))
		{
			KiemTra = HAL_GetTick() - ThoiGian;
			ThoiGian = HAL_GetTick();

			HAL_GPIO_WritePin(BOM_GPIO_Port, BOM_Pin, 1);
			HAL_Delay(1000);
			HAL_GPIO_WritePin(BOM_GPIO_Port, BOM_Pin, 0);

		}

		Dem ++;
		if(Dem > 200)
			{
				RotTroChoi =0;
				Dem =0;
				DungLai =0;
			}
	}
}
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, 1);
	HAL_UART_Receive_IT(&huart1, (uint8_t*) Rx_data, 4); // Nhận Data
	lcd_init();
	HAL_TIM_Base_Start ( &htim1);


	// Dong cơ khởi động
	HAL_Delay(1000);

	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 1);
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, 1);
	HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, 1);

	if(HAL_GPIO_ReadPin(TC_GPIO_Port, TC_Pin) == 1)
	{
		for(int i=1;i<=45;i++)
		{
			HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 1);
			stepper_set_rpm(40);
			HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 0);
			stepper_set_rpm(40);
		}
	}
	else
	{
		for(int i=1;i<=200;i++)
		{
			if(HAL_GPIO_ReadPin(TC_GPIO_Port, TC_Pin) == 1) break;
			HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 1);
			stepper_set_rpm(40);
			HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 0);
			stepper_set_rpm(40);
		}
		for(int i=1;i<=45;i++)
		{
			HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 1);
			stepper_set_rpm(40);
			HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 0);
			stepper_set_rpm(40);
		}
	}

	lcd_goto_XY(1, 6);
	lcd_send_string("Hu La");
	lcd_goto_XY(2, 0);
	lcd_send_string("Mode: Dieu Khien");
	HAL_Delay(1000);

	ThoiGian = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		switch (Mode)
		{
			case 0:
			{
				if(Rot ==1 )
				{

					TrangThai = TiemCan;
					TiemCan = HAL_GPIO_ReadPin(TC_GPIO_Port, TC_Pin);

					HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 1);
					stepper_set_rpm(30);
					HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 0);
					stepper_set_rpm(30);

					if((TiemCan != TrangThai) && (TiemCan == 1) && (HAL_GetTick() - ThoiGian > 1200))
					{
						KiemTra = HAL_GetTick() - ThoiGian;
						ThoiGian = HAL_GetTick();
						HAL_GPIO_WritePin(BOM_GPIO_Port, BOM_Pin, 1);
						HAL_Delay(1000);
						HAL_GPIO_WritePin(BOM_GPIO_Port, BOM_Pin, 0);
					}

					Dem ++;
					if(Dem > 200)
						{
							Rot =0;
							Dem =0;
						}
				}
				break;
			}
			case 1:
			{
				switch (HamRot)
				{
					case 1:
					{
						MotLy();
						break;
					}
					case 2:
					{
						HaiLy();
						break;
					}
					case 3:
					{
						BaLy();
						break;
					}
					case 6:
					{
						SauLy();
						break;
					}
					case 10:
					{
						NuaLy();
						break;
					}
					case 9:
					{
						QuaLuot();
						break;
					}
				}
				break;
			}
		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, STEP_Pin|BOM_Pin|LED1_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED2_Pin|LED1B10_Pin|LED2B11_Pin|LED4_Pin
                          |LED5_Pin|LED6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BTN2_Pin */
  GPIO_InitStruct.Pin = BTN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_Pin STEP_Pin BOM_Pin LED1_Pin
                           LED3_Pin */
  GPIO_InitStruct.Pin = DIR_Pin|STEP_Pin|BOM_Pin|LED1_Pin
                          |LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN1_Pin */
  GPIO_InitStruct.Pin = BTN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin LED1B10_Pin LED2B11_Pin LED4_Pin
                           LED5_Pin LED6_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED1B10_Pin|LED2B11_Pin|LED4_Pin
                          |LED5_Pin|LED6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TC_Pin */
  GPIO_InitStruct.Pin = TC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TC_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
