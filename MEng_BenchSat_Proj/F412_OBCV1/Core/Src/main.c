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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "process.h"

#include <stdio.h>

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
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
LIS2MDL comp;
ISM330 acc_gyro;
solenoid sol;

CANBUS comm;

can_buf can;
tel_rx_data tel_rx;


SD SD_BUTTON_move;
SD SD_RCS_move;
SD SD_RCS_spin;
SD SD_RECORD;

SD* SD_Active;

uint8_t button_flag = 0;

uint8_t acknack;

uint8_t OBC_STATE;

uint8_t CURR_TASK;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_CAN_RxFifo0MsgPendingCallback( CAN_HandleTypeDef *hcan )
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &comm.RXHeader, comm.RXdata);
	CAN_Receive( &comm );
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
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */

  /*
   * INIT ALL DEVICES AND PERIPHERALS
   */

  // GIVE PERIPHERALS TIME TO BOOT
  HAL_Delay(500);

  LIS2MDL_Initialise( &comp, &hi2c1, 0, &button_flag );

  ISM330_Initialise(&acc_gyro, &hi2c1);

  CAN_Initialise( &comm, &hcan1, SAT_OBC, 0x0F0, 0, 10 );

  SD_Mount();

//  Process_LIS2MDL_TEST( &comp, &LIS_card );

//  Process_ISM330_ACC_TEST(&acc_gyro, &LIS_card);

  Process_ISM330_GYRO_TEST(&acc_gyro, &SD_RECORD);


  uint32_t beacon_time = HAL_GetTick();

  uint32_t flash_time = HAL_GetTick();

  uint32_t test_time;

  uint16_t flash = 2000;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*
	   * Button related tasks
	   */
	  if( button_flag == 1 )
	  {
		  for (uint8_t j = 0; j < 25; j++)
		  {
			  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			  HAL_Delay(200);
		  }
		  for (uint8_t i = 0; i < 50; i++)
		  {
			  Process_ISM330_GYRO_SD(&acc_gyro, &SD_RECORD);
		  }
		  for (uint8_t j = 0; j < 10; j++)
		  {
			  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			  HAL_Delay(100);
		  }
		  button_flag = 0;
	  }
	  /*
	   * CAN RELATED TASKS
	   */
	  can = Process_CAN_INBOX( &comm );

	  switch (can.msg_typ)
	  {
	  case 4: //NO MESSAGE msg_typ is 2 bit so 4 is out of range
		  break;
	  case 0:
		  Process_TELEMETRY_REQ(&comm, can, &acc_gyro, &comp);
		  Process_TELEMETRY_RX(&comm, can, &tel_rx);
		  break;
	  case 1:
		  // Process COMMANDS
		  if (can.buffer[0] == TASK_AB_TEST)
		  {
			  if (OBC_STATE == 0)
			  {
				  Process_OBC_Movement_Test( &comp, &SD_RCS_move, 1 );
				  SD_Active = &SD_RCS_move;
				  OBC_STATE = 1;
				  test_time = HAL_GetTick();
				  flash = 500;
				  CURR_TASK = TASK_AB_TEST;
				  SEND_ACKNACK( &comm, TASK_AB_TEST, 0, can.source);
			  }
			  else
			  {
				  SEND_ACKNACK( &comm, TASK_AB_TEST, 1, can.source);
			  }

		  }
		  else if (can.buffer[0] == TASK_SPIN_TEST)
		  {
			  if (OBC_STATE == 0)
			  {
				  Process_OBC_Movement_Test( &comp, &SD_RCS_spin, 2 );
				  SD_Active = &SD_RCS_spin;
				  OBC_STATE = 1;
				  test_time = HAL_GetTick();
				  flash = 500;
				  CURR_TASK = TASK_SPIN_TEST;
				  SEND_ACKNACK( &comm, TASK_SPIN_TEST, 0, can.source);
			  }
			  else
			  {
				  SEND_ACKNACK( &comm, TASK_SPIN_TEST, 1, can.source);
			  }
		  }
		  else if (can.buffer[0] == TASK_RECORD)
		  {
			  if (OBC_STATE == 0)
			  {
				  OBC_STATE = 4;
				  Process_OBC_Movement_Test( &comp, &SD_RECORD, 3 );
				  SD_Active = &SD_RECORD;
				  flash = 500;
				  CURR_TASK = TASK_RECORD;
				  SEND_ACKNACK( &comm, TASK_RECORD, 0, can.source);
			  }
			  else if (OBC_STATE == 4)
			  {
				  OBC_STATE = 0;
				  flash = 2000;
				  CURR_TASK = 0xFF;
				  SEND_ACKNACK( &comm, TASK_RECORD, 0, can.source);
			  }
			  else
			  {
				  SEND_ACKNACK( &comm, TASK_RECORD, 1, can.source);
			  }
		  }
		  break;
	  case 2:
		  // Process DIRECT
		  if (can.buffer[0] == 0)
		  {
			  acknack = Solenoid_Move_Start( &sol, can.buffer[1], can.buffer[2] );
			  SEND_ACKNACK( &comm, TASK_SOL, acknack, can.source);
			  // IF returns 1 send NACK
		  }
		  break;
	  }

	  /*
	   * EVERY LOOP TASKS
	   */

	  Process_Solenoid( &sol );

//	  Process_TEST(uint8_t* OBC_STATE, uint8_t CURR_TASK)

	  if (OBC_STATE)
	  {
		  if (OBC_STATE == 1)
		  {
			  if (HAL_GetTick() - test_time > 5000)
			  {
				  if (CURR_TASK == TASK_AB_TEST)
					  Solenoid_Move_Start(&sol, 2, 12);
				  else if (CURR_TASK == TASK_SPIN_TEST)
					  Solenoid_Move_Start(&sol, 3, 10);
				  OBC_STATE = 2;
				  flash = 1000;
			  }
		  }
		  else if (OBC_STATE == 2)
		  {
			  if (HAL_GetTick() - test_time > 15000)
			  {
				  if (CURR_TASK == TASK_AB_TEST)
					  Solenoid_Move_Start(&sol, 1, 5);
				  else if (CURR_TASK == TASK_SPIN_TEST)
					  Solenoid_Move_Start(&sol, 4, 10);
				  OBC_STATE = 3;
				  flash = 100;
			  }
		  }
		  else if (OBC_STATE == 3)
		  {
			  if (HAL_GetTick() - test_time > 30000)
			  {
				  OBC_STATE = 0;
				  flash = 2000;
			  }
		  }
		  // STORE DATA
		  Process_OBC_Movement_Test_WRITE_DATA(OBC_STATE, sol.move_selected, &acc_gyro, &comp, SD_Active, &tel_rx);
	  }

	  /*
	   * TIMED TASKS
	   */

	  if (HAL_GetTick() - beacon_time > 5000)
	  {
		  // SEND BEACON
		  beacon_time = HAL_GetTick();
		  Process_BEACON( &comm, OBC_STATE );
	  }

	  if (HAL_GetTick() - flash_time > flash)
	  {
		  flash_time = HAL_GetTick();
		  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 32000-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 10000-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SD_CS1_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SOL_N1_Pin|SOL_N2_Pin|SOL_N3_Pin|SOL_N4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SW1_EXT_Pin */
  GPIO_InitStruct.Pin = SW1_EXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW1_EXT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_CS1_Pin LED1_Pin */
  GPIO_InitStruct.Pin = SD_CS1_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : EXT_COMP_Pin */
  GPIO_InitStruct.Pin = EXT_COMP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EXT_COMP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EXT_GYRO_Pin EXT_ACC_Pin */
  GPIO_InitStruct.Pin = EXT_GYRO_Pin|EXT_ACC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SOL_N1_Pin SOL_N2_Pin SOL_N3_Pin SOL_N4_Pin */
  GPIO_InitStruct.Pin = SOL_N1_Pin|SOL_N2_Pin|SOL_N3_Pin|SOL_N4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
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
  while (1)
  {
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
