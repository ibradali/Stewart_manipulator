/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "NRF24L01.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define I2C
//#define RF
//#define CAN

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */


uint8_t RxData[13];
uint16_t adc_raw[12];
uint16_t curr_pot[6];
uint16_t target_pot[6];
double i_e[6];
double error[6];
uint16_t mot_control_signal[6][3];

uint8_t adc_ready;

uint8_t suction_signal;
uint8_t motor_en_signal;

#ifdef CAN
	CAN_RxHeaderTypeDef pRxHeader;
	uint8_t CAN_RxData[8];
#endif

#ifdef RF
	uint8_t RF_address[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE};
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */

void unpack_data(void);
void speed_control(void);
void control_motors(void);

#ifdef CAN
void CAN_Recieve_Unpack(void);
void CAN_UnpackRxMessage(void);
static void CAN_Filter_Config(void);
#endif

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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);


#ifdef RF
  	  NRF24_Init();
  	  NRF24_RxMode(RF_address, 1);
#endif

#ifdef CAN

  	  CAN_Filter_Config();
  	  HAL_CAN_Start(&hcan1);
  	  HAL_CAN_WakeUp(&hcan1);
  	  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_raw, 12);

	  // read and filter ADC values
	  if (adc_ready) {

		  curr_pot[0] = (curr_pot[0] + adc_raw[0] + adc_raw[6]) / 3;
		  curr_pot[1] = (curr_pot[1] + adc_raw[1] + adc_raw[7]) / 3;
		  curr_pot[2] = (curr_pot[2] + adc_raw[2] + adc_raw[8]) / 3;
		  curr_pot[3] = (curr_pot[3] + adc_raw[3] + adc_raw[9]) / 3;
		  curr_pot[4] = (curr_pot[4] + adc_raw[4] + adc_raw[10]) / 3;
		  curr_pot[5] = (curr_pot[5] + adc_raw[5] + adc_raw[11]) / 3;

		  adc_ready = 0;

	  }

#ifdef I2C
	  if (HAL_I2C_Slave_Receive(&hi2c1, RxData, sizeof(RxData), 50) == HAL_OK) {
		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  }

#endif

#ifdef RF
	  if (isDataAvailable(1) == 1) {
		  NRF24_Receive(RxData);
		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  }

#endif

	  unpack_data();

#ifdef CAN

	  CAN_Recieve_Unpack();

#endif

	  if (suction_signal == 1) {
		  HAL_GPIO_WritePin(suction_enable_GPIO_Port, suction_enable_Pin, 1);
	  }
	  else if (suction_signal == 0){
		  HAL_GPIO_WritePin(suction_enable_GPIO_Port, suction_enable_Pin, 0);
	  }

	  speed_control();
	  control_motors();

	  HAL_Delay(10);



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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hcan1.Init.Prescaler = 15;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 100-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4096-1;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 50-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4096-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, C6_In1_Pin|C6_In2_Pin|C2_In2_Pin|C2_In1_Pin
                          |C1_In2_Pin|C1_In1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, C5_In1_Pin|C5_In2_Pin|CE_Pin|CSN_Pin
                          |C4_In2_Pin|C4_In1_Pin|C3_In2_Pin|C3_In1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(suction_enable_GPIO_Port, suction_enable_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : C6_In1_Pin C6_In2_Pin C2_In2_Pin C2_In1_Pin
                           C1_In2_Pin C1_In1_Pin */
  GPIO_InitStruct.Pin = C6_In1_Pin|C6_In2_Pin|C2_In2_Pin|C2_In1_Pin
                          |C1_In2_Pin|C1_In1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : C5_In1_Pin C5_In2_Pin CE_Pin CSN_Pin
                           C4_In2_Pin C4_In1_Pin C3_In2_Pin C3_In1_Pin */
  GPIO_InitStruct.Pin = C5_In1_Pin|C5_In2_Pin|CE_Pin|CSN_Pin
                          |C4_In2_Pin|C4_In1_Pin|C3_In2_Pin|C3_In1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : suction_enable_Pin */
  GPIO_InitStruct.Pin = suction_enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(suction_enable_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void unpack_data(void) {

	target_pot[0] = (uint16_t) RxData[0];
	target_pot[0] |= (uint16_t) (RxData[1]<<8);

	target_pot[1] = (uint16_t) RxData[2];
	target_pot[1] |= (uint16_t) (RxData[3]<<8);

	target_pot[2] = (uint16_t) RxData[4];
	target_pot[2] |= (uint16_t) (RxData[5]<<8);

	target_pot[3] = (uint16_t) RxData[6];
	target_pot[3] |= (uint16_t) (RxData[7]<<8);

	target_pot[4] = (uint16_t) RxData[8];
	target_pot[4] |= (uint16_t) (RxData[9]<<8);

	target_pot[5] = (uint16_t) RxData[10];
	target_pot[5] |= (uint16_t) (RxData[11]<<8);

	suction_signal = (uint8_t) RxData[12] & 0x01;
	motor_en_signal = (uint8_t) (RxData[12] >> 1) & 0x01;



}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	adc_ready = 1;

}


void speed_control(void) {


	int m_signal;
	double p[6] = {24,24,24,24,24,24};
	double T_i = 20;
	double T_d = 15;
	float elapsed_time = 0.01;
	double error_curr;
	double d_error;


	for (int i=0; i<6;i++) {

		// error term
		error_curr = target_pot[i] - curr_pot[i];

		// derivative error
		d_error = (error_curr - error[i]);

		// update error array
		error[i] = error_curr;

		// integral error calculation and limiting wind up
		i_e[i] += elapsed_time * error_curr;

		if (T_i * i_e[i] > 4095) i_e[i] = 4095 / T_i;

		else if (T_i * i_e[i] < -4095) i_e[i] = -4095 / T_i;

		// motor control signal
		m_signal = (int) p[i] * error_curr + T_i * i_e[i] + T_d * d_error;

		// cap the motor PWM signal at max value
		if (m_signal > 3500) m_signal = 3500;
		else if (m_signal < -3500) m_signal = -3500;


		if (m_signal > 0 && motor_en_signal == 1) {
			mot_control_signal[i][0] =  m_signal;
			mot_control_signal[i][1] = 1;
			mot_control_signal[i][2] = 0;

		}
		else if (m_signal < 0 && motor_en_signal == 1) {
			mot_control_signal[i][0] = - m_signal;
			mot_control_signal[i][1] = 0;
			mot_control_signal[i][2] = 1;
		}

		else if (m_signal == 0 || motor_en_signal == 0) {
			mot_control_signal[i][0] = 0;
			mot_control_signal[i][1] = 0;
			mot_control_signal[i][2] = 0;

		}
	}
}


void control_motors(void) {


	HAL_GPIO_WritePin(C1_In1_GPIO_Port, C1_In1_Pin, mot_control_signal[0][1]);
	HAL_GPIO_WritePin(C1_In2_GPIO_Port, C1_In2_Pin, mot_control_signal[0][2]);
	TIM1->CCR1 = mot_control_signal[0][0];

	HAL_GPIO_WritePin(C2_In1_GPIO_Port, C2_In1_Pin, mot_control_signal[1][1]);
	HAL_GPIO_WritePin(C2_In2_GPIO_Port, C2_In2_Pin, mot_control_signal[1][2]);
	TIM1->CCR2 = mot_control_signal[1][0];

	HAL_GPIO_WritePin(C3_In1_GPIO_Port, C3_In1_Pin, mot_control_signal[2][1]);
	HAL_GPIO_WritePin(C3_In2_GPIO_Port, C3_In2_Pin, mot_control_signal[2][2]);
	TIM1->CCR3 = mot_control_signal[2][0];

	HAL_GPIO_WritePin(C4_In1_GPIO_Port, C4_In1_Pin, mot_control_signal[3][1]);
	HAL_GPIO_WritePin(C4_In2_GPIO_Port, C4_In2_Pin, mot_control_signal[3][2]);
	TIM1->CCR4 = mot_control_signal[3][0];

	HAL_GPIO_WritePin(C5_In1_GPIO_Port, C5_In1_Pin, mot_control_signal[4][1]);
	HAL_GPIO_WritePin(C5_In2_GPIO_Port, C5_In2_Pin, mot_control_signal[4][2]);
	TIM2->CCR1 = mot_control_signal[4][0];

	HAL_GPIO_WritePin(C6_In1_GPIO_Port, C6_In1_Pin, mot_control_signal[5][1]);
	HAL_GPIO_WritePin(C6_In2_GPIO_Port, C6_In2_Pin, mot_control_signal[5][2]);
	TIM2->CCR2 = mot_control_signal[5][0];


}

#ifdef CAN

void CAN_Recieve_Unpack(void) {

	while (HAL_CAN_GetRxFifoFillLevel (&hcan1, CAN_RX_FIFO0) != 0)
	  {
		  HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &pRxHeader, RxData);
		  CAN_UnpackRxMessage();
		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  }


	while (HAL_CAN_GetRxFifoFillLevel (&hcan1, CAN_RX_FIFO1) != 0)
	  {
		  HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO1, &pRxHeader, RxData);
		  CAN_UnpackRxMessage();
		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  }


}



void CAN_UnpackRxMessage(void) {

	if (pRxHeader.StdId == 0) {
		target_pot[0] = (uint16_t) CAN_RxData[0];
		target_pot[0] |= (uint16_t) (CAN_RxData[1]<<8);

		target_pot[1] = (uint16_t) CAN_RxData[2];
		target_pot[1] |= (uint16_t) (CAN_RxData[3]<<8);

		target_pot[2] = (uint16_t) CAN_RxData[4];
		target_pot[2] |= (uint16_t) (CAN_RxData[5]<<8);

		target_pot[3] = (uint16_t) CAN_RxData[6];
		target_pot[3] |= (uint16_t) (CAN_RxData[7]<<8);

	}

	else if (pRxHeader.StdId == 1) {

		target_pot[4] = (uint16_t) RxData[0];
		target_pot[4] |= (uint16_t) (RxData[1]<<8);

		target_pot[5] = (uint16_t) RxData[2];
		target_pot[5] |= (uint16_t) (RxData[3]<<8);

		suction_signal = (uint8_t) RxData[4] & 0x01;
		motor_en_signal = (uint8_t) (RxData[5] >> 1) & 0x01;

	}

}

static void CAN_Filter_Config(void) {

	//TODO: Configure filter

	CAN_FilterTypeDef sFilterConfig;

	sFilterConfig.FilterActivation = ENABLE;

	sFilterConfig.FilterBank = 0;

	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;

	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;

	sFilterConfig.FilterIdHigh = 0x0000;

	sFilterConfig.FilterIdLow = 0x0000;

	sFilterConfig.FilterMaskIdHigh = 0x0000;

	sFilterConfig.FilterMaskIdLow = 0x0000;

	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}

	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}
}

#endif

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  while (1)
  {
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
	  HAL_Delay(3000);

	  NVIC_SystemReset();
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
