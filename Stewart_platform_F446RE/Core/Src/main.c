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

#include <stdio.h>
#include "../stewart/stewart.h"
#include "NRF24L01.h"
#include "sh1106.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define I2C
#define OLED
//#define RF
//#define S_DEBUG


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint16_t adc_raw[6];
float joyx, joyy, joyz, joyrot_x, joyrot_y, joyrot_z;

uint8_t adc_ready;

uint16_t target_pot[6];
uint8_t TxData[12];


#ifdef S_DEBUG
	uint8_t uart_buffer[100];
#endif

#ifdef OLED
	oled_tx_buffer[16];
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
static void MX_SPI1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void move_target_position(stewart* stewart);
float adc_raw_to_joystick(uint16_t adc_raw);
void debug_platform(stewart* stewart, uint8_t output_type);
uint16_t c_length_to_pot_value(float cylinder_length);
void pack_data();


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
	stewart platform;

	platform_init(&platform);

	// set position limits
	platform.xyz_limit = 50.0;	// [mm]
	platform.tilt_limit = 10;	// deg
	platform.rot_limit = 10;	// deg


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
  MX_SPI1_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  adc_ready = 0;


#ifdef RF
  	  NRF24_Init();
  	  nrf24_reset(2);
  	  NRF24_TxMode(RF_address, 1);
#endif

#ifdef OLED
  	  disp_init();
#endif


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_raw, 6);

	  if (adc_ready) {

		  joyx = adc_raw_to_joystick(adc_raw[0]);
		  joyy = adc_raw_to_joystick(adc_raw[1]);
		  joyz = adc_raw_to_joystick(adc_raw[2]);
		  joyrot_x = adc_raw_to_joystick(adc_raw[3]);
		  joyrot_y = adc_raw_to_joystick(adc_raw[4]);
		  joyrot_z = adc_raw_to_joystick(adc_raw[5]);

		  adc_ready = 0;

	  }

	  move_target_position(&platform);

	  rotate_platform(&platform, platform.a1);
	  rotate_platform(&platform, platform.a2);
	  rotate_platform(&platform, platform.a3);
	  rotate_platform(&platform, platform.a4);
	  rotate_platform(&platform, platform.a5);
	  rotate_platform(&platform, platform.a6);

	  run_platform(&platform);



	  target_pot[0] = c_length_to_pot_value(platform.c_target[0]);
	  target_pot[1] = c_length_to_pot_value(platform.c_target[1]);
	  target_pot[2] = c_length_to_pot_value(platform.c_target[2]);
	  target_pot[3] = c_length_to_pot_value(platform.c_target[3]);
	  target_pot[4] = c_length_to_pot_value(platform.c_target[4]);
	  target_pot[5] = c_length_to_pot_value(platform.c_target[5]);

	  pack_data();

#ifdef I2C

	  // sending commands via I2C
	  if (HAL_I2C_Master_Transmit(&hi2c2, 0x00, TxData, sizeof(TxData), 200) == HAL_OK) {
		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  }
#endif


#ifdef OLED
	  sprintf(oled_tx_buffer, "x: %.2f, rotx: %.2f", platform.tp_target_pos[0], platform.tp_target_pos[3]);
	  display_string(1, oled_tx_buffer);
	  sprintf(oled_tx_buffer, "y: %.2f, roty: %.2f", platform.tp_target_pos[1], platform.tp_target_pos[4]);
	  display_string(2, oled_tx_buffer);
	  sprintf(oled_tx_buffer, "z: %.2f, rotz: %.2f", platform.tp_target_pos[2], platform.tp_target_pos[5]);
	  display_string(2, oled_tx_buffer);
	  disp_data();
#endif


#ifdef S_DEBUG
	  debug_platform(&platform, 0)
#endif


#ifdef RF

	  lost_packets = nrf24_ReadReg(0x08);
	  if (NRF24_Transmit(TxData) == 1) {
		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  }

#endif


	  HAL_Delay(20);


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
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
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
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CE_Pin|CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CE_Pin CSN_Pin */
  GPIO_InitStruct.Pin = CE_Pin|CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {

	adc_ready = 1;


}


void move_target_position(stewart* stewart) {


	if (stewart->tp_target_pos[0] > stewart->xyz_limit) stewart->tp_target_pos[0] = stewart->xyz_limit;
	else if (stewart->tp_target_pos[0] < -stewart->xyz_limit) stewart->tp_target_pos[0] = -stewart->xyz_limit;
	else if (stewart->tp_target_pos[0] >= -stewart->xyz_limit || stewart->tp_target_pos[0] <= stewart->xyz_limit )
	{
		stewart->tp_target_pos[0] += joyx;
	}

	if (stewart->tp_target_pos[1] > stewart->xyz_limit) stewart->tp_target_pos[1] = stewart->xyz_limit;
	else if (stewart->tp_target_pos[1] < -stewart->xyz_limit) stewart->tp_target_pos[1] = -stewart->xyz_limit;
	else if (stewart->tp_target_pos[1] >= -stewart->xyz_limit || stewart->tp_target_pos[1] <= stewart->xyz_limit )
	{
		stewart->tp_target_pos[1] += joyy;
	}

	if (stewart->tp_target_pos[2] > 0) stewart->tp_target_pos[2] = 0;
	else if (stewart->tp_target_pos[2] < -2*stewart->xyz_limit) stewart->tp_target_pos[2] = -2*stewart->xyz_limit;
	else if (stewart->tp_target_pos[2] >= -2*stewart->xyz_limit || stewart->tp_target_pos[2] <= 0 )
	{
		stewart->tp_target_pos[2] += joyz;
	}


	if (stewart->tp_target_pos[3] > stewart->tilt_limit) stewart->tp_target_pos[3] = stewart->tilt_limit;
	else if (stewart->tp_target_pos[3] < -stewart->tilt_limit) stewart->tp_target_pos[3] = -stewart->tilt_limit;
	else if (stewart->tp_target_pos[3] >= -stewart->tilt_limit || stewart->tp_target_pos[3] <= stewart->tilt_limit )
	{
		stewart->tp_target_pos[3] += joyrot_x;
	}


	if (stewart->tp_target_pos[4] > stewart->tilt_limit) stewart->tp_target_pos[4] = stewart->tilt_limit;
	else if (stewart->tp_target_pos[4] < -stewart->tilt_limit) stewart->tp_target_pos[4] = -stewart->tilt_limit;
	else if (stewart->tp_target_pos[4] >= -stewart->tilt_limit || stewart->tp_target_pos[4] <= stewart->tilt_limit )
	{
		stewart->tp_target_pos[4] += joyrot_y;
	}


	if (stewart->tp_target_pos[5] > stewart->rot_limit) stewart->tp_target_pos[5] = stewart->rot_limit;
	else if (stewart->tp_target_pos[5] < -stewart->rot_limit) stewart->tp_target_pos[5] = -stewart->rot_limit;
	else if (stewart->tp_target_pos[5] >= -stewart->rot_limit || stewart->tp_target_pos[5] <= stewart->rot_limit )
	{
		stewart->tp_target_pos[5] += joyrot_z;
	}


}


float adc_raw_to_joystick(uint16_t adc_raw) {

	float joy_val;

	if (adc_raw >= 1900 && adc_raw <= 2100) {
		return 0;
	}

	else if (adc_raw > 2100) {
		joy_val = (float) (adc_raw - 2100) / 1000;

	}

	else if (adc_raw <1900) {
		joy_val = (float) (adc_raw - 1900)/ 1000;

	}
	return joy_val;
}


void debug_platform(stewart* stewart, uint8_t output_type) {

	if (output_type == 0) {
		uint8_t buf_len = sprintf((char *) uart_buffer, "x: %.2f, y: %.2f, z: %.2f thetax: %.2f thetay: %.2f thetaz: %.2f \n\r",
				stewart->tp_target_pos[0], stewart->tp_target_pos[1], stewart->tp_target_pos[2],
				stewart->tp_target_pos[3], stewart->tp_target_pos[4], stewart->tp_target_pos[5]);

		HAL_UART_Transmit(&huart2, uart_buffer, buf_len, 100);

	}

	else if (output_type == 1) {

		uint8_t buf_len = sprintf((char *) uart_buffer, "l1:%.2f, l2:%.2f, l3:%f, l4:%.2f, l5:%.2f l6:%.2f  \n\r",
				stewart->c_target[0], stewart->c_target[1], stewart->c_target[2],
				stewart->c_target[3], stewart->c_target[4], stewart->c_target[5]);

		HAL_UART_Transmit(&huart2, uart_buffer, buf_len, 100);
	}
	else if (output_type == 2) {
		uint8_t buf_len = sprintf((char *) uart_buffer,
				"target pot1:%.2f, target pot2:%.2f, target pot3:%f, target pot4:%.2f, target pot5:%.2f target pot6:%.2f  \n\r",
					target_pot[0], target_pot[1], target_pot[2],
					target_pot[3], target_pot[4], target_pot[5]);
	}

}


uint16_t c_length_to_pot_value(float cylinder_length) {

	uint16_t pot_val;

	pot_val = (uint16_t) (cylinder_length - 400.00) * 13.6533;

	return pot_val;
}

void pack_data(void) {

	TxData[0] = (uint8_t) (target_pot[0] & 0xFF);
	TxData[1] = (uint8_t) (target_pot[0] >> 8) & 0x0F;

	TxData[2] = (uint8_t) (target_pot[1] & 0xFF);
	TxData[3] = (uint8_t) (target_pot[1] >> 8) & 0x0F;

	TxData[4] = (uint8_t) (target_pot[2] & 0xFF);
	TxData[5] = (uint8_t) (target_pot[2] >> 8) & 0x0F;

	TxData[6] = (uint8_t) (target_pot[3] & 0xFF);
	TxData[7] = (uint8_t) (target_pot[3] >> 8) & 0x0F;

	TxData[8] = (uint8_t) (target_pot[4] & 0xFF);
	TxData[9] = (uint8_t) (target_pot[4] >> 8) & 0x0F;

	TxData[10] = (uint8_t) (target_pot[5] & 0xFF);
	TxData[11] = (uint8_t) (target_pot[5] >> 8) & 0x0F;

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

  while (1)
  {
	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  HAL_Delay(1000);
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
