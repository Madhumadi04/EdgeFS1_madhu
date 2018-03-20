/*
 * File:		init.c
 * Project:	Foxconn4Tech - EdgeNodeFS
 * Author:	Ivan Doležal
 * Date:		26. 2. 2018
 * MCU:     STM32F415RG
 * HW:		Edge FS Main Board V1.0
 * Library:	STM32F4Cube HAL V1.18
 * Description:
HAL peripheral initializations
 */

#include "edge.h"

/* Private variables ---------------------------------------------------------*/

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

RTC_HandleTypeDef hrtc;
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

// CubeMX HAL Init Functions

/** System Clock Configuration
*/
void SystemClock_Config(void)
	{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

	/**Configure the main internal regulator output voltage 
	*/
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks 
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSI; // RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSEState = RCC_LSI_ON; // RCC_LSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
		{
		_Error_Handler(__FILE__, __LINE__);
		}

	/**Initializes the CPU, AHB and APB busses clocks 
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
	                            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
		{
		_Error_Handler(__FILE__, __LINE__);
		}

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI; // RCC_RTCCLKSOURCE_LSE;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
		{
		_Error_Handler(__FILE__, __LINE__);
		}

	/**Configure the Systick interrupt time 
	*/
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick 
	*/
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
	}

/* ADC1 init function */
void MX_ADC1_Init(void)
	{

	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
	*/
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; 	// 21 MHz from APB2=84 MHz
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 5;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
		{
		_Error_Handler(__FILE__, __LINE__);
		}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. */
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES; 	// Cycles: 3, 15, 28, 56, 84, 112, 144, 480
																		// ADC conversion = + bit resolution (i.e. +12 cycles for 12 bit)
	if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
		_Error_Handler(__FILE__, __LINE__);
		}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. */
	sConfig.Channel = ADC_CHANNEL_7;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
		_Error_Handler(__FILE__, __LINE__);
		}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. */
	sConfig.Channel = ADC_CHANNEL_14;
	sConfig.Rank = 3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
		_Error_Handler(__FILE__, __LINE__);
		}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. */
	sConfig.Channel = ADC_CHANNEL_15;
	sConfig.Rank = 4;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
		_Error_Handler(__FILE__, __LINE__);
		}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. */
	sConfig.Channel = ADC_CHANNEL_VREFINT;
	sConfig.Rank = 5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
		_Error_Handler(__FILE__, __LINE__);
		}

	}

/* CAN1 init function */
void MX_CAN1_Init(void)
	{

	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 16;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SJW = CAN_SJW_1TQ;
	hcan1.Init.BS1 = CAN_BS1_1TQ;
	hcan1.Init.BS2 = CAN_BS2_1TQ;
	hcan1.Init.TTCM = DISABLE;
	hcan1.Init.ABOM = DISABLE;
	hcan1.Init.AWUM = DISABLE;
	hcan1.Init.NART = DISABLE;
	hcan1.Init.RFLM = DISABLE;
	hcan1.Init.TXFP = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK)
		{
		_Error_Handler(__FILE__, __LINE__);
		}

	}

/* RTC init function */
void MX_RTC_Init(void)
	{
	/**Initialize RTC Only 
	*/
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK)
		{
		_Error_Handler(__FILE__, __LINE__);
		}

	/**Initialize RTC and set the Time and Date 
	*/
	if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2)
		{
		if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
			{
			_Error_Handler(__FILE__, __LINE__);
			}

		if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
			{
			_Error_Handler(__FILE__, __LINE__);
			}

		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0x32F2);
		}

	}

/* SPI2 init function */
void MX_SPI2_Init(void)		// IO-Link IC
	{
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK)
		{
		_Error_Handler(__FILE__, __LINE__);
		}
	__HAL_SPI_ENABLE(&hspi2);   	// added
	}

/* SPI3 init function */
void MX_SPI3_Init(void)		// COM interface (LoRaWAN)
	{

	/* SPI3 parameter configuration*/
	hspi3.Instance = SPI3;
	hspi3.Init.Mode = SPI_MODE_MASTER;
	hspi3.Init.Direction = SPI_DIRECTION_2LINES;
	hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi3.Init.NSS = SPI_NSS_SOFT;
	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;	// 42/8 = 5.25 Mb/s (DS: 10 Mb/s max)
	hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi3.Init.CRCPolynomial = 10; if (HAL_SPI_Init(&hspi3) != HAL_OK)
		{
		_Error_Handler(__FILE__, __LINE__);
		} 
	__HAL_SPI_ENABLE(&hspi3);  	// added

	}

/* UART4 init function */
void MX_UART4_Init(void)	// IO-Link channel 2
	{

	huart4.Instance = UART4;
	huart4.Init.BaudRate = 9600;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart4) != HAL_OK)
		{
		_Error_Handler(__FILE__, __LINE__);
		}
	__HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);    	// had to be added
	}

/* UART5 init function */
void MX_UART5_Init(void)	// IO-Link channel 1
	{

	huart5.Instance = UART5;
	huart5.Init.BaudRate = 9600;
	huart5.Init.WordLength = UART_WORDLENGTH_8B;
	huart5.Init.StopBits = UART_STOPBITS_1;
	huart5.Init.Parity = UART_PARITY_NONE;
	huart5.Init.Mode = UART_MODE_TX_RX;
	huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart5.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart5) != HAL_OK)
		{
		_Error_Handler(__FILE__, __LINE__);
		}
	__HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);   	// had to be added
	}

/* USART1 init function */
void MX_USART1_UART_Init(void)	// COM interface (GSM LTE)
	{

	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;  // UART_HWCONTROL_RTS_CTS;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
		{
		_Error_Handler(__FILE__, __LINE__);
		}
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE); 	// had to be added
	}

/* USART2 init function */
void MX_USART2_UART_Init(void)	// RS232 for DMM Agilent (optionally STDIO)
	{

	huart2.Instance = USART2;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
#ifdef RS232_DMM	// DMM
	huart2.Init.BaudRate = 9600;
	huart2.Init.StopBits = UART_STOPBITS_2;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
#else	// STDIO	huart2.Init.BaudRate = 115200;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
#endif
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
		{
		_Error_Handler(__FILE__, __LINE__);
		}
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE); 	// had to be added
	}

/* USART6 init function */
void MX_USART6_UART_Init(void)	// RS485 - MODBUS
	{

	huart6.Instance = USART6;
	huart6.Init.BaudRate = 9600;
	huart6.Init.WordLength = UART_WORDLENGTH_9B;	// 8 data + 1 parity
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_EVEN;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK)
		{
		_Error_Handler(__FILE__, __LINE__);
		}
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);  	// had to be added
	}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)		// for ADC1
	{
	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

	}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
	{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();


	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, LEDR_Pin | LEDB_Pin | RS485_TXE_Pin | GSM_RESET_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, DIGOUT1_Pin | DIGOUT2_Pin | CS1_COM_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, ADC1_IN8_ADI5_AUX1_Pin | ADC1_IN9_ADI6_AUX2_Pin | IOLINK1_TXE_Pin | IOLINK2_TXE_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CS_IOLINK_GPIO_Port, CS_IOLINK_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GSM_PWRKEY_GPIO_Port, GSM_PWRKEY_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : AUX3IN_Pin */
	GPIO_InitStruct.Pin = AUX3IN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(AUX3IN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LEDR_Pin LEDB_Pin RS485_TXE_Pin */
	GPIO_InitStruct.Pin = LEDR_Pin | LEDB_Pin | RS485_TXE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : DIP0_Pin DIP1_Pin */
	GPIO_InitStruct.Pin = DIP0_Pin | DIP1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : DIGOUT1_Pin DIGOUT2_Pin CS1_COM_Pin */
	GPIO_InitStruct.Pin = DIGOUT1_Pin | DIGOUT2_Pin | CS1_COM_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : ADC1_IN8_ADI5_AUX1_Pin ADC1_IN9_ADI6_AUX2_Pin IOLINK1_TXE_Pin IOLINK2_TXE_Pin 
	                         CS_IOLINK_Pin */
	GPIO_InitStruct.Pin = ADC1_IN8_ADI5_AUX1_Pin | ADC1_IN9_ADI6_AUX2_Pin | IOLINK1_TXE_Pin | IOLINK2_TXE_Pin 
	                        | CS_IOLINK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : IOLINK_INTR_Pin GSM_INTR_Pin */	// Maybe different IRQ edges !!!
	GPIO_InitStruct.Pin = IOLINK_INTR_Pin | GSM_INTR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : COM_AUX_Pin == CTS1_GSM_Pin */	// Setting may be overwrite by USART1 CTS
	GPIO_InitStruct.Pin = CTS1_GSM_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : GSM_RESET_Pin */
	GPIO_InitStruct.Pin = GSM_RESET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GSM_RESET_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : GSM_PWRKEY_Pin */
	GPIO_InitStruct.Pin = GSM_PWRKEY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GSM_PWRKEY_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : ONEWIRE_Pin */
	GPIO_InitStruct.Pin = ONEWIRE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ONEWIRE_GPIO_Port, &GPIO_InitStruct);
	 }

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
	{
#ifdef DEBUG_PRINT
	printf("Error Handler: file %s on line %d\r\n", file, line);	
#endif // DEBUG_PRINT
	SETPIN(LEDR_GPIO_Port, LEDR_Pin);	
	while (1) 
		{
		}
	}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
	{
	/* User can add his own implementation to report the file name and line number,
	  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	}

#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
