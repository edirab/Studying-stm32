/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fatfs_sd.h"
#include "string.h"
#include "stdio.h"

#include "DHT.h"
#include "myRTC.h"
#include "i2c-lcd.h"
#include "bmp280.h"
#include "myFunctions.h"
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
	DHT_type dht_type;
	DHT_data dht_data;

	FATFS fs;  // file system
	FIL fil; // File
	FRESULT fresult;  // result

	char days[7][4] = { "MON\0", "TUE\0", "WED\0", "THU\0", "FRI\0", "SAT\0", "SUN\0" };
	char SD_data[10][88];

	char lcd_upper[16+1];
	char lcd_lower[16+1];
	char buffer[BUFFER_SIZE];  // to store strings..

	RTC_DS3231 myRTC= {0};

	BMP280_HandleTypedef bmp280;
	float pressure, temperature, humidity;
	uint16_t size;

	// > 4000 - разрыв, абсолютно сухая почва,
	// 3700 .. 3850 если взяться пальцами
	// 1400 .. 3000 при погружении в стакан с кипячёнкой
	// < 400 - короткое замыкание проводом по выводам
	uint16_t adc0_soil;

	uint16_t adc0_light1;
	uint16_t adc0_light2;

	App myApp = {0, 0, 0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	__disable_irq();
	switch(GPIO_Pin){
	case Plus_Pin:
		//size = sprintf(buffer, "+\n");
		update_RTC(&myRTC, 1, hi2c1);
		break;

	case Minus_Pin:
		update_RTC(&myRTC, -1, hi2c1);
		//size = sprintf(buffer, "-\n");
		break;

	case Set_Pin:
		myApp.state++;
		//size = sprintf(buffer, "s, state = %d\n", state);
		break;
	default:
		size = sprintf(buffer, "!\n");
		break;
	}
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);
	clear_buffer();
	__enable_irq();
	HAL_Delay(20);
}


uint16_t ADC_Result(ADC_HandleTypeDef *hadc, uint32_t ch){

       ADC_ChannelConfTypeDef sConfig;
       uint16_t adcResult = 0;

       sConfig.Channel = ch;
       sConfig.Rank = ADC_REGULAR_RANK_1;
       sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
       HAL_ADC_ConfigChannel(hadc, &sConfig);

       HAL_ADC_Start(hadc);
       HAL_ADC_PollForConversion(hadc, 100);
       adcResult = HAL_ADC_GetValue(hadc);
       HAL_ADC_Stop(hadc);

       return adcResult;
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
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_FATFS_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  	//set_RTC(hi2c1, &myRTC);

	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c1;


	while (!bmp280_init(&bmp280, &bmp280.params)) {
		size = sprintf(buffer, "BMP280 initialization failed\n");
		HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
		HAL_Delay(2000);
	}
	bool bme280p = bmp280.id == BME280_CHIP_ID;
	size = sprintf(buffer, "BMP280: found %s\n", bme280p ? "BME280" : "BMP280");
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);


	// *******************  �?нициализация карты памяти SD ******************************
	fresult = f_mount(&fs, "/", 1);
	if (fresult != FR_OK) {
		send_uart ("ERROR!!! in mounting SD CARD...\n\n");
	}
	else {
		send_uart("SD CARD mounted successfully...\n\n");
		/* Open file to write/ create a file if it doesn't exist */
	    fresult = f_open(&fil, "data.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);

		/* Move to offset to the end of the file */
		fresult = f_lseek(&fil, f_size(&fil));
		if (fresult == FR_OK)send_uart ("About to update the DATA.TXT\n");
	}

	lcd_init ();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		if (myApp.state == 0) { // Нормальный цикл измерения
			//HAL_GPIO_TogglePin(Debug_GPIO_Port, Debug_Pin);
			HAL_GPIO_WritePin(Debug_LED_Y_GPIO_Port, Debug_LED_Y_Pin, GPIO_PIN_SET);

			// ************************ 1. RTC ***********************************************
			myRTC.RTC_RX_buffer[0] = 0;
			RTC_WriteBuffer(hi2c1, &myRTC, 1);

			while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {}

			RTC_ReadBuffer(hi2c1, &myRTC, 7);

			myRTC.date = BCD_to_DEC(myRTC.RTC_RX_buffer[4]);
			myRTC.month = BCD_to_DEC(myRTC.RTC_RX_buffer[5]);
			myRTC.year = BCD_to_DEC(myRTC.RTC_RX_buffer[6]);
			myRTC.day = BCD_to_DEC(myRTC.RTC_RX_buffer[3]);
			myRTC.hour = BCD_to_DEC(myRTC.RTC_RX_buffer[2]);
			myRTC.min = BCD_to_DEC(myRTC.RTC_RX_buffer[1]);
			myRTC.sec = BCD_to_DEC(myRTC.RTC_RX_buffer[0]);


			// ********************** 2. BMP **************************************************
			//HAL_Delay(100);
			while (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) {
				size = sprintf(buffer, "BMP280 reading failed\n");
				HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);
				HAL_Delay(500);
			}


			// ********************** 3. DHT ****************************************************
			if (myApp.lcd_cycle_counter == 3 || myApp.lcd_cycle_counter == 6){
				dht_data = DHT_getData(DHT11);
			}


			// ********************** 4. Опрос АЦП ********************************************
			adc0_soil   = ADC_Result(&hadc1, 0);
			adc0_light1 = ADC_Result(&hadc1, 1);
			adc0_light2 = ADC_Result(&hadc1, 2);

			// ********************** 5. Передача по UART ***************************************
			clear_buffer();
			// 87 символов в этой строке
			snprintf(buffer, 100, "%02u:%02u:%02u %s %02u/%02u/%02u T = %.2f*C H = %02d%% P = %.2f Pa S = %04d L1 = %04d L2 = %04d\n",
																myRTC.hour, myRTC.min, myRTC.sec,
																days[myRTC.day-1],
																myRTC.date, myRTC.month, myRTC.year,
																temperature, (uint8_t)dht_data.hum, pressure,
																adc0_soil, adc0_light1, adc0_light2);
			HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);


			// *********************** 6. Вывод на LCD ******************************************
			// NB! S-N-PRINTF !!! not a S-PRINTF
			snprintf(lcd_upper, 17, "%02u:%02u %02u/%02u/%02u", myRTC.hour, myRTC.min, myRTC.date, myRTC.month, myRTC.year);

			if (myApp.lcd_cycle_counter < 3){
				snprintf(lcd_lower, 17, "%.1f*C  %.1f mm", temperature, pressure/133.3244);
				//lcd_lower[0] = 'A';
			} else {
				//snprintf(lcd_lower, 17, "%.1f*C  %d%%     ", temperature, (uint8_t)dht_data.hum);
				snprintf(lcd_lower, 17, "%.1f*C  %02d%%  %s", temperature, (uint8_t)dht_data.hum, days[myRTC.day-1]);
			}

			//lcd_clear ();
			lcd_put_cur(0, 0);
			lcd_send_string(lcd_upper);
			lcd_put_cur(1, 0);
			lcd_send_string(lcd_lower);

			//snprintf(buffer, 100, "%02u\n", cycle_counter);
			//HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);

			// *********************** 7. Запись на SD-карту ***********************************
			if (myApp.sd_cycle_counter == 10) {

				for (uint8_t i = 0; i < 10; i++){
					fresult = f_puts(SD_data[i], &fil);
				}
				f_sync(&fil);

				// Clear SD_data buffer
				for (uint8_t i = 0; i < 10; i++){
					for (uint8_t j = 0; j < 88; j++){
						SD_data[i][j] = '\0';
					}
				}
				clear_buffer();

				myApp.sd_cycle_counter = 0;
			}
			else {

				memcpy(SD_data[myApp.sd_cycle_counter], buffer, strlen(buffer));
				myApp.sd_cycle_counter++;
			}

			myApp.lcd_cycle_counter++;


			if (myApp.lcd_cycle_counter == 6) myApp.lcd_cycle_counter = 0;

			HAL_GPIO_WritePin(Debug_LED_Y_GPIO_Port, Debug_LED_Y_Pin, GPIO_PIN_RESET);
			HAL_Delay(975);
		}

		// **************************************************************************************
		// **************************  Настройка часов  *****************************************
		// **************************************************************************************
		else if (myApp.state > 0 && myApp.state < 6) {

			lcd_clear ();
			// �?сключаем мигание дисплея при обновлении
			while(myApp.state > 0 && myApp.state < 6){

				lcd_put_cur(0, 0);
				snprintf(lcd_upper, 17, "%02u:%02u %02u/%02u/%02u", myRTC.hour, myRTC.min, myRTC.date, myRTC.month, myRTC.year);
				lcd_send_string(lcd_upper);
				//HAL_Delay(500);

				switch(myApp.state){

				case 1:
					lcd_put_cur(0, 0);
					snprintf(lcd_upper, 17, "  :%02u %02u/%02u/%02u", /* myRTC.hour,*/ myRTC.min, myRTC.date, myRTC.month, myRTC.year);
					lcd_send_string(lcd_upper);
					//HAL_Delay(500);
					break;
				case 2:
					lcd_put_cur(0, 0);
					snprintf(lcd_upper, 17, "%02u:   %02u/%02u/%02u", myRTC.hour, /* myRTC.min,*/ myRTC.date, myRTC.month, myRTC.year);
					lcd_send_string(lcd_upper);
					//HAL_Delay(500);
					break;
				case 3:
					lcd_put_cur(0, 0);
					snprintf(lcd_upper, 17, "%02u:%02u   /%02u/%02u", myRTC.hour, myRTC.min, /* myRTC.date,*/ myRTC.month, myRTC.year);
					lcd_send_string(lcd_upper);
					//HAL_Delay(500);
					break;
				case 4:
					lcd_put_cur(0, 0);
					snprintf(lcd_upper, 17, "%02u:%02u %02u/  /%02u", myRTC.hour, myRTC.min, myRTC.date, /*myRTC.month,*/ myRTC.year);
					lcd_send_string(lcd_upper);
					//HAL_Delay(500);
					break;
				case 5:
					lcd_put_cur(0, 0);
					snprintf(lcd_upper, 17, "%02u:%02u %02u/%02u/  ", myRTC.hour, myRTC.min, myRTC.date, myRTC.month /*myRTC.year*/);
					lcd_send_string(lcd_upper);
					//HAL_Delay(500);
					break;

				case 6:
					snprintf(lcd_upper, 17, "Set time to %02u:%02u %02u/%02u/%02u", myRTC.hour, myRTC.min, myRTC.date, myRTC.month, myRTC.year);
					HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);
					set_RTC(hi2c1, &myRTC);

					lcd_put_cur(1, 0);
					snprintf(lcd_lower, 17, "Saved");
					lcd_send_string(lcd_lower);
					//HAL_Delay(500);
					myApp.state = 0;
					break;
				}
			}
		}
//		else if (state == 6) {
//			snprintf(lcd_upper, 17, "Set time to %02u:%02u %02u.%02u.%02u", myRTC.hour, myRTC.min, myRTC.date, myRTC.month, myRTC.year);
//			HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);
//			set_RTC(hi2c1, &myRTC);
//		}

		else {
			size = sprintf(buffer, "Unknown execution state\n");
			HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);
			myApp.state = 0;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Debug_LED_Y_Pin|Debug_LED_G_Pin|DHT_11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Debug_LED_Y_Pin Debug_LED_G_Pin DHT_11_Pin */
  GPIO_InitStruct.Pin = Debug_LED_Y_Pin|Debug_LED_G_Pin|DHT_11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Set_Pin Minus_Pin Plus_Pin */
  GPIO_InitStruct.Pin = Set_Pin|Minus_Pin|Plus_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
