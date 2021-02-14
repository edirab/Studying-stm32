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
#include "adc.h"
#include "fatfs.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */
	DHT_type dht_type;
	DHT_data dht_data;

	FATFS fs;  // file system
	FIL fil; // File
	FRESULT fresult;  // result

	char days[7][4] = {"SUN\0", "MON\0", "TUE\0", "WED\0", "THU\0", "FRI\0", "SAT\0"}; // <-----
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

	App myApp = {0, 0, 0, 0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	__disable_irq();

	// Фиксируем, что произошло действие от пользователя
	myApp.settings_counter = 0;

	switch(GPIO_Pin){
	case Plus_Pin:
		//size = sprintf(buffer, "+\n");
		update_RTC(&myRTC, 1, hi2c1);
		break;

	case Minus_Pin:
		update_RTC(&myRTC, -1, hi2c1);
		//size = sprintf(buffer, "-\n");
		break;

	// По каждому изменению происходит сохранение
	case Set_Pin:
		myApp.state++;
		//size = sprintf(buffer, "s, state = %d\n", state);
		break;
	default:
		//size = sprintf(buffer, "!\n");
		break;
	}
	//HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);
	//clear_buffer();
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
  MX_IWDG_Init();
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
	HAL_IWDG_Refresh(&hiwdg);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		// **************************************************************************************
		// **************************  Нормальный цикл измерения  *******************************
		// **************************************************************************************
		if (myApp.state == 0) {

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
			bool bmp_status = bmp280_read_float(&bmp280, &temperature, &pressure, &humidity);

			if (!bmp_status){
				size = sprintf(buffer, "BMP280 reading failed\n");
				HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);
			}

			// ********************** 3. DHT ****************************************************
			if (myApp.lcd_cycle_counter % 3 == 0){
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
																days[myRTC.day - 1],
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
				snprintf(lcd_lower, 17, "%.1f*C  %02d%%  %s", temperature, (uint8_t)dht_data.hum, days[myRTC.day - 1]);
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


			if (myApp.lcd_cycle_counter >= 6) myApp.lcd_cycle_counter = 0;

			HAL_GPIO_WritePin(Debug_LED_Y_GPIO_Port, Debug_LED_Y_Pin, GPIO_PIN_RESET);
			HAL_Delay(975);
			HAL_IWDG_Refresh(&hiwdg);
		}

		// **************************************************************************************
		// **************************  Настройка часов  *****************************************
		// **************************************************************************************
		else if (myApp.state > 0 && myApp.state < 9) {

			lcd_clear();
			for (uint8_t i = 0; i < 17; i++){
				lcd_upper[i] = '\0';
				lcd_lower[i] = '\0';
			}

			while(myApp.state > 0 && myApp.state < 9)
			{
				// Считываем значение из часов
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


				snprintf(lcd_upper, 17, "%02u:%02u:%02u", myRTC.hour, myRTC.min, myRTC.sec);
				snprintf(lcd_lower, 17, "%02u/%02u/%02u %s", myRTC.date, myRTC.month, myRTC.year, days[myRTC.day - 1]);

				//lcd_clear();
				lcd_put_cur(0, 0);
				lcd_send_string(lcd_upper);
				lcd_put_cur(1, 0);
				lcd_send_string(lcd_lower);
				HAL_Delay(500);

				switch(myApp.state){

				// часы
				case 1:
					snprintf(lcd_upper, 17, "  :%02u:%02u", /* myRTC.hour,*/ myRTC.min, myRTC.sec);
					break;
				// минуты
				case 2:
					snprintf(lcd_upper, 17, "%02u:  :%02u", myRTC.hour, /* myRTC.min,*/ myRTC.sec);
					break;
				// секунды
				case 3:
					snprintf(lcd_upper, 17, "%02u:%02u:  ", myRTC.hour, myRTC.min /* myRTC.sec,*/);
					break;
				// число
				case 4:
					snprintf(lcd_lower, 17, "  /%02u/%02u %s", /*myRTC.date*/ myRTC.month, myRTC.year, days[myRTC.day - 1]);
					break;
				// месяц
				case 5:
					snprintf(lcd_lower, 17, "%02u/  /%02u %s", myRTC.date, /*myRTC.month,*/ myRTC.year, days[myRTC.day - 1]);
					break;
				// год
				case 6:
					snprintf(lcd_lower, 17, "%02u/%02u/   %s", myRTC.date, myRTC.month /*myRTC.year*/, days[myRTC.day - 1]);
					break;
				// день недели
				case 7:
					snprintf(lcd_lower, 17, "%02u/%02u/%02u    ", myRTC.date, myRTC.month, myRTC.year /*days[myRTC.day]*/);
					break;
				// сохранено!
				case 8:
					snprintf(buffer, 100, "Set time to %02u:%02u:%02u %s %02u/%02u/%02u", myRTC.hour, myRTC.min, myRTC.sec, days[myRTC.day-1], myRTC.date, myRTC.month, myRTC.year);
					HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);

					lcd_clear();
					lcd_put_cur(1, 0);
					snprintf(lcd_lower, 17, "Saved");
					myApp.state = 0;
					myApp.settings_counter = 0;
					break;
				}

				lcd_put_cur(0, 0);
				lcd_send_string(lcd_upper);
				lcd_put_cur(1, 0);
				lcd_send_string(lcd_lower);

				HAL_Delay(500);
				HAL_IWDG_Refresh(&hiwdg);

				// Выходим из настройки, если пользователь ничего не сделал в течение 30 секунд
				myApp.settings_counter++;
				if (myApp.settings_counter == MAX_SETTINGS_COUNTER){
					//break;
					myApp.state = 8;
				}
			} // end while(myApp.state > 0 && myApp.state < 9)
			clear_buffer();
		}
		else {
			size = sprintf(buffer, "Unknown execution state\n");
			HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);
			myApp.state = 0;
			clear_buffer();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
	snprintf(buffer, 100, "SError Handler\n");
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);
	clear_buffer();
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
