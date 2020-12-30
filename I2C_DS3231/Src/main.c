/* USER CODE BEGIN Header */
/**
 *
 * https://narodstream.ru/stm-urok-9-hal-shina-i2c-prodolzhaem-rabotu-s-ds3231/
 * https://terraideas.ru/article/podklyuchenie-dht11-i-dht22-k-stm32f103c8t6-4
 *
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "string.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DHT_SIZE 5 // size of temp_s
#define DHT_MAX_US 200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

typedef struct {
	uint8_t HI; // целая часть влажности
	uint8_t HD; // дробная часть влажности
	uint8_t TI; // целая часть температуры
	uint8_t TD; // дробная часть температуры
	uint8_t crc; // контрольная сумма
} temp_s;





temp_s dht11_data;

uint8_t aTxBuffer[8];
char str[100];

uint32_t i=0;
uint8_t sec=0, min=0, hour=0, day=0, date=0, month=0, year=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// state - какой уровень считаем
uint8_t DHTGetUs(uint8_t state){

	uint8_t cnt = 0; // считаем сколько us продержался наш уровень, и выходим если что то пошло не так
	while (HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin) == state && (cnt++ < DHT_MAX_US)) {

		//delay_us(1);
		TIM4->CNT = 0;
		while((TIM4->CNT < 2 * 1)) {}
	}
	return cnt;
}


uint8_t DHTRecv(temp_s *data) {
	// для удобства получения данных в цикле
	// представим нашу структуру как указатель на массив байт

	uint8_t *buf = data;
	uint8_t cnt, i, b = 0;
	// Поднимем ногу датчика к питанию на некоторое время, что бы датчик начал работать

	HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET);
	//delay_ms(100); // прижмем к питанию минимум на 18 мкс для DHT 11
	HAL_Delay(100);

	HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET);
	//delay_ms(30); // отпустим ногу
	HAL_Delay(30);
	HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET); // датчик не сразу прижимает ногу в 0

	// по этому ждем некоторое время
	DHTGetUs(1); // теперь, когда нога прижата, ждем пока она прижата
	// 5 тут просто для того что бы убедится что прошло минимум 5 мкс
	if (DHTGetUs(0) < 5) return ERROR; // теперь ждем пока нога поднята
	if (DHTGetUs(1) < 5) return ERROR; // импульс присутствия датчика получен

	// значит нужно начинать получать наши биты // цикл по размеру структуры, в нашем случае 5 байт
	for (b = 0; b < DHT_SIZE; ++b) { // обнулим предыдущие показания
		*buf = 0;
		// получаем 8 бит в первый байт
		for (i = 0; i < 8; ++i) {
			//получаем время на сколько нога была прижата к 0
			cnt = DHTGetUs(0);
			// теперь получаем сколько нога прижата к 1 и сравниваем
			// если к 1 прижата дольше (>) значит пришла единица
			// если нет то пришел 0
			// и пишем это в нашу структуру
			*buf |= (DHTGetUs(1) > cnt) << (7 - i); }
		// сдвигаем указатель на следущий байт структуры
		buf++; }
	// проверяем контрольную сумму
	if (data->crc != data->HD + data->HI + data->TD + data->TI)
		return ERROR;
	return SUCCESS;
}



//перевод двоично-десятичного числа в десятичное
uint8_t RTC_ConvertFromDec(uint8_t c) {

 uint8_t ch = ((c>>4)*10+(0x0F&c));
       return ch;
}

//перевод десятичного числа в двоично-десятичное
uint8_t RTC_ConvertFromBinDec(uint8_t c) {

       uint8_t ch = ((c/10)<<4)|(c%10);
       return ch;
}


void I2C_WriteBuffer(I2C_HandleTypeDef hi, uint8_t DEV_ADDR, uint8_t sizebuf) {

	while(HAL_I2C_Master_Transmit(&hi, (uint16_t)DEV_ADDR,(uint8_t*) &aTxBuffer, (uint16_t)sizebuf, (uint32_t)1000)!= HAL_OK) {

	   if (HAL_I2C_GetError(&hi) != HAL_I2C_ERROR_AF) {

			   sprintf(str, "Buffer error");
			   HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 1000);
	   }
	}
}

void I2C_ReadBuffer(I2C_HandleTypeDef hi, uint8_t DEV_ADDR, uint8_t sizebuf) {

	while(HAL_I2C_Master_Receive(&hi, (uint16_t)DEV_ADDR, (uint8_t*) &aTxBuffer, (uint16_t)sizebuf, (uint32_t)1000)!= HAL_OK) {

	   if (HAL_I2C_GetError(&hi) != HAL_I2C_ERROR_AF) {

			   sprintf(str, "Buffer error");
			   HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 1000);
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
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {

	  if (DHTRecv(&dht11_data) != SUCCESS) {

		  snprintf(str, 63, "Can't read from dht\n");
		  HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 1000);
	  }


     aTxBuffer[0]=0;
     I2C_WriteBuffer(hi2c1,(uint16_t)0xD0,1);

	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {}

	I2C_ReadBuffer(hi2c1,(uint16_t)0xD0,7);

	date=aTxBuffer[4];
	date = RTC_ConvertFromDec(date); //Преобразуем в десятичный формат

	month=aTxBuffer[5];
	month = RTC_ConvertFromDec(month); //Преобразуем в десятичный формат

	year=aTxBuffer[6];
	year = RTC_ConvertFromDec(year); //Преобразуем в десятичный формат

	day=aTxBuffer[3];
	day = RTC_ConvertFromDec(day); //Преобразуем в десятичный формат

	hour=aTxBuffer[2];
	hour = RTC_ConvertFromDec(hour); //Преобразуем в десятичный формат

	min=aTxBuffer[1];
	min = RTC_ConvertFromDec(min); //Преобразуем в десятичный формат

	sec=aTxBuffer[0];
	sec = RTC_ConvertFromDec(sec); //Преобразуем в десятичный формат

	snprintf(str, 63, "%u : %u : %u, T = %d.%d, H = %d.%d\n", hour, min, sec, dht11_data.TI, dht11_data.TD, dht11_data.HI, dht11_data.HD);

	HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 1000);



	HAL_Delay(100);
	i++;
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 36;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart3.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DHT11_Pin */
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);

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
