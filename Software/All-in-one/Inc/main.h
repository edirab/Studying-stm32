/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

#define MAX_SETTINGS_COUNTER 90

// App Control
typedef struct {
	uint8_t lcd_cycle_counter;
	uint8_t sd_cycle_counter;
	uint8_t state;
	uint8_t settings_counter; // Через MAX_SETTINGS_COUNTER секунд если пользователь не произвёл никакого действия, завершаем цикл настройки часов
	uint8_t x10; 			  // или 1, или 10. 1 при уможении ничего не изменит
} App;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Debug_LED_Y_Pin GPIO_PIN_0
#define Debug_LED_Y_GPIO_Port GPIOB
#define Debug_LED_G_Pin GPIO_PIN_1
#define Debug_LED_G_GPIO_Port GPIOB
#define x10_Pin GPIO_PIN_15
#define x10_GPIO_Port GPIOA
#define x10_EXTI_IRQn EXTI15_10_IRQn
#define Set_Pin GPIO_PIN_3
#define Set_GPIO_Port GPIOB
#define Set_EXTI_IRQn EXTI3_IRQn
#define Minus_Pin GPIO_PIN_4
#define Minus_GPIO_Port GPIOB
#define Minus_EXTI_IRQn EXTI4_IRQn
#define Plus_Pin GPIO_PIN_5
#define Plus_GPIO_Port GPIOB
#define Plus_EXTI_IRQn EXTI9_5_IRQn
#define DHT_11_Pin GPIO_PIN_8
#define DHT_11_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
