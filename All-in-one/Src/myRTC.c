/*
 * myRTC.c
 *
 *  Created on: 4 янв. 2021 г.
 *      Author: corsair
 */

#include "myRTC.h"



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

	while(HAL_I2C_Master_Transmit(&hi, (uint16_t)DEV_ADDR,(uint8_t*) &RTC_RX_buffer, (uint16_t)sizebuf, (uint32_t)1000)!= HAL_OK) {

	   if (HAL_I2C_GetError(&hi) != HAL_I2C_ERROR_AF) {

			   sprintf(str, "Buffer error");
			   HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 1000);
	   }
	}
}

void I2C_ReadBuffer(I2C_HandleTypeDef hi, uint8_t DEV_ADDR, uint8_t sizebuf) {

	while(HAL_I2C_Master_Receive(&hi, (uint16_t)DEV_ADDR, (uint8_t*) &RTC_RX_buffer, (uint16_t)sizebuf, (uint32_t)1000)!= HAL_OK) {

	   if (HAL_I2C_GetError(&hi) != HAL_I2C_ERROR_AF) {

			   sprintf(str, "Buffer error");
			   HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 1000);
	   }
	}
}
