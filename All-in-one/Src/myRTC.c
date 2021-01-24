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

/*
 * Записывает содержимое буфера RTC_RX_buffer[7] в часы реального времени
 * */
void RTC_WriteBuffer(I2C_HandleTypeDef hi, uint8_t DEV_ADDR, uint8_t sizebuf) {

	while(HAL_I2C_Master_Transmit(&hi, (uint16_t)DEV_ADDR,(uint8_t*) &RTC_RX_buffer, (uint16_t)sizebuf, (uint32_t)1000)!= HAL_OK) {

	   if (HAL_I2C_GetError(&hi) != HAL_I2C_ERROR_AF) {

			   sprintf(str, "Buffer error");
			   HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 1000);
	   }
	}
}

/*
 *Вычитывает 7 байт в соответствующий буфер
 * */
void RTC_ReadBuffer(I2C_HandleTypeDef hi, uint8_t DEV_ADDR, uint8_t sizebuf) {

	while(HAL_I2C_Master_Receive(&hi, (uint16_t)DEV_ADDR, (uint8_t*) &RTC_RX_buffer, (uint16_t)sizebuf, (uint32_t)1000)!= HAL_OK) {

	   if (HAL_I2C_GetError(&hi) != HAL_I2C_ERROR_AF) {

			   sprintf(str, "Buffer error");
			   HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 1000);
	   }
	}
}

void set_RTC(I2C_HandleTypeDef hi){

	uint8_t data[8];

	data[0] = 0;	//
	data[1] = RTC_ConvertFromBinDec(0);	// sec
	data[2] = RTC_ConvertFromBinDec(49);	// min
	data[3] = RTC_ConvertFromBinDec(19);	// hour
	data[4] = RTC_ConvertFromBinDec(7);	// day - SUN
	data[5] = RTC_ConvertFromBinDec(24);	// date
	data[6] = RTC_ConvertFromBinDec(1);	// month
	data[7] = RTC_ConvertFromBinDec(21);	// year - 2021

	while(HAL_I2C_Master_Transmit(&hi, (uint16_t)DEVICE_ADDR_RTC, (uint8_t*) &data, 8, (uint32_t)1000)!= HAL_OK) {

	   if (HAL_I2C_GetError(&hi) != HAL_I2C_ERROR_AF) {
			   sprintf(str, "Buffer error");
			   HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 1000);
	   }
	}

	//RTC_WriteBuffer(hi, (uint16_t)DEVICE_ADDR_RTC, 7);
}
