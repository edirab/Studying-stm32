/*
 * myRTC.c
 *
 *  Created on: 4 янв. 2021 г.
 *      Author: corsair
 */

#include "myRTC.h"



//перевод двоично-десятичного числа в десятичное
uint8_t BCD_to_DEC(uint8_t c) {

 uint8_t ch = ((c>>4)*10+(0x0F&c));
       return ch;
}

//перевод десятичного числа в двоично-десятичное RTC_ConvertFromBinDec
uint8_t DEC_to_BCD(uint8_t c) {

       uint8_t ch = ((c/10)<<4)|(c%10);
       return ch;
}

/*
 * Записывает содержимое буфера RTC_RX_buffer[7] в часы реального времени
 * */
void RTC_WriteBuffer(I2C_HandleTypeDef hi, RTC_DS3231 *myRTC, uint8_t sizebuf) {

	while(HAL_I2C_Master_Transmit(&hi, (uint16_t)DEVICE_ADDR_RTC, (uint8_t*)myRTC->RTC_RX_buffer, (uint16_t)sizebuf, (uint32_t)1000)!= HAL_OK) {

	   if (HAL_I2C_GetError(&hi) != HAL_I2C_ERROR_AF) {

			   sprintf(buffer, "Buffer error in RTC_WriteBuffer \n");
			   HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	   }
	}
}

/*
 *Вычитывает 7 байт в соответствующий буфер
 * */
void RTC_ReadBuffer(I2C_HandleTypeDef hi, RTC_DS3231 *myRTC, uint8_t sizebuf) {

	while(HAL_I2C_Master_Receive(&hi, (uint16_t)DEVICE_ADDR_RTC, (uint8_t*)myRTC->RTC_RX_buffer, (uint16_t)sizebuf, (uint32_t)1000)!= HAL_OK) {

	   if (HAL_I2C_GetError(&hi) != HAL_I2C_ERROR_AF) {

			   sprintf(buffer, "Buffer error in 'RTC_ReadBuffer'\n");
			   HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	   }
	}
}

void set_RTC(I2C_HandleTypeDef hi, RTC_DS3231 *myRTC){

	uint8_t data[8];

	data[0] = 0;	//
	data[1] = DEC_to_BCD(myRTC->sec);	// sec
	data[2] = DEC_to_BCD(myRTC->min);	// min
	data[3] = DEC_to_BCD(myRTC->hour);	// hour
	data[4] = DEC_to_BCD(myRTC->day);	// day - SUN
	data[5] = DEC_to_BCD(myRTC->date);	// date
	data[6] = DEC_to_BCD(myRTC->month);	// month
	data[7] = DEC_to_BCD(myRTC->year);	// year - 2021


	__disable_irq();
	HAL_GPIO_WritePin(Debug_LED_G_GPIO_Port, Debug_LED_G_Pin, GPIO_PIN_SET);
	while(HAL_I2C_Master_Transmit(&hi, (uint16_t)DEVICE_ADDR_RTC, (uint8_t*) &data, 8, (uint32_t)1000)!= HAL_OK) {

	   if (HAL_I2C_GetError(&hi) != HAL_I2C_ERROR_AF) {
			   sprintf(buffer, "Buffer error in set_RTC \n");
			   HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	   }
	}
	HAL_GPIO_WritePin(Debug_LED_G_GPIO_Port, Debug_LED_G_Pin, GPIO_PIN_RESET);
	__enable_irq();
	//RTC_WriteBuffer(hi, (uint16_t)DEVICE_ADDR_RTC, 7);
}

void update_RTC(RTC_DS3231 *myRTC, int8_t direction, I2C_HandleTypeDef hi){

		switch(myApp.state){
		case 1:
			// Избавляемся от проблемы переполнения прибавлением периода
			if (direction == -1) myRTC->hour += 24;
			myRTC->hour += 1 * direction;
			myRTC->hour %= 24;
			break;
		case 2:
			if (direction == -1) myRTC->min += 60;
			myRTC->min += 1 * direction;
			myRTC->min %= 60;
			break;

		case 3:
			myRTC->sec = 0; // секунды всегда сбрасываем по любому нажатию
			break;

		case 4:
			if (direction == -1) myRTC->date += 32;
			myRTC->date += 1 * direction;
			myRTC->date %= 32;
			break;
		case 5:
			if (direction == -1) myRTC->month += 13;
			myRTC->month += 1 * direction;
			myRTC->month %= 13;
			break;
		case 6:
			if (direction == -1) myRTC->year += 100;
			myRTC->year += 1 * direction;
			myRTC->year %= 100;
			break;
		case 7:
			if (direction == -1) myRTC->day += 7;
			myRTC->day += 1 * direction;
			myRTC->day %= 7;
			break;

		default:
			sprintf(buffer, "Dir %d, State is %d, nothing to do\n", direction, myApp.state);
		    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);
			break;
		}
		set_RTC(hi, myRTC);


}
