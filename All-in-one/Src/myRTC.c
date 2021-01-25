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

			   sprintf(buffer, "Buffer error");
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

			   sprintf(buffer, "Buffer error");
			   HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	   }
	}
}

void set_RTC(I2C_HandleTypeDef hi, RTC_DS3231 *myRTC){

	uint8_t data[8];

	//data[0] = 0;	//
	//data[1] = DEC_to_BCD(0);	// sec
	//data[2] = DEC_to_BCD(20);	// min
	//data[3] = DEC_to_BCD(14);	// hour
	//data[4] = DEC_to_BCD(1);	// day - SUN
	//data[5] = DEC_to_BCD(25);	// date
	//data[6] = DEC_to_BCD(1);	// month
	//data[7] = DEC_to_BCD(21);	// year - 2021

	data[0] = 0;	//
	data[1] = DEC_to_BCD(myRTC->sec);	// sec
	data[2] = DEC_to_BCD(myRTC->min);	// min
	data[3] = DEC_to_BCD(myRTC->hour);	// hour
	data[4] = DEC_to_BCD(1);	// day - SUN
	data[5] = DEC_to_BCD(myRTC->date);	// date
	data[6] = DEC_to_BCD(myRTC->month);	// month
	data[7] = DEC_to_BCD(myRTC->year);	// year - 2021

	while(HAL_I2C_Master_Transmit(&hi, (uint16_t)DEVICE_ADDR_RTC, (uint8_t*) &data, 8, (uint32_t)1000)!= HAL_OK) {

	   if (HAL_I2C_GetError(&hi) != HAL_I2C_ERROR_AF) {
			   sprintf(buffer, "Buffer error");
			   HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
	   }
	}

	//RTC_WriteBuffer(hi, (uint16_t)DEVICE_ADDR_RTC, 7);
}

void update_RTC(RTC_DS3231 *myRTC, uint8_t direction){

	if(direction){
		switch(state){
		case 1:
			myRTC->hour += 1 * direction;
			break;
		case 2:
			myRTC->min += 1 * direction;
			break;
		case 3:
			myRTC->date += 1 * direction;
			break;
		case 4:
			myRTC->month += 1 * direction;
			break;
		case 5:
			myRTC->year += 1 * direction;
			break;
		default:
			sprintf(buffer, "State is %d, nothing to do\n", state);
		    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 100);
			break;
		}
	}


}
