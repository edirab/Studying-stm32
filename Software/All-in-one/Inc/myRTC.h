#ifndef MY_RTC_H
#define MY_RTC_H


#include <stdio.h>
#include <string.h> // correct header

#include "main.h"

#define DEVICE_ADDR_RTC 0xD0

extern UART_HandleTypeDef huart3;
extern char buffer[];
extern App myApp;

typedef struct {

	uint8_t RTC_RX_buffer[8];
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint8_t day; //e.i. MON, TUE, WED, THU, FRI
	uint8_t date;
	uint8_t month;
	uint8_t year;
} RTC_DS3231;

//перевод двоично-десятичного числа в десятичное
uint8_t BCD_to_DEC(uint8_t c);


//перевод десятичного числа в двоично-десятичное
uint8_t DEC_to_BCD(uint8_t c);


void RTC_WriteBuffer(I2C_HandleTypeDef hi, RTC_DS3231 *myRTC, uint8_t sizebuf);

void RTC_ReadBuffer(I2C_HandleTypeDef hi, RTC_DS3231 *myRTC, uint8_t sizebuf);

void set_RTC(I2C_HandleTypeDef hi, RTC_DS3231 *myRTC);

void update_RTC(RTC_DS3231 *myRTC, int8_t direction, I2C_HandleTypeDef hi);

#endif
