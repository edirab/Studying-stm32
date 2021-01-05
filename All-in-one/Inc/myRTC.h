#ifndef MY_RTC_H
#define MY_RTC_H


#include <stdio.h>
 #include <string.h> // correct header

#include "main.h"

#define DEVICE_ADDR_RTC 0xD0

extern UART_HandleTypeDef huart3;
extern uint8_t RTC_RX_buffer[];
extern char str[];

//перевод двоично-десятичного числа в десятичное
uint8_t RTC_ConvertFromDec(uint8_t c);


//перевод десятичного числа в двоично-десятичное
uint8_t RTC_ConvertFromBinDec(uint8_t c);


void I2C_WriteBuffer(I2C_HandleTypeDef hi, uint8_t DEV_ADDR, uint8_t sizebuf);


void I2C_ReadBuffer(I2C_HandleTypeDef hi, uint8_t DEV_ADDR, uint8_t sizebuf);


#endif
