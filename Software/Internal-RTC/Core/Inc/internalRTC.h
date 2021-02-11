/*
 * internalRTC.h
 *
 *  Created on: 11 февр. 2021 г.
 *      Author: corsair
 */

#ifndef INC_INTERNALRTC_H_
#define INC_INTERNALRTC_H_

#include "main.h"

#define JULIAN_DATE_BASE	2440588

typedef struct
{
	uint8_t RTC_Hours;
	uint8_t RTC_Minutes;
	uint8_t RTC_Seconds;
	uint8_t RTC_Date;
	uint8_t RTC_Wday;
	uint8_t RTC_Month;
	uint16_t RTC_Year;
} RTC_DateTimeTypeDef;

void RTC_GetDateTime(uint32_t RTC_Counter, RTC_DateTimeTypeDef* RTC_DateTimeStruct);

uint32_t RTC_GetRTC_Counter(RTC_DateTimeTypeDef* RTC_DateTimeStruct);

void RTC_GetMyFormat(RTC_DateTimeTypeDef* RTC_DateTimeStruct, char * buffer);




#endif /* INC_INTERNALRTC_H_ */
