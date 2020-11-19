/*!
 * @file clock.h
 *
 *	File containing function definitions to operate the Real Time Clok (RTC) module of the MSP430FR6820 device.
 *
 *  @date: Nov 18, 2020
 *
 *  @author: User
 */
#ifndef CLOCK_H_
#define CLOCK_H_

#include <stdint.h>
#include <msp430.h>

/*! Date-time structure */
typedef struct {
	uint8_t second;
	uint8_t minute;
	uint8_t hour;
	uint8_t day;
	uint8_t month;
	uint16_t year;
} timeStruct;

extern volatile timeStruct rtcCurrentTime;
extern timeStruct RTC_ResetTime;

void RTC_Setup(void);
void RTC_TimeStamp2Ascii(volatile timeStruct* time, uint8_t* buff);
uint8_t RTC_Ascii2TimeStamp(uint8_t* buff, timeStruct* time);
int8_t RTC_SetClock(timeStruct* pNewTime);
uint8_t binary_to_BCD(uint64_t copy, uint8_t* res);

#endif /* CLOCK_H_ */
