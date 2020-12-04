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
/*monitor flags*/
extern volatile uint8_t _monChanged;
extern volatile uint8_t _yrChanged;

extern volatile timeStruct currentTime;
extern const timeStruct resetTime;

extern volatile uint8_t _sampling_timeUp;

void rtc_config(void);
int8_t rtc_set_time(timeStruct* pNewTime);
uint8_t rtc_binary_to_bcd(uint64_t copy, uint8_t* res);

#endif /* CLOCK_H_ */
