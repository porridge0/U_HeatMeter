/*
 * @file clock.c
 *
 * File containing functions to configure and start the MSP430FR6820 Real Time Clock module.
 *
 * @note Implementation is based on product datasheet and associated user manual.
 *
 * @author Deksios Bekele
 *
 * @date Nov 24, 2020
 *
 */
#include "clock.h"
#include "../signal/constants.h"

const timeStruct resetTime = { .year = 2020, .month = 11, .day = 25, .hour = 6,
		.minute = 59, .second = 59, }; /* Default time loaded when system is reset */
volatile uint16_t prevYear = 2020;
volatile timeStruct currentTime; /*! Global variable holding the current hardware time.
 *  This variable is updated every second */
volatile uint8_t _monChanged = 0;
volatile uint8_t _yrChanged = 0;

volatile uint8_t _sampling_timeUp = 0;
volatile uint8_t _count_samples = 0;
/*!
 * @brief Function Name: rtc_config
 *
 * Configure and initialize RTC registers.
 *
 * @param none
 * @return none
 */
void rtc_config(void) {
	RTCCTL0_H = 0xA5; /* Unlock RTC module */
	RTCCTL1 = RTCHOLD; /* Hold RTC to start configuration */
	RTCCTL1 |= (RTCMODE + RTCTEV__0000); /* BCD mode + Calendar mode + 00:00 changed interrupts */

	RTCCTL0_L = (RTCRDYIE + RTCTEVIE); /* RTC ready interrupt + RTC event interrupt*/
	RTCCTL0_L &= ~RTCRDYIFG;
	RTCCTL0_L &= ~RTCAIFG;
	RTCCTL0_L &= ~RTCTEVIFG;

	RTCCTL1 &= ~(RTCHOLD);  //! enable the RTC module
}

/*!
 * @brief Function Name: rtc_set_time
 *
 * The function updates the current time of the RTC hardware to a new time.
 *
 * @param newTime : New time to be written to RTC hardware
 *
 * @return error code:
 *              \li 0: Success
 *              \li Error codes TBD
 */
int8_t rtc_set_time(timeStruct* pNewTime) {
	RTCYEAR = pNewTime->year;
	RTCMON = pNewTime->month;
	RTCDAY = pNewTime->day;
	RTCDOW = 0; /* Day-of-week is unused, simply initialize to a known value 0 */
	RTCHOUR = pNewTime->hour;
	RTCMIN = pNewTime->minute;
	RTCSEC = pNewTime->second;
	return 0;
}
uint8_t rtc_binary_to_bcd(uint64_t copy, uint8_t* res) {

}

/*!
 * @brief Function Name: RTCC_ISR
 *
 * Interrupt handler for RTC interrupts
 * @todo : Handle Oscillator failure interrupt
 */
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = RTC_VECTOR
__interrupt void RTCC_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(RTC_VECTOR))) RTCC_ISR (void)
#else
#error Compiler not supported!
#endif
{
	switch (__even_in_range(RTCIV, 0x10)) {
	case 0x00: /* no interrupt */
		break;
	case 0x02: /* RTC Oscillator Failure */
		break;
	case 0x04: /* RTC ready; Interrupt Flag: RTCRDYIFG
	 Update current time */
		currentTime.year = RTCYEAR;
		currentTime.month = RTCMON;
		currentTime.day = RTCDAY;
		currentTime.hour = RTCHOUR;
		currentTime.minute = RTCMIN;
		currentTime.second = RTCSEC;
		RTCCTL0_L &= ~RTCRDYIFG;
		if (_count_samples != SAMPLING_RATE)
			_count_samples++;
		else {
			_sampling_timeUp = 1;
			_count_samples = 0;
		}
		break;
	case 0x06: /*RTC interval timer; Interrupt Flag: RTCTEVIFG */
		if (RTCMIN == 0) { /*RTCMINC*/
			/*todo -> hour changed*/
			if (RTCHOUR == 0) {
				/*todo -> day changed*/
				if (RTCDAY == 1) {
					/*todo -> month changed*/
					_monChanged = 1;
				}
				if (RTCYEAR == prevYear + 1) {
					prevYear = RTCYEAR;
					/*todo -> year changed*/
					_yrChanged = 1;
				}
			}
		}
		__bic_SR_register_on_exit(LPM3_bits);
		break;
	case 0x08:
		/*RTC user alarm; Interrupt Flag: RTCAIFG*/
		break;
	case 0x0A:
		/*RTC prescaler 0; Interrupt Flag: RT0PSIFG*/
		break;
	case 0x0C:
		/* RTC prescaler 1; Interrupt Flag: RT1PSIFG */
		break;
	case 0x0E:
		/* Reserved */
		break;
	case 0x10:
		/* Reserved */
		break;
		// no interrupt is pending in the interrupt sevice routine
	default:
		break;

	}
}
