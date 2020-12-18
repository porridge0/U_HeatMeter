/*!
 * @file system.h
 *
 * File for declaring specific system-wide definitions.
 *
 * @note :
 *
 * @author Deksios Bekele
 *
 * @date Nov 2, 2020
 *
 */
#ifndef CONFIG_SYSTEM_H_
#define CONFIG_SYSTEM_H_

#include <stdint.h>
#include <msp430.h>

typedef enum {
	TRUE, FALSE
} boolean;

#define UHM_CNFG_SMCLK_FREQ 4
#define _ENABLE_SYSTEM_WIDE_DEFS_

#ifdef _ENABLE_SYSTEM_WIDE_DEFS_
void inline __ATOMIZE() {
	__disable_interrupt();
	__no_operation();
}
void inline __END_ATOMIC() {
	__enable_interrupt();
}
void inline sysClock_config() {
	PJSEL0 |= BIT4;  // select pin Xin
	PJSEL1 &= ~BIT4;
	PJSEL0 |= BIT5;  // select pin Xin
	/* Disable the FLL control loop */
	__bis_SR_register(SCG0);
	CSCTL0_H = CSKEY >> 8; /*CSCTL0 = 0xA500;*//* Unlock CS Module */
#if UHM_CNFG_SMCLK_FREQ == 1
	CSCTL1 = DCOFSEL_0; /* Configure DCO for 1MHz operation */
#elif UHM_CNFG_SMCLK_FREQ == 4
	CSCTL1 = DCOFSEL_3; /* Configure DCO for 4MHz operation */
#elif UHM_CNFG_SMCLK_FREQ == 8
	CSCTL1 = DCOFSEL_6; /* Configure DCO for 8MHz operation */
#else
#error SMCLK Configuration not supported
#endif
	CSCTL2 = SELA_0 + SELS_3 + SELM_3;
	CSCTL3 = DIVA_0 + DIVS_0 + DIVM_0;
	//CSCTL4 = HFXTBYPASS + HFXTOFF + LFXTDRIVE_3 + VLOOFF + LFXTOFF;
	CSCTL4 = LFXTDRIVE_3 + VLOOFF + LFXTOFF;
	do {
		CSCTL5 &= ~(LFXTOFFG); /* Clear XT2 fault flag */
		SFRIFG1 &= ~OFIFG;
		/*! @todo this loops forever unless crystal functionality is restored, define an limp-mode operation using RFMODCLK */
	} while (SFRIFG1 & OFIFG); /* Test oscillator fault flag */

	__bic_SR_register(SCG0);

}
#endif

#endif /* CONFIG_SYSTEM_H_ */
