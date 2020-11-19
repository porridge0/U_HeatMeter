/*!
 * @file main.c
 *
 * File containing main routines for metering, display and communication functionalities
 * in the ultrasonic heat meter.
 *
 * @author Deksios Bekele
 *
 * @date Oct 12, 2020
 *
 */
#include <msp430.h> 
#include <stdio.h>
#include <stdlib.h>
#include "config/pin_io.h"
#include "comm/app.h"
#include "comm/phy.h"
#include "util/params.h"
/*
 * main.c
 */
/*!-- GLOBALS */

extern volatile uint16_t current_baud;
extern volatile uint8_t tx_buff[];
extern volatile snd_req_status c_status;


int main(void) {
	// Disable the GPIO power-on default high-impedance mode to activate
	// previously configured port settings
	PM5CTL0 &= ~LOCKLPM5;
	WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

	uint8_t *pa_changed_status = (uint8_t *) calloc(1,sizeof(uint8_t));

	_enable_interrupt();
	__bis_SR_register(GIE);

	while(1){
		if(c_status == response_ready){
			x_lnk_res();
		}
	}
}
