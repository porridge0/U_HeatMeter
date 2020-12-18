/*!
 * @file phy.c
 *
 * File for configuring the physical and link layers' of the M-Bus
 * communication protocol according to the EN 13757-2 standard.
 *
 * @note Implementation is based on industry recommended practices.
 *
 * @author Deksios Bekele
 *
 * @date Oct 12, 2020
 *
 */
#include <msp430.h>
#include <assert.h>
#include "phy.h"
#include "app.h"
#include <stdio.h>
#include "../config/pin_io.h"
#include "../config/system.h"

void (*cb_t)(void);
/*!-- GLOBALS */
uint8_t rx_index = 0; //0-indexed
volatile lnk_frame *r_frame;
volatile lnk_frame *rsp_frame;
volatile uint8_t ud_buff[MBUS_FRAME_MAX_USER_DATA_SIZE];
volatile uint8_t tx_buff[MBUS_FRAME_MAX_USER_DATA_SIZE];
uint8_t ud_buff_index = 1;
uint8_t tx_buff_index = 0;
uint8_t c_tx_length = 0;
volatile snd_req_status c_status = waiting_for_master;
volatile uint16_t current_baud = THREE_HUNDERED; // Track working baud
volatile uint8_t r_delay = 0;
volatile uint16_t auto_br_delay = 0;

/*Slave addresses*/
volatile uint8_t PRIMARY_ADDRESS = 0x00;
volatile uint32_t SERIAL_NUMBER = 0x00000000;

/*@note :
 * 	Product date format :
 * 	December 18,2020
 *  Format : 18/12/20
 *
 */

#pragma PERSISTENT(devInfo)
deviceInfo devInfo = { .primaryAddress = 0, .serialNumber = 0x00000000,
		.productNumber = 0x00000000, .productDate = 0x120C14 };

/*!
 * @brief Function Name: set_baudRate
 *
 * Sets the new selected working baudrate for the bus.
 *
 * @param : the new baudrate to set
 *
 * @return none
 */
void set_baudRate(uint16_t baudRate) {

	/* Set baudrate */
	switch (baudRate) {
	case THREE_HUNDERED: //300
		MBUS_UART_UCxCTL1 |= UCSSEL__ACLK; /* ACLK is the clock source */
		MBUS_UART_UCxBRW = 0x6D;
		MBUS_UART_UCxMCTLW = 0x44;
		break;
	case SIX_HUNDRED: //600
		MBUS_UART_UCxCTL1 |= UCSSEL__ACLK; /* ACLK is the clock source */
		MBUS_UART_UCxBRW = 0x36;
		MBUS_UART_UCxMCTLW = 0x00B5;
		break;
	case TWELVE_HUNDRED: //1200
		MBUS_UART_UCxCTL1 |= UCSSEL__ACLK; /* ACLK is the clock source */
		MBUS_UART_UCxBRW = 0x1B;
		MBUS_UART_UCxMCTLW = 0x12;
		break;
	case TWENTY_FOUR_HUNDRED: //2400
		MBUS_UART_UCxCTL1 |= UCSSEL__ACLK; /* ACLK is the clock source */
		MBUS_UART_UCxBRW = 0x0D;
		MBUS_UART_UCxMCTLW = 0x0C;
		break;
	case FOURTY_EIGHT_HUNDRED: //4800
		MBUS_UART_UCxCTL1 |= UCSSEL__ACLK; /* ACLK is the clock source */
		MBUS_UART_UCxBRW = 0x06;
		MBUS_UART_UCxMCTLW = 0x6F;
		break;
	case NINETY_SIX_HUNDRED: //9600
		MBUS_UART_UCxCTL1 |= UCSSEL__SMCLK; /* SMCLK is the clock source */
		MBUS_UART_UCxBRW = 52;
		MBUS_UART_UCxMCTLW = 0x4911;
		break;
	case NINETEEN_THOUSAND_TWO_HUNDRED: //19200
		MBUS_UART_UCxCTL1 |= UCSSEL__SMCLK; /* SMCLK is the clock source */
		MBUS_UART_UCxBRW = 26;
		MBUS_UART_UCxMCTLW = 0xB601;
		break;

	default: //300 is the default baudrate
		MBUS_UART_UCxCTL1 |= UCSSEL__ACLK; /* ACLK is the clock source */
		MBUS_UART_UCxBRW = 0x6D;
		MBUS_UART_UCxMCTLW = 0x44;
		break;
	}
	current_baud = baudRate; /* Set baud */
}

/*!
 * @brief Function Name: ux_config
 *
 * Initializes the UART module registers with the appropriate configuration parameters
 *
 * @param The operating baudrate.
 *
 * @return none
 */
void ux_config(uint16_t baudRate) {

	/* Initialize GPIO port */
	MBUS_TX_PxSEL0 &= ~MBUS_TX_BIT; /* Reset bit first....  */
	MBUS_TX_PxSEL0 |= MBUS_TX_PxSEL0_VAL; /* ....Then set the desired value */
	MBUS_TX_PxSEL1 &= ~MBUS_TX_BIT; /* Reset bit first....  */
	MBUS_TX_PxSEL1 |= MBUS_TX_PxSEL1_VAL; /* ....Then set the desired value */

	MBUS_RX_PxSEL0 &= ~MBUS_RX_BIT; /* Reset bit first....  */
	MBUS_RX_PxSEL0 |= MBUS_RX_PxSEL0_VAL; /* ....Then set the desired value */
	MBUS_RX_PxSEL1 &= ~MBUS_RX_BIT; /* Reset bit first....  */
	MBUS_RX_PxSEL1 |= MBUS_RX_PxSEL1_VAL; /* ....Then set the desired value */

	/* Configure MSP uart module */

	MBUS_UART_UCxCTL1 |= UCSWRST; /* disable and reset to allow configuration */
	MBUS_UART_UCxCTL0 |= UCPEN; /* enable parity */
	MBUS_UART_UCxCTL0 &= ~UCMSB; /* LSB first  default */
	MBUS_UART_UCxCTL0 &= ~UC7BIT; /* 8 -bit    default */
	MBUS_UART_UCxCTL0 &= ~UCSPB; /* one stop bit default */
	MBUS_UART_UCxCTL0 |= UCMODE_0; /* UART Mode default */
	MBUS_UART_UCxCTL0 &= ~UCSYNC; /* Asynchronous mode default */

	/*Set a default buadrate of 300*/
	set_baudRate(baudRate);
	//UCA0STATW |= UCLISTEN;

	MBUS_UART_UCxCTL1 &= ~UCSWRST; /* enable module to apply configuration */
	MBUS_UART_UCxIE |= UCRXIE; /* Enable RX interrupt */
	MBUS_UART_UCxIFG &= ~(UCTXIFG + UCRXIFG); /* Clear interrupts if set */
}

/*!
 * @brief Function Name: x_reset
 *
 * Resets frame parameters.
 *
 * @param none
 *
 * @return none
 */
void inline x_reset() {
	__ATOMIZE();
	//! Reset any previously cached datagram monitors
	rx_index = 0;
	r_frame->l_fieldL = 0;
	r_frame->l_fieldH = 0;
	r_frame->checksum = 0;
	c_status = waiting_for_master;
	__END_ATOMIC();
}
/*!
 * @brief Function Name: rx_lnk_frame
 *
 * Constructs a complete MBUS frame from each byte received in the UART RX buffer.
 * A timer is started with each new received frame to detect inter-frame timeout/idle time.
 * Full telegram reception is signalled by setting a flag and the message processed in the main thread.
 *
 * @param received byte in the UART RX buffer
 *
 * @return none
 */
void inline rx_lnk_frame(uint8_t data) {
	if (rx_index == 0 && c_status == waiting_for_master) {
		/*!-- start inter-byte timer */
		TA2R = 0;
		TA2CCR0 = 32768; // 32768/4 = 8192 ~ 8KHz

		if (data == MBUS_FRAME_SHORT_START) {
			r_frame->type = short_frame;
		} else if (data != MBUS_FRAME_CONTROL_SIZE) {
			/*!-- Telegram error*/
			return;
		}
		/* TODO:
		 * 	 reset timer
		 * 	 redundant validity checker
		 */
		r_frame->start = data;
		rx_index++;
	} else if (rx_index == 1) {
		TA2R = 0;
		if (r_frame->type == short_frame) {
			r_frame->c_field = data;
		} else {
			if (data == MBUS_FRAME_CONTROL_SIZE) {
				r_frame->type = control_frame;
			} else {
				r_frame->type = long_frame;
			}
			r_frame->l_fieldL = data;
		}
		rx_index++;
	} else if (rx_index == 2) {
		TA2R = 0;
		if (r_frame->type == short_frame) {
			r_frame->a_field = data;
		} else {
			assert(data == r_frame->l_fieldL);
			r_frame->l_fieldH = data;
		}
		rx_index++;
	} else if (rx_index == 3) {
		TA2R = 0;
		if (r_frame->type == short_frame) {
			r_frame->checksum = data;
		} else {
			assert(data == MBUS_FRAME_CONTROL_OR_LONG_START);
			r_frame->startR = data;
		}
		rx_index++;
	} else if (rx_index == 4) {
		TA2R = 0;
		if (r_frame->type == short_frame) {
			r_frame->stop = data;
			assert(data == MBUS_FRAME_STOP);
			c_status = waiting_for_response;
			/*!--TODO: process short frame*/
		} else {
//			assert(data == MBUS_FRAME_CONTROL_OR_LONG_START);
			r_frame->c_field = data;
			rx_index++;
		}
	} else if (rx_index == 5) {
		TA2R = 0;
		r_frame->a_field = data;
		//assert(data == MBUS_FRAME_CONTROL_OR_LONG_START);
		rx_index++;
	} else if (rx_index == 6) {
		TA2R = 0;
		r_frame->ci_field = data;
		//assert(data == MBUS_FRAME_CONTROL_OR_LONG_START);
		rx_index++;
	} else if (rx_index == 7) {
		TA2R = 0;
		if (r_frame->type == control_frame) {
			r_frame->checksum = data;
		} else {
			ud_buff[0] = data;
		}
		rx_index++;
	} else if (rx_index == 8) {
		TA2R = 0;
		if (r_frame->type == control_frame) {
			r_frame->stop = data;
			assert(data == MBUS_FRAME_STOP);
			c_status = waiting_for_response;
			/*!--TODO: process control frame*/
		} else {
			ud_buff[ud_buff_index++] = data;
			if (rx_index > r_frame->l_fieldL) {
				rx_index++;
			}
		}
	} else if (rx_index == 9) {
		TA2R = 0;
		r_frame->checksum = data;
		rx_index++;
	} else if (rx_index == 10) {
		TA2R = 0;
		TA2CCR0 = 0; // stop timer

		r_frame->user_data = (uint8_t *) ud_buff;
		r_frame->stop = data;
		assert(data == MBUS_FRAME_STOP);
		c_status = waiting_for_response;
		/*!--TODO: process long frame*/
	} else {
		/*slave busy--ignore*/
		return;
	}
	if (c_status == waiting_for_response) {
		/*!---validate frame*/
		if (error == t_validator(r_frame)) {
			x_reset();
			return;
		}
		TA1R = 0;
		TA1CCR0 = 32768; // 32768/8 = 4096 ~ 4KHz
		rx_index = 0;
	}
}

void tx_lnk_frame(uint8_t length) {
	/*!-- All responses will be read from a single global buffer.
	 *  This should return immediately after sending the first character.
	 *  Enables an asynchronous handling of message transfer. */
	c_tx_length = length; //! Set buffer size

	UCA1IE |= UCTXIE; //!-- enable TX interrupt
	UCA1TXBUF = tx_buff[0]; //!-- send first character
	tx_buff_index++;
	c_tx_length -= 1;
	return;
}

void x_lnk_res(void) {
	cb_t = t_handler;
	cb_t();
	return;
}

uint8_t inline getCheksum(volatile lnk_frame *frame) {
	uint8_t cs = 0;
	cs = (frame->c_field + frame->a_field + frame->ci_field);
	int y = frame->l_fieldH - 3;
	for (; y >= 0; y--) {
		cs += *frame->user_data;
		frame->user_data++;
	}
	cs &= 0x0f;
	return cs;
}
cs_result inline t_validator(lnk_frame *frame) {
	uint8_t cs = 0;
	switch (frame->type) {
	case short_frame: {
		cs = (frame->c_field + frame->a_field) & 0x0f;
		break;
	}
	case control_frame: {
		/*!-- TODO
		 * Handle control frame -> ACK or ignore */
		cs = (frame->c_field + frame->a_field + frame->ci_field) & 0x0f;
		break;
	}
	case long_frame: {
		cs = (frame->c_field + frame->a_field + frame->ci_field);
		int y = frame->l_fieldH - 3;
		for (; y >= 0; y--) {
			cs += *frame->user_data;
			frame->user_data++;
		}
		cs &= 0x0f;
		break;
	}
	default:
		break;
	}
	return cs == frame->checksum ? valid : error;
}

#pragma vector = USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void) {
	switch (__even_in_range(UCA1IV, 0x08)) {
	case 0:  // no interrupt
		break;
	case 2:  //!-- RX interrupt
		rx_lnk_frame(UCA1RXBUF);
		break;
	case 4:  //!-- TX interrupt
		if (c_tx_length >= 1) {
			UCA1TXBUF = tx_buff[tx_buff_index];
			tx_buff_index++;
			c_tx_length--;
		} else {
			tx_buff_index = 0;
			c_tx_length = 0;
			UCA1IE &= ~UCTXIE;
		}
		break;
	default:
		break;
	}
	LPM4_EXIT;
}

/*!
 * @brief Function Name: res_delay
 *
 * Introduces a delay between sucessive telegrams.
 * Precise time values can be calculated using the current baud rate.
 *
 * @param none
 *
 * @return none
 */

void inline res_delay() {
	TA1CTL = TACLR;
	TA1CTL = TASSEL__ACLK + MC_1 + ID_2; // ACLK as the CLK source and timer in up mode
	TA1CCR0 = 0; // stop timer
	//TA1CCR0 = 32768; // 32768/4 = 8192 ~ 8KHz
	TA1CCTL0 = CCIE;
	TA1CCTL0 &= ~CCIFG;
}

/*!
 * @brief Function Name: inter_byte_time_out
 *
 * Counts a maximum of the allowed time bit times in milliseconds after reception of a byte.
 * Accordingly, if a byte is received before the timer expires, it is reset again. Otherwise,
 * the timer will expire and any previously buffered bytes will be discarded.
 *
 * @param none
 *
 * @return none
 */

void inline inter_byte_time_out() {
	TA2CTL = TACLR;
	TA2CTL = TASSEL__ACLK + MC_1 + ID_2; // ACLK as the CLK source and timer in up mode
	TA2CCR0 = 0; // stop timer
	//TA2CCR0 = 32768; // 32768/4 = 8192 ~ 8KHz
	TA2CCTL0 = CCIE;
	TA2CCTL0 &= ~CCIFG;
}

/*!
 * @brief Function Name: autospeed_detect
 *
 * Counts a maximum of the recommended time delay(2-10 mins) after reception of a set baudrate command
 * waiting for a master communication at the new baudrate. If no communication is detected,
 * the slave falls back to its previous baudrate setting.
 *
 *
 * @param none
 *
 * @return none
 */

void inline autospeed_detect() {
	TA3CTL = TACLR;
	TA3CTL = TASSEL__ACLK + MC_1 + ID_3; // ACLK as the CLK source and timer in up mode
	TA3CCR0 = 0; // stop timer
	//TA3CCR0 = 32768; // 32768/8 = 4096 ~ 4KHz
	TA3CCTL0 = CCIE;
	TA3CCTL0 &= ~CCIFG;
}

#pragma vector = TIMER1_A1_VECTOR
__interrupt void TA1_ISR(void) {
	uint8_t x_gap = MIN_REQ_RES_DELAY(current_baud);
	r_delay++;
	if (r_delay == x_gap) {
		c_status = response_ready;
		r_delay = 0;
	}
}

#pragma vector = TIMER2_A1_VECTOR
__interrupt void TA2_ISR(void) {
	x_reset();
}

#pragma vector = TIMER3_A1_VECTOR
__interrupt void TA3_ISR(void) {
	auto_br_delay++;
	if (auto_br_delay == (uint16_t) AUTO_BAUD_RESET_DELAY) {
		auto_br_delay = 0;
	}
}
