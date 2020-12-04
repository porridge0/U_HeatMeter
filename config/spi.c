/*!
 * @file spi.h
 *
 *	MSP430FR6820 device specific SPI module configuration.
 *
 *  @date: Oct 3, 2020
 *
 *  @author: Deksios Bekele
 */
#include <msp430.h>
#include "spi.h"
#include "../config/pin_io.h"

void spi_config() {

	/* Initialize SPI peripheral */
	CNFG_PORT_1_DIR |= CNFG_TDC_SLVSELCT_BIT; /* Select output direction for slave select pin*/

	SPI_TDC_DIS; /* Disable TDC_GP22 SPI by default */

	CNFG_PORT_1_DIR &= ~CNFG_TDC_INTN_BIT; /* Select input direction for TDC_GP22 INTN signal input */
	CNFG_PORT_1_IES |= CNFG_TDC_INTN_BIT; /* Select falling edge interrupt for TDC_GP22 INTN signal */

	CNFG_TDC_SPI_UCxCTL1 |= UCSWRST; /* Put SPI into reset mode before doing configuration*/

	/* Select SPI functionality for MOSI,MISO and SCLK pins */
	CNFG_TDC_MOSI_PxSEL0 &= ~CNFG_TDC_MOSI_BIT; /* Reset bit first.....*/
	CNFG_TDC_MOSI_PxSEL0 |= CNFG_TDC_MOSI_PxSEL0_VAL; /* ....then "OR' desired bit value*/
	CNFG_TDC_MOSI_PxSEL1 &= ~CNFG_TDC_MOSI_BIT;
	CNFG_TDC_MOSI_PxSEL1 |= CNFG_TDC_MOSI_PxSEL1_VAL;

	CNFG_TDC_MISO_PxSEL0 &= ~CNFG_TDC_MISO_BIT; /* Reset bit first........*/
	CNFG_TDC_MISO_PxSEL0 |= CNFG_TDC_MISO_PxSEL0_VAL; /* ...... the 'OR' desired bit value */
	CNFG_TDC_MISO_PxSEL1 &= ~CNFG_TDC_MISO_BIT;
	CNFG_TDC_MISO_PxSEL1 |= CNFG_TDC_MISO_PxSEL1_VAL;

	CNFG_TDC_SCLK_PxSEL0 &= ~CNFG_TDC_SCLK_BIT; /* Reset bit first........*/
	CNFG_TDC_SCLK_PxSEL0 |= CNFG_TDC_SCLK_PxSEL0_VAL; /* ...... the 'OR' desired bit value */
	CNFG_TDC_SCLK_PxSEL1 &= ~CNFG_TDC_SCLK_BIT;
	CNFG_TDC_SCLK_PxSEL1 |= CNFG_TDC_SCLK_PxSEL1_VAL;

	CNFG_TDC_SPI_UCxCTL0 = (BIT7/*UCCKPH*/+ BIT5/*UCMSB*/+ BIT3/*UCMST*/+ BIT2/* UCxSTE active low*/+ BIT0/*UCSYNC*/); /* 4-wire, 8-bit SPI master, MSB first */

	/*!TODO Verify clock source and baud rate selection */
	CNFG_TDC_SPI_UCxCTL1 |= UCSSEL_3; /* SMCLK as clock source */
	CNFG_TDC_SPI_UCxBRW = 0x00; /* baud rate = SMCLK/1 */

	CNFG_TDC_SPI_UCxCTL1 &= ~UCSWRST; /* Configuration complete; restart Peripheral. Initialize USCI state machine */
//	CNFG_TDC_SPI_UCxIE |= UCRXIE; /* Enable RX interrupt */
//	CNFG_TDC_SPI_UCxIFG &= ~(UCTXIFG + UCRXIFG); /* Clear interrupts if set */
}
uint8_t spi_transfer(uint8_t byte, boolean end) {
	volatile uint8_t data;
	SPI_TDC_EN; /* Select device */
	CNFG_TDC_SPI_UCxTXBUF = byte; /* load dummy data */
	LPM0; /* wait in LPM0 until ready */
	data = CNFG_TDC_SPI_UCxRXBUF; /* read data */
	if (FALSE == end)
		SPI_TDC_DIS; /* Deselect device */

	return data; /* return data */
}
uint8_t spi_transfer_end(uint8_t byte) {
	volatile uint8_t data;
	SPI_TDC_EN; /* Select device */
	CNFG_TDC_SPI_UCxTXBUF = byte; /* load dummy data */
	LPM0; /* wait in LPM0 until ready */
	data = CNFG_TDC_SPI_UCxRXBUF; /* read data */
	SPI_TDC_DIS; /* Deselect device */

	return data; /* return data */
}

