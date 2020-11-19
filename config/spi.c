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

	CNFG_TDC_EN_DIR |= CNFG_TDC_EN_BIT; /* Select output direction for ENABLE pin control output */
	CNFG_TDC_EN_OUT &= ~CNFG_TDC_EN_BIT; /* Disable chip by default on startup */

	/* Initialize SPI peripheral */
	CNFG_TDC_CSB_DIR |= CNFG_TDC_SLVSELCT_BIT; /* Select output direction for SPI chip slect pin*/
	SPI_TDC_DIS; /* Disable TDC_GP22 SPI by default */

	CNFG_TDC_EN_DIR |= CNFG_TDC_EN_BIT; /* Select output direction for TDC_GP22 ChipEnable signal */
	CNFG_TDC_EN_OUT &= ~CNFG_TDC_EN_BIT; /* Disable TDC_GP22 by default. Chip in Sleep Mode */

	CNFG_TDC_INTN_DIR &= ~CNFG_TDC_INTN_BIT; /* Select input direction for TDC_GP22 INTN signal input */
	CNFG_TDC_INTN_IES |= CNFG_TDC_INTN_BIT; /* Select falling edge interrupt for TDC_GP22 INTN signal */

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

	CNFG_TDC_SPI_UCxCTL0 = (BIT7/*UCCKPH*/+ BIT5/*UCMSB*/+ UCMODE_2/* UCxSTE active low*/
	+ BIT3/*UCMST*/
	+ BIT0/*UCSYNC*/); /* 4-wire, 8-bit SPI master, MSB first */
	;

	/*!TODO Verify clock source and baud rate selection */
	CNFG_TDC_SPI_UCxCTL1 |= UCSSEL_3; /* SMCLK as clock source */
	CNFG_TDC_SPI_UCxBRW = 0x00; /* baud rate = SMCLK/1 */

	CNFG_TDC_SPI_UCxCTL1 &= ~UCSWRST; /* Configuration complete; restart Peripheral. Initialize USCI state machine */

}

