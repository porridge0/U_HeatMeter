/*!
 * @file spi.h
 *
 * This file contains function implementations to interface the MS1022 Time to Digital converter using serial SPI communication.
 *
 * @note Implementation details referenced from TI's device specific datasheet.
 *
 * @Author Deksios Bekele
 * @date Oct 3, 2020
 *
 */
#ifndef CONFIG_SPI_H_
#define CONFIG_SPI_H_

#include <stdint.h>
#include <msp430.h>
#include <assert.h>
#include "../config/pin_io.h"
#include "../config/system.h"

/*! SPI_TDC_EN: Macro to pull SPI Chip select pin LOW, Clear previous interrupts and enable interrupts....*/
#define SPI_TDC_EN   { \
							(CNFG_TDC_SPI_UCxIFG &= ~(UCTXIFG + UCRXIFG)); \
							(CNFG_TDC_SPI_UCxIE  |=  (UCTXIE + UCRXIE));  \
							(CNFG_PORT_1_OUT    &= ~CNFG_TDC_SLVSELCT_BIT); \
						 }
/*! SPI_TDC_DIS: Macro to pull SPI Chip select pin Hi, Clear previous interrupts and disable interrupts....*/
#define SPI_TDC_DIS  { \
						    (CNFG_TDC_SPI_UCxIFG &= ~(UCTXIFG + UCRXIFG)); \
							(CNFG_TDC_SPI_UCxIE  &= ~(UCTXIE + UCRXIE));  \
							(CNFG_PORT_1_OUT    |= CNFG_TDC_SLVSELCT_BIT); \
						}
const extern boolean SPI_CONTINUE;
/* Configure SPI registers for operation */
void spi_config();
uint8_t spi_transfer(uint8_t byte, boolean end);
uint8_t spi_transfer_end(uint8_t byte);
#endif /* CONFIG_SPI_H_ */
