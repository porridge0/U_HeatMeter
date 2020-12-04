/*!
 * @file pin_io.h
 *
 *	File contains system wide GPIO configurations and macros
 *
 *  @date: Oct 15, 2020
 *
 *  @author: User
 */

#ifndef CONFIG_PIN_IO_H_
#define CONFIG_PIN_IO_H_

#include <msp430.h>

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

/**@name MCU/Board Configuration: TDC_GP22 EN signal control output port and pin configuration.
 * In case of board configuration change software can be reconfigured here without affecting the rest of the code*/
/**@{*/
#define CNFG_PORT_1_DIR  			(P1DIR)	  	/* Port direction register */
#define CNFG_PORT_1_OUT  			(P1OUT)    /* Port data out register */
#define CNFG_PORT_1_IN   			(P1IN)  	/*!<Port data in register */
#define	CNFG_PORT_1_IES      		(P1IES)
#define	CNFG_PORT_1_IE       		(P1IE)
#define CNFG_PORT_1_IFG  			(P1IFG) 	/*!<Port Interrrupt flag register */
#define CNFG_PUSH_BTN_INT_BIT      	(BIT0)
#define CNFG_MBUS_POWER_FAIL_BIT  	(BIT1)     /* Pin bit mask */
#define CNFG_TDC_INTN_BIT  			(BIT2)     /* Pin bit mask */
#define CNFG_REED_SWTCH_INT_BIT     (BIT3)
#define CNFG_TDC_SLVSELCT_BIT  		(BIT5)     /* Pin bit mask */

/** @name MCU/Board Configuration: Port 9 */
/**@{*/
#define CNFG_PORT_9_DIR  			(P9DIR) /*!<Port direction register */
#define CNFG_PORT_9_OUT  			(P9OUT) /*!<Port data out register */
#define CNFG_PORT_9_IFG  			(P9IFG) /*!<Port Interrrupt flag register */
#define CNFG_PORT_9_IE   			(P9IE)  /*!<Port Interrupt enable register */
#define CNFG_PORT_9_IES  			(P9IES) /*!<Port Interrrupt edge setting register */
#define CNFG_PORT_9_IN   			(P9IN)  /*!<Port data in register */
#define CNFG_TDC_START_BIT  		(BIT4)  /*!<Pin bit mask*/
#define CNFG_TDC_RSTN_BIT  			(BIT5)  /*!<Pin bit mask*/
#define CNFG_TDC_EN_START_BIT   	(BIT6)  /*!<Pin bit mask*/
#define CNFG_TDC_FIRE_IN_BIT  		(BIT7)  /*!<Pin bit mask*/
/**@}*/

/** @name MCU/Board Configuration: TDC_GP22 SPI port register selection */
/**@{*/
#define CNFG_TDC_SPI_UCxCTL1   		(UCB0CTL1)   /*  SPI peripheral control register 1 */
#define CNFG_TDC_SPI_UCxCTL0   		(UCB0CTL0)   /*  SPI Peripheral control register 0 */
#define CNFG_TDC_SPI_UCxIE     		(UCB0IE)     /*  SPI Peipheral Interupt flag register */
#define CNFG_TDC_SPI_UCxIFG    		(UCB0IFG)    /*  SPI Peipheral Interupt flag register */
#define CNFG_TDC_SPI_UCxTXBUF  		(UCB0TXBUF)  /*  SPI Transmit buffer */
#define CNFG_TDC_SPI_UCxRXBUF  		(UCB0RXBUF)  /*  SPI Transmit buffer */
#define CNFG_TDC_SPI_UCxSTATW  		(UCB0STATW)  /*  SPI Status Word */
#define CNFG_TDC_SPI_UCxBRW    		(UCB0BRW)    /*  SPI Baud Rate Register */

#define CNFG_TDC_SPI_INT_VECTOR 	(USCI_B0_VECTOR) /* SPI interrupt vector */
#define CNFG_TDC_SPI_INT_IV_REG 	(UCB0IV)         /* SPI interrupt vector flag register */

#define CNFG_TDC_MOSI_BIT          (BIT6)       /*  SPI MOSI Pin bit mask */
#define CNFG_TDC_MOSI_PxSEL1       (P1SEL1)     /*  SPI MOSI Pin I/O function selection register 1 */
#define CNFG_TDC_MOSI_PxSEL0       (P1SEL0)     /*  SPI MOSI Pin I/O function selection register 0 */
#define CNFG_TDC_MOSI_PxSEL1_VAL    0                       /* PxSEL1 register bit value that would enable SPI function */
#define CNFG_TDC_MOSI_PxSEL0_VAL   (CNFG_TDC_MOSI_BIT)   /* PxSEL0 register bit value that would enable SPI function */

#define CNFG_TDC_MISO_BIT          (BIT7)       /*  SPI MISO Pin bit mask */
#define CNFG_TDC_MISO_PxSEL1       (P1SEL1)     /*  SPI MISO Pin I/O function selection register 1 */
#define CNFG_TDC_MISO_PxSEL0       (P1SEL0)     /*  SPI MISO Pin I/O function selection register 0 */
#define CNFG_TDC_MISO_PxSEL1_VAL    0          /*  PxSEL1 register bit value that would enable SPI function */
#define CNFG_TDC_MISO_PxSEL0_VAL   (CNFG_TDC_MISO_BIT)   /* PxSEL0 register bit value that would enable SPI function */

#define CNFG_TDC_SCLK_BIT          (BIT4)       /*  SPI MOSI Pin bit mask */
#define CNFG_TDC_SCLK_PxSEL1       (P1SEL1)     /*  SPI SCLK Pin I/O function selection register 1 */
#define CNFG_TDC_SCLK_PxSEL0       (P1SEL0)     /*  SPI SCLK Pin I/O function selection register 0 */
#define CNFG_TDC_SCLK_PxSEL1_VAL    0   					/* PxSEL1 register bit value that would enable SPI function */
#define CNFG_TDC_SCLK_PxSEL0_VAL    (CNFG_TDC_SCLK_BIT)     /* PxSEL0 register bit value that would enable SPI function */
/**@}*/

/**@name MCU/Board Configuration: Optical Port register selection */
/**@{*/
#define MBUS_RX_TX_DIR         	(P4DIR)           /* RF mod. comm. TX pin direction register */
#define MBUS_RX_TX_OUT         	(P4OUT)            /* RF mod. comm. TX pin output register */

#define MBUS_TX_BIT            	(BIT2)            /* Optical port TX Pin bit mask */
#define MBUS_TX_PxSEL1         	(P4SEL1)          /* Optical port TX Pin I/O function selection register 1 */
#define MBUS_TX_PxSEL0         	(P4SEL0)          /* Optical port TX Pin I/O function selection register 0 */
#define MBUS_TX_PxSEL1_VAL     	(0)               /* PxSEL1 register bit value that would enable UART function */
#define MBUS_TX_PxSEL0_VAL     	(MBUS_TX_BIT) 		/* PxSEL0 register bit value that would eable UART function */

#define MBUS_RX_BIT            	(BIT3)            /* Optical port TX Pin bit mask */
#define MBUS_RX_PxSEL1         	(P4SEL1)          /* Optical port TX Pin I/O function selection register 1 */
#define MBUS_RX_PxSEL0         	(P4SEL0)          /* Optical port TX Pin I/O function selection register 0 */
#define MBUS_RX_PxSEL1_VAL     	(0)               /* PxSEL1 register bit value that would enable UART function */
#define MBUS_RX_PxSEL0_VAL     	(MBUS_RX_BIT) 		/* PxSEL0 register bit value that would eable UART function */

#define MBUS_UART_UCxCTL1    	(UCA0CTL1)        /*  UART peripheral control register 1 */
#define MBUS_UART_UCxCTL0     	(UCA0CTL0)        /*  UART Peripheral control register 0 */
#define MBUS_UART_UCxIE       	(UCA0IE)          /*  UART Peipheral Interupt flag register */
#define MBUS_UART_UCxIFG      	(UCA0IFG)         /*  UART Peipheral Interupt flag register */
#define MBUS_UART_UCxTXBUF    	(UCA0TXBUF)       /*  UART Transmit buffer */
#define MBUS_UART_UCxRXBUF    	(UCA0RXBUF)       /*  UART Transmit buffer */
#define MBUS_UART_UCxSTATW    	(UCA0STATW0)      /*  UART Status Word */
#define MBUS_UART_INT_VECTOR  	(USCI_A0_VECTOR)   /*  USCI Interrupt vector */
#define MBUS_UART_UCxBRW      	(UCA0BRW)         /* USCI Baud Rate Register */
#define MBUS_UART_UCxMCTLW    	(UCA0MCTLW)
#define MBUS_UART_UCxIV       	(UCA0IV)          /*  Interrupt vector register */

/**@}*/

/**@name MCU/Board Configuration: SMCLK (System Main Clock) Frequency configuration*/
/**@{*/
#define SM606_CNFG_SMCLK_FREQ 8   /*! SMCLK (System main clock) Freqency in MHz. Possible values are:
									   *  \li : 1 => 1MHz frequency
									   *  \li : 4 => 4MHz frequency
									   *  \li : 8 => 8MHz frequency
									   */
/**@}*/

#endif /* CONFIG_PIN_IO_H_ */
