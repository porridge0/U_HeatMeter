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

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

/**@name MCU/Board Configuration: TDC_GP22 EN signal control output port and pin configuration.
 * In case of board configuration change software can be reconfigured here without affecting the rest of the code*/
/**@{*/
#define CNFG_TDC_EN_DIR  (P1DIR)	  /* Port direction register */
#define CNFG_TDC_EN_OUT  (P1OUT)    /* Port data out register */
#define CNFG_TDC_RESET_BIT  (BIT1)     /* Pin bit mask */
#define CNFG_TDC_SLVSELCT_BIT  (BIT5)     /* Pin bit mask */
/**@}*/

/** @name MCU/Board Configuration: TDC_GP22 START input pin. */
/**@{*/
#define CNFG_TDC_STRT  (P9DIR) /*!<Port direction register */
#define CNFG_TDC_START  (P9OUT) /*!<Port data out register */
#define CNFG_TDC_START  (BIT4)  /*!<Pin bit mask*/
/**@}*/

/** @name MCU/Board Configuration: TDC_GP22 INTN Interrupt flag pin */
/**@{*/
#define CNFG_TDC_INTN_DIR  (P9DIR) /*!<Port direction register */
#define CNFG_TDC_INTN_IFG  (P9IFG) /*!<Port Interrrupt flag register */
#define CNFG_TDC_INTN_IE   (P9IE)  /*!<Port Interrupt enable register */
#define CNFG_TDC_INTN_IES  (P9IES) /*!<Port Interrrupt edge setting register */
#define CNFG_TDC_INTN_IN   (P9IN)  /*!<Port data in register */
#define CNFG_TDC_INTN_BIT  (BIT6)  /*!<Pin bit mask*/
/**@}*/

/** @name MCU/Board Configuration: TDC_GP22 SPI port register selection */
/**@{*/
#define CNFG_TDC_SPI_UCxCTL1   (UCB0CTL1)   /*  SPI peripheral control register 1 */
#define CNFG_TDC_SPI_UCxCTL0   (UCB0CTL0)   /*  SPI Peripheral control register 0 */
#define CNFG_TDC_SPI_UCxIE     (UCB0IE)     /*  SPI Peipheral Interupt flag register */
#define CNFG_TDC_SPI_UCxIFG    (UCB0IFG)    /*  SPI Peipheral Interupt flag register */
#define CNFG_TDC_SPI_UCxTXBUF  (UCB0TXBUF)  /*  SPI Transmit buffer */
#define CNFG_TDC_SPI_UCxRXBUF  (UCB0RXBUF)  /*  SPI Transmit buffer */
#define CNFG_TDC_SPI_UCxSTATW  (UCB0STATW)  /*  SPI Status Word */
#define CNFG_TDC_SPI_UCxBRW    (UCB0BRW)    /*  SPI Baud Rate Register */

#define CNFG_TDC_SPI_INT_VECTOR (USCI_B0_VECTOR) /* SPI interrupt vector */
#define CNFG_TDC_SPI_INT_IV_REG (UCB0IV)         /* SPI interrupt vector flag register */

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

/**@}*/

/**@name MCU/Board Configuration:  external channel select pin configuration.
 * In case of board configuration change software can be reconfigured here with out affecting the rest of the code*/
/**@{*/
#define CNFG_TDC_EXT_CH_SEL_DIR  (PJDIR)   /* Port direction register */
#define CNFG_TDC_EXT_CH_SEL_OUT  (PJOUT)   /* Port data out register */
#define CNFG_TDC_EXT_CH_SEL_BIT  (BIT6)    /* Pin bit mask */
/**@}*/

/**@name MCU/Board Configuration:  RESET signal control output port and pin configuration.*/
/**@{*/
#define CNFG_TDC_RST_DIR (P6DIR)  /* Port direction register */
#define CNFG_TDC_RST_OUT (P6OUT)  /* Port data out register */
#define CNFG_TDC_RST_BIT (BIT0)   /* Pin bit mask*/
/**@}*/

/**@name MCU/Board Configuration:  ERROR pin control output port and pin configuration.*/
/**@{*/
#define CNFG_TDC_ERR_DIR (P1DIR) /* Port direction register */
#define CNFG_TDC_ERR_IN  (P1IN)  /* Port data in register */
#define CNFG_TDC_ERR_BIT (BIT3)  /* Pin bit mask */
#define CNFG_TDC_ERR_IFG (P1IFG) /* Interrupt flag register */
#define CNFG_TDC_ERR_IE  (P1IE)  /* Interrupt enable register */
#define CNFG_TDC_ERR_IES  (P1IES)/* Interrupt configuration register */
/**@}*/


/**@name MCU/Board Configuration: TDC_GP22 Clock source EN signal control output port and pin configuration.*/
/**@{*/
#define CNFG_TDC_CLK_EN_DIR  (P6DIR)	  /* Port direction register */
#define CNFG_TDC_CLK_EN_OUT  (P6OUT)    /* Port data out register */
#define CNFG_TDC_CLK_EN_BIT  (BIT2)     /* Pin bit mask */
/**@}*/

/**@name MCU/Board Configuration: Optical Port register selection */
/**@{*/
#define MBUS_TX_BIT            (BIT2)            /* Optical port TX Pin bit mask */
#define MBUS_TX_PxSEL1         (P4SEL1)          /* Optical port TX Pin I/O function selection register 1 */
#define MBUS_TX_PxSEL0         (P4SEL0)          /* Optical port TX Pin I/O function selection register 0 */
#define MBUS_TX_PxSEL1_VAL     (0)               /* PxSEL1 register bit value that would enable UART function */
#define MBUS_TX_PxSEL0_VAL     (MBUS_TX_BIT) /* PxSEL0 register bit value that would eable UART function */

#define MBUS_RX_BIT            (BIT3)            /* Optical port TX Pin bit mask */
#define MBUS_RX_PxSEL1         (P4SEL1)          /* Optical port TX Pin I/O function selection register 1 */
#define MBUS_RX_PxSEL0         (P4SEL0)          /* Optical port TX Pin I/O function selection register 0 */
#define MBUS_RX_PxSEL1_VAL     (0)               /* PxSEL1 register bit value that would enable UART function */
#define MBUS_RX_PxSEL0_VAL     (MBUS_RX_BIT) /* PxSEL0 register bit value that would eable UART function */

#define MBUS_UART_UCxCTL1    	(UCA1CTL1)        /*  UART peripheral control register 1 */
#define MBUS_UART_UCxCTL0     	(UCA1CTL0)        /*  UART Peripheral control register 0 */
#define MBUS_UART_UCxIE       	(UCA1IE)          /*  UART Peipheral Interupt flag register */
#define MBUS_UART_UCxIFG      	(UCA1IFG)         /*  UART Peipheral Interupt flag register */
#define MBUS_UART_UCxTXBUF    	(UCA1TXBUF)       /*  UART Transmit buffer */
#define MBUS_UART_UCxRXBUF    	(UCA1RXBUF)       /*  UART Transmit buffer */
#define MBUS_UART_UCxSTATW    	(UCA1STATW0)      /*  UART Status Word */
#define MBUS_UART_INT_VECTOR  	 USCI_A1_VECTOR   /*  USCI Interrupt vector */
#define MBUS_UART_UCxBRW      	(UCA1BRW)         /* USCI Baud Rate Register */
#define MBUS_UART_UCxMCTLW    	(UCA1MCTLW)
#define MBUS_UART_UCxIV       	(UCA1IV)          /*  Interrupt vector register */

/**@}*/

/**@name MCU/Board Configuration: RF Module Port and Register selection */
/**@{*/

#define CNFG_RF_TX_DIR               (P4DIR)           /* RF mod. comm. TX pin direction register */
#define CNFG_RF_TX_OUT                P4OUT            /* RF mod. comm. TX pin output register */
#define CNFG_RF_TX_BIT               (BIT2)            /* RF mod. comm. TX pin bit mask */
#define CNFG_RF_TX_PxSEL1            (P4SEL1)          /* RF mod. comm. TX pin I/O function selection register 1 */
#define CNFG_RF_TX_PxSEL0            (P4SEL0)          /* RF mod. comm. TX pin I/O function selection register 0 */
#define CNFG_RF_TX_PxSEL1_VAL        (0)               /* PxSEL1 register bit value that would enable UART function */
#define CNFG_RF_TX_PxSEL0_VAL        (CNFG_RF_TX_BIT)  /* PxSEL0 register bit value that would enable UART function */

#define CNFG_RF_RX_BIT               (BIT3)            /* RF mod. comm. RX pin bit mask */
#define CNFG_RF_RX_PxSEL1            (P4SEL1)          /* RF mod. comm. RX pin I/O function selection register 1 */
#define CNFG_RF_RX_PxSEL0            (P4SEL0)          /* RF mod. comm. RX pin I/O function selection register 0 */
#define CNFG_RF_RX_PxSEL1_VAL        (0)               /* PxSEL1 register bit value that would enable UART function */
#define CNFG_RF_RX_PxSEL0_VAL        (CNFG_RF_RX_BIT)  /* PxSEL0 register bit value that would enable UART function */

#define CNFG_RF_RESET_BIT            (BIT2)            /* RF mod. comm. RESET pin bit mask */
#define CNFG_RF_RESET_OUT            (P9OUT)           /* RF mod. comm. RESET pin gpio port output register */
#define CNFG_RF_RESET_DIR            (P9DIR)           /* RF mod. comm. RESET pin gpio port direction register */
#define CNFG_RF_RESET_PxSEL1         (P9SEL1)          /* RF mod. comm. RESET pin I/O function selection register 1 */
#define CNFG_RF_RESET_PxSEL0         (P9SEL0)          /* RF mod. comm. RESET pin I/O function selection register 0 */
#define CNFG_RF_RESET_PxSEL1_VAL     (0)               /* PxSEL1 register bit value that would enable UART function */
#define CNFG_RF_RESET_PxSEL0_VAL     (0) 			   /* PxSEL0 register bit value that would enable UART function */

#define CNFG_RF_RTS_BIT              (BIT7)             /* RF mod. comm. RTS pin bit mask */
#define CNFG_RF_RTS_OUT              (PJOUT)            /* RF mod. comm. RTS pin gpio port output register */
#define CNFG_RF_RTS_DIR              (PJDIR)            /* RF mode. comm. RTS pin gpio port direction register */
#define CNFG_RF_RTS_PxSEL1           (PJSEL1)           /* RF mod. comm. RTS pin I/O function selection register 1 */
#define CNFG_RF_RTS_PxSEL0           (PJSEL0)           /* RF mod. comm. RTS pin I/O function selection register 0 */
#define CNFG_RF_RTS_PxSEL1_VAL       (0)                /* PxSEL1 register bit value that would enable GPIO function */
#define CNFG_RF_RTS_PxSEL0_VAL       (0)                /* PxSEL0 register bit value that would enable GPIO function */

#define CNFG_RF_CTS_BIT              (BIT0)             /* RF mod. comm. CTS pin bit mask */
#define CNFG_RF_CTS_OUT              (P1OUT)            /* RF mod. comm. CTS pin gpio port output register */
#define CNFG_RF_CTS_DIR              (P1DIR)            /* RF mod. comm. CTS pin gpio port direction register */
#define CNFG_RF_CTS_PxSEL1           (P1SEL1)           /* RF mod. comm. CTS pin I/O function selection register 1 */
#define CNFG_RF_CTS_PxSEL0           (P1SEL0)           /* RF mod. comm. CTS pin I/O function selection register 0 */
#define CNFG_RF_CTS_PxSEL1_VAL       (0)                /* PxSEL1 register bit value that would enable GPIO function */
#define CNFG_RF_CTS_PxSEL0_VAL       (0)                /* PxSEL0 register bit value that would enable GPIO function */

#define CNFG_RF_UART_UCxCTL1    	(UCA0CTL1)        /*  UART peripheral control register 1 */
#define CNFG_RF_UART_UCxCTL0     	(UCA0CTL0)        /*  UART Peripheral control register 0 */
#define CNFG_RF_UART_UCxIE       	(UCA0IE)          /*  UART Peipheral Interupt flag register */
#define CNFG_RF_UART_UCxIFG      	(UCA0IFG)         /*  UART Peipheral Interupt flag register */
#define CNFG_RF_UART_UCxTXBUF    	(UCA0TXBUF)       /*  UART Transmit buffer */
#define CNFG_RF_UART_UCxRXBUF    	(UCA0RXBUF)       /*  UART Transmit buffer */
#define CNFG_RF_UART_UCxSTATW    	(UCA0STATW0)      /*  UART Status Word */
#define CNFG_RF_UART_INT_VECTOR  	 USCI_A0_VECTOR   /*  USCI Interrupt vector */
#define CNFG_RF_UART_UCxBRW      	(UCA0BRW)         /* USCI Baud Rate Register */
#define CNFG_RF_UART_UCxMCTLW    	(UCA0MCTLW)
#define CNFG_RF_UART_UCxIV       	(UCA0IV)          /*  Interrupt vector register */

/**@}/	 */

/**@name MCU/Board Configuration: EEPROM interface Port and Register selection */
/**@{*/
#define CNFG_EEPROM_PWR_CTRL_DIR          (P9DIR)
#define CNFG_EEPROM_PWR_CTRL_OUT          (P9OUT)
#define CNFG_EEPROM_PWR_CTRL_BIT          (BIT7)

#define	CNFG_EEPROM_I2C_UCxCTLW0   (UCB1CTLW0)      /* I2C control register word 0 */
#define	CNFG_EEPROM_I2C_UCxCTLW1   (UCB1CTLW1)      /* I2C control register word 1 */
#define	CNFG_EEPROM_I2C_UCxBRW     (UCB1BRW)        /* I2C baud rate control register word */
#define	CNFG_EEPROM_I2C_UCxTBCNT   (UCB1TBCNT)      /* Number of I2C data bytes after which the automatic STOP or the UCSTPIFG should occur */
#define CNFG_EEPROM_I2C_UCxI2CSA   (UCB1I2CSA)      /* Slave address register */

#define CNFG_EEPROM_I2C_UCxTXBUF  (UCB1TXBUF)      /* Transmit buffer */
#define CNFG_EEPROM_I2C_UCxRXBUF  (UCB1RXBUF)       /* Recieve buffer */
#define CNFG_EEPROM_I2C_UCxIFG    (UCB1IFG)        /* Interrupt flag register */

#define CNFG_EEPROM_I2C_UCxIE     (UCB1IE)          /* Interrupt enable register */

/* Select I2C functionality for SDA and SCL pins */
#define	CNFG_EEPROM_I2C_SDA_BIT          (BIT1)
#define	CNFG_EEPROM_I2C_SDA_PxSEL1       (P3SEL1)
#define	CNFG_EEPROM_I2C_SDA_PxSEL0       (P3SEL0)
#define	CNFG_EEPROM_I2C_SDA_PxSEL1_VAL   (0)
#define	CNFG_EEPROM_I2C_SDA_PxSEL0_VAL   (CNFG_EEPROM_I2C_SDA_BIT)

#define CNFG_EEPROM_I2C_SCL_BIT          (BIT2)
#define	CNFG_EEPROM_I2C_SCL_PxSEL1       (P3SEL1)
#define	CNFG_EEPROM_I2C_SCL_PxSEL0       (P3SEL0)
#define	CNFG_EEPROM_I2C_SCL_PxSEL1_VAL   (0)
#define	CNFG_EEPROM_I2C_SCL_PxSEL0_VAL   (CNFG_EEPROM_I2C_SCL_BIT)

/**@}/	 */

/**@name MCU/Board Configuration:   TPS62740 voltage regulator control pin definitions */
/**@{*/
#define CNFG_TPS62740_VSEL3_DIR        (P9DIR)
#define CNFG_TPS62740_VSEL3_OUT        (P9OUT)
#define CNFG_TPS62740_VSEL3_BIT        (BIT3)

#define CNFG_TPS62740_VSEL4_DIR        (P9DIR)
#define CNFG_TPS62740_VSEL4_OUT        (P9OUT)
#define CNFG_TPS62740_VSEL4_BIT        (BIT6)
/**@}/	 */

/**@}/	 */

/**@name MCU/Board Configuration:   Battery voltage Measurement pin defintions */
/**@{*/
#define CNFG_BAT_MEAS_EN_PIN_DIR     (P9DIR)
#define CNFG_BAT_MEAS_EN_PIN_OUT     (P9OUT)
#define CNFG_BAT_MEAS_EN_PIN_BIT     (BIT0)

#define CNFG_BAT_MEAS_ADC_PIN_DIR    (P9DIR)
#define CNFG_BAT_MEAS_ADC_PIN_SEL0   (P9SEL0)
#define CNFG_BAT_MEAS_ADC_PIN_SEL1   (P9SEL1)
#define CNFG_BAT_MEAS_ADC_PIN_BIT    (BIT1)

#define CNFG_TPS62740_VSEL3_DIR        (P9DIR)
#define CNFG_TPS62740_VSEL3_OUT        (P9OUT)
#define CNFG_TPS62740_VSEL3_BIT        (BIT3)

#define CNFG_TPS62740_VSEL4_DIR        (P9DIR)
#define CNFG_TPS62740_VSEL4_OUT        (P9OUT)
#define CNFG_TPS62740_VSEL4_BIT        (BIT6)
/**@}/	 */

/**@name MCU/Board Configuration: SMCLK (System Main Clock) Frequency configuration*/
/**@{*/
#define SM606_CNFG_SMCLK_FREQ 8   /*! SMCLK (System main clock) Freqency in MHz. Possible values are:
									   *  \li : 1 => 1MHz frequency
									   *  \li : 4 => 4MHz frequency
									   *  \li : 8 => 8MHz frequency
									   */
/**@}*/

/**@name MCU/Board Configuration: cover-removed switch I/O configuration*/
/**@{*/
#define CNFG_COVER_REMOVED_INT_DIR      (P2DIR)
#define	CNFG_COVER_REMOVED_INT_IES      (P2IES)
#define	CNFG_COVER_REMOVED_INT_IE       (P2IE)
#define CNFG_COVER_REMOVED_INT_BIT      (BIT4)

#endif /* CONFIG_PIN_IO_H_ */
