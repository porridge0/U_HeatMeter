/**
 * @file lcdx.c
 *
 *  Implementation of functions used to manipulate what is displayed on the UHM LCD.
 *
 * @note Implementation based on Rev 1.0 LCD design
 *
 * @author Deksios Bekele

 * @date Oct 27, 2020
 *
 */
#include <msp430.h>
#include <assert.h>
#include "lcdx.h"

/*********************************************************************************************************************/
/* LCD Main Numeric dispaly Digit Segment-to-Register Mapping.
 * eg. DIGIT_REGS[i] is register controlling segments of the ith digit.
 * Segment numbering is shown below:
 *
 *                 0(A)
 *              |||||||
 *            ||       ||
 *       5(F) ||       ||  1(B)
 *            ||  6(G) ||
 *              |||||||
 *            ||       ||
 *       4(E) ||       ||  2(C)
 *            ||       ||
 *              |||||||
 *                 3(D)
 */
#define NUM_DIGITS 8
volatile uint8_t* const DIGIT_REGS[10] = { &LCDM4, &LCDM5, &LCDM6, &LCDM7,
		&LCDM8, &LCDM9, &LCDM10, &LCDM11, &LCDM12, &LCDM13 };

/*****************************************************************************************************************************/
/* Array holding address of LCD registers used by the program*/
volatile uint8_t* LCD_REGS[13] = { &LCDM3, &LCDM4, &LCDM5, &LCDM6, &LCDM7,
		&LCDM8, &LCDM9, &LCDM10, &LCDM11, &LCDM12, &LCDM13, &LCDM14, &LCDM15 };
/* Array containing 7-bit segment information used to display each 0-9 digit
 * const uint8_t SevenSegs[10] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07,
 0x7F, 0x6F };
 * eg. SevenSegs[0] is 0x3F and it indicates that segments 0,1,2,3,4 and 5 should be on and segment 6 off to write a 0
 *     SevenSegs[1] is 0x06 and it indicates that segments 1 and 2 should be on and the rest off
 */

/*!
 * @brief Function Name: LCD_Setup
 *
 * The function initialized registers and I/O pins associated with LCD operation
 *
 * @param none
 * @return none
 */
void LCD_Setup() {

	/* Enable COMx Pin */

	P6SEL0 |= BIT3 + BIT4 + BIT5 + BIT6;
	P6SEL1 |= BIT3 + BIT4 + BIT5 + BIT6;

	LCDCPCTL0 = 0xFFF4; /* Enable LCD S2-S15 */
	LCDCPCTL1 = 0xE7CF; /* Enable LCD S16-18, S22-S26, S29-S31 */
	LCDCPCTL2 = 0x0000; /* Enable LCD S32-43 */

#ifdef LCD_USE_CHARGE_PUMP
	LCDCVCTL = VLCD_7 + LCDCPEN; /* Use charge pump */
	LCDCCPCTL = LCDCPCLKSYNC; /* Synchronize charge pump with internal clock */
#else
	P6SELC = 0x07; /* select external resistor */
	LCDCVCTL = LCDREXT + R03EXT + LCDEXTBIAS; /* Use resistor */
#endif
	LCDCMEMCTL = LCDCLRM; /* Clear LCD memory */
	LCDCCTL0 = LCDDIV_3 + LCDPRE_5 + LCD4MUX + LCDLP + LCDON; /* 4 MUX, Low power waveform, use ACLK, turn on LCD */
}

/*!
 * @brief Function Name: clear_digits
 *
 * This function blanks the entire number display area
 *
 * @param none
 *
 * @return none
 *
 */
void clear_digits() {
	uint8_t digit;

	for (digit = 0; digit < NUM_DIGITS; digit++) {
		*(DIGIT_REGS[digit]) &= 0x0;
	}

}

/*!
 * @brief Function Name: bin_2_bcd
 *
 * Utility function to be used to convert binary number to 8 digit packed BCD format
 *
 * @param digit: Number in binary format
 *
 * @return 8 digit packed BCD formated number
 *
 */
uint64_t bin_2_bcd(int32_t bin) {
	volatile int32_t value;
	volatile uint64_t bcd = 0x00000000;

	value = bin;
	if (value < 0) {
		value = 0 - value;
	}
	assert(value <= 999999999);
	bcd = 0;
	while (value >= 100000000) /* Extract the 9th Digit */
	{
		value -= 100000000;
		bcd++;
	}
	bcd <<= 4;
	while (value >= 10000000) /* Extract the 8th Digit */
	{
		value -= 10000000;
		bcd++;
	}
	bcd <<= 4;
	while (value >= 1000000) /* Extract the 7th Digit */
	{
		value -= 1000000;
		bcd++;
	}
	bcd <<= 4;
	while (value >= 100000) /* Extract the 6th Digit */
	{
		value -= 100000;
		bcd++;
	}
	bcd <<= 4;
	while (value >= 10000) /* Extract the 5th Digit */
	{
		value -= 10000;
		bcd++;
	}
	bcd <<= 4;
	while (value >= 1000) /* Extract the 4rd Digit */
	{
		value -= 1000;
		bcd++;
	}
	bcd <<= 4;
	while (value >= 100) /* Extract the 3nd Digit */
	{
		value -= 100;
		bcd++;
	}
	bcd <<= 4;
	while (value >= 10) /* Extract the 2nd Digit */
	{
		value -= 10;
		bcd++;
	}
	bcd <<= 4;
	bcd |= value; /* Extract the 1st Digit */

	return bcd;
}

/*!
 * @brief Function Name: all_on
 *
 * The function turns on all symbols on LCD. This is to be used for test purpose
 *
 * @param none
 *
 * @return none
 */
void all_on() {
	uint8_t i;

	for (i = 0; i < sizeof(LCD_REGS); i++) {
		*LCD_REGS[i] = 0xFF;
	}
}
/*!
 * @brief Function Name: all_off
 *
 * The function turns off all symbols on LCD. This is to be used for test purpose
 *
 * @param none
 *
 * @return none
 */
void all_off() {
	uint8_t i;

	for (i = 0; i < sizeof(LCD_REGS); i++) {
		*LCD_REGS[i] = 0x00;
	}
}

void write_minus(uint8_t pos) {
	if (--pos != 5 || --pos != 6) {
		(*(DIGIT_REGS[pos])) |= BIT2;
		return;
	}
	(*(DIGIT_REGS[pos])) |= BIT5;
}

#define TEST_DISPLAY
#ifdef TEST_DISPLAY

/*volatile uint8_t* FIRST_DIGIT_REGS[2] = { &LCDM8, &LCDM9 };
 volatile uint8_t* SECOND_DIGIT_REGS[2] = { &LCDM7, &LCDM8 };
 volatile uint8_t* THIRD_DIGIT_REGS[2] = { &LCDM6, &LCDM7 };
 volatile uint8_t* FOURTH_DIGIT_REGS[2] = { &LCDM5, &LCDM6 };
 volatile uint8_t* FIFTH_DIGIT_REGS[2] = { &LCDM10, &LCDM11 };
 volatile uint8_t* SIXTH_DIGIT_REGS[1] = { &LCDM4 };
 volatile uint8_t* SEVENTH_DIGIT_REGS[1] = { &LCDM12 };
 volatile uint8_t* EIGHTH_DIGIT_REGS[1] = { &LCDM13 };*/

volatile uint8_t* NUM_REGS[NUM_DIGITS][2] = { { &LCDM8, &LCDM9 }, { &LCDM7,
		&LCDM8 }, { &LCDM6, &LCDM7 }, { &LCDM5, &LCDM6 }, { &LCDM10, &LCDM11 },
		{ &LCDM4 }, { &LCDM12 }, { &LCDM13 } };

const uint8_t MASK_H[10] = { 0x7, 0x6, 0xE, 0x7, 0x6, 0x5, 0x5, 0x7, 0x7, 0x7 };
const uint8_t MASK_L[10] = { 0xD, 0x0, 0xE, 0xA, 0xB, 0xB, 0xF, 0x0, 0xF, 0xB };
const uint8_t MASK_2_H[10] =
		{ 0x5, 0x7, 0x6, 0x2, 0x3, 0x3, 0x7, 0x0, 0x7, 0x3 };
const uint8_t MASK_2_L[10] =
		{ 0xF, 0x9, 0xB, 0xF, 0x6, 0xD, 0xD, 0x7, 0xF, 0xF };
/*!
 * @brief Function Name: write_digit
 *
 * The function writes a single digit number to the numeric display of the LCD
 *
 * @param digit: The digit to be updated. There are a total of 8 digits. Range[0:7]
 *               Least significant digit is indexed by 0 and most significant by 7
 * @param value: Single digit number to be written to display. Range[0:9]
 *
 * @return none
 */
void write_digit(uint8_t digit, uint8_t num) {

	//	/*test - write 0-9 on the first digit - i.e , digit = 1*/
	//	// registers are LCDM8 and LCDM9
	//	LCDM8 = BIT4 + BIT5 + BIT6 + BIT7;
	//	LCDM9 = BIT0 + BIT2;
	//	/*test - write 0-9 on the SECOND digit - i.e , digit = 2*/
	//	// registers are LCDM7 and LCDM8
	//	LCDM7 = BIT4 + BIT5 + BIT6 + BIT7;
	//	LCDM8 = BIT0 + BIT2;
	//	/*test - write 0-9 on the THIRD digit - i.e , digit = 3*/
	//	// registers are LCDM6 and LCDM7
	//	LCDM6 = BIT4 + BIT5 + BIT6 + BIT7;
	//	LCDM7 = BIT0 + BIT2;
	//	/*test - write 0-9 on the FOURTH digit - i.e , digit = 4*/
	//	// registers are LCDM5 and LCDM6
	//	LCDM5 = BIT4 + BIT5 + BIT6 + BIT7;
	//	LCDM6 = BIT0 + BIT2;
	//	/*test - write 0-9 on the FIFTH digit - i.e , digit = 5*/
	//	// registers are LCDM10 and LCDM11
	//	LCDM10 = BIT4 + BIT6;
	//	LCDM11 = BIT0 + BIT1 + BIT2 + BIT3;
	//	/*test - write 0-9 on the SIXTH digit - i.e , digit = 6*/
	//	// registers are LCDM4
	//	LCDM4 = BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT6;
	//	/*test - write 0-9 on the SEVENTH digit - i.e , digit = 7*/
	//	// registers are LCDM12
	//	LCDM12 = BIT0 + BIT2 + BIT4 + BIT5 + BIT6 + BIT7;
	//	/*test - write 0-9 on the EIGHTH digit - i.e , digit = 8*/
	//	// registers are LCDM13
	//	LCDM13 = BIT0 + BIT2 + BIT4 + BIT5 + BIT6 + BIT7;
	assert((digit < NUM_DIGITS) && (num <= 9));
	/* Activate all segments that make up the seven segment digit */
	*(NUM_REGS[digit][0]) |= MASK_H[num];
	*(NUM_REGS[digit][1]) |= MASK_L[num];
}

/*!
 * @brief Function Name: write_num
 *
 * The function writes a bcd formated number to the numeric display of the LCD
 *
 * @param bcdValue: The number to be displayed in bcd format
 * @param decimalPts: Decimal points applicable to the number
 * @param sign: Sign to be displayed for the number 0 for negative 1 for positive
 *
 * @return none
 * @note : An auxilary function bin_2_bcd() is provided in this library for binary int to bcd conversion
 */
void write_num(uint32_t digitPlace, uint64_t bcdValue, uint8_t decimalPts,
		uint8_t sign) {
	assert(bcdValue <= 0x99999999);

	volatile uint8_t i;
	volatile uint64_t _bcdValue;

	_bcdValue = bcdValue;
	for (i = digitPlace; i < NUM_DIGITS; i++) /*i = digitPlace means -> start writing from the specified digit place  */
	{
		write_digit(i, (0x000000000000000F & _bcdValue));
		_bcdValue >>= 4;

		if (_bcdValue == 0 && i >= decimalPts) {
			break;
		}
	}
	if (0 == sign) {
		assert((i + 1) < NUM_DIGITS);
		write_minus(i + 1);
	}
	write_dc_pt(decimalPts);
}

/*!
 * @brief Function Name: LCD_TurnOnSym
 *
 * The function turns on/off a symbol on the LCD based on the state value.
 *
 * @param sym: Parameter specifiying symbol to be displayed. Possible values are:
 *              \li logo: Manufacturer logo - Istron
 *              \li signal_bar: signal strength
 *              \li summation/accumulated: total accumulated value in register
 *              \li alarm_clk: alarm event
 *              \li calendar: calendar time
 *              \li serial_number: the device's unique serial number
 *				\li barcode: the device's barcode identifier
 *              \li time_diff: measured upstream/downstream flow time difference
 *              \li flow_dir_left: flow rotation arrow to the left
 *              \li flow_dir_right: flow rotation arrow to the right
 *              \li bus_line: bus line active
 *              \li warning: generic warning event
 *              \li battery_status: device is being powered from the battery
 *              \li column1: ?
 *              \li column2: ?
 *
 *
 * @return none
 */
void write_symbol(LCD_symbol sym, uint8_t state) {
	switch (sym) {
	case logo:
		if (state == ON) {
			LCDM10 |= BIT3;
			return;
		}
		LCDM10 &= ~BIT3;
		break;
	case signal_bar:
		/*!--todo: include digit*/
		//state == ON ? LCDM10 |= BIT3 : LCDM10 &= ~BIT3;
		break;
	case summation:
		if (state == ON) {
			LCDM5 |= BIT0;
			return;
		}
		LCDM5 &= ~BIT0;
		break;
	case alarm_clk:
		if (state == ON) {
			LCDM11 |= BIT4;
			return;
		}
		LCDM11 &= ~BIT4;
		break;
	case calendar:
		if (state == ON) {
			LCDM3 |= BIT4;
			return;
		}
		LCDM3 &= ~BIT4;
		break;
	case serial_number:
		if (state == ON) {
			LCDM3 |= BIT6;
			return;
		}
		LCDM3 &= ~BIT6;
		break;
	case barcode:
		if (state == ON) {
			LCDM3 |= BIT7;
			return;
		}
		LCDM3 &= ~BIT7;
		break;
	case time_diff:
		if (state == ON) {
			LCDM14 |= BIT3;
			return;
		}
		LCDM14 &= ~BIT3;
		break;
	case flow_dir_left:
		if (state == ON) {
			LCDM12 |= BIT3;
			return;
		}
		LCDM12 &= ~BIT3;
		break;
	case flow_dir_right:
		if (state == ON) {
			LCDM11 |= BIT7;
			return;
		}
		LCDM11 &= ~BIT7;
		break;
	case bus_line:
		if (state == ON) {
			LCDM10 |= BIT7;
			return;
		}
		LCDM10 &= ~BIT7;
		break;
	case warning:
		if (state == ON) {
			LCDM5 |= BIT3;
			return;
		}
		LCDM5 &= ~BIT3;
		break;
	case battery_status:
		if (state == ON) {
			LCDM9 |= BIT3;
			return;
		}
		LCDM9 &= ~BIT3;
		break;
	case column1:
		if (state == ON) {
			LCDM11 |= BIT6;
			return;
		}
		LCDM11 &= ~BIT6;
		break;
	case column2:
		if (state == ON) {
			LCDM5 |= BIT1;
			return;
		}
		LCDM5 &= ~BIT1;
		break;
	default:
		break;
	}
}
/*!
 * @brief Function Name: write_unit
 *
 * The function turns on/off a measurement unit on LCD
 *
 * @param sym: Parameter specifiying unit to be displayed. Possible values are:
 *              \li kilowatt_hour: kwh
 *              \li megawatt_hour: Mwh
 *              \li cubic_meters_per_hour: m3/h
 *              \li liters_per_hour: lt/h
 *              \li btu: BTU
 *              \li joules: J
 *              \li kelvin: K
 *              \li celcius: C
 *
 * @return none
 */
void write_unit(unit unit, uint8_t state) {
	switch (unit) {
	case kilowatt_hour:
		if (state == ON) {
			/*!--turn off other unit forms first*/
			LCDM14 &= ~BIT4;
			LCDM15 &= ~BIT1;

			LCDM15 |= BIT0;
			LCDM3 |= BIT5;
			return;
		}
		LCDM15 &= ~BIT0;
		LCDM3 &= ~BIT5;
		break;
	case megawatt_hour:
		if (state == ON) {
			/*!--turn off other unit forms first*/
			LCDM15 &= ~BIT0;
			LCDM3 &= ~BIT5;

			LCDM14 |= BIT4;
			LCDM15 |= BIT1;
			return;
		}
		LCDM14 &= ~BIT4;
		LCDM15 &= ~BIT1;
		break;
	case cubic_meters_per_hour:
		if (state == ON) {
			/*!--turn off other unit forms first*/
			LCDM14 &= ~BIT6;
			LCDM15 &= ~BIT3;

			LCDM14 |= BIT5;
			LCDM15 |= BIT2;
			return;
		}
		LCDM14 &= ~BIT5;
		LCDM15 &= ~BIT2;
		break;
	case liters_per_hour:
		if (state == ON) {
			/*!--turn off other unit forms first*/
			LCDM14 &= ~BIT5;
			LCDM15 &= ~BIT2;

			LCDM14 |= BIT6;
			LCDM15 |= BIT3;
			return;
		}
		LCDM14 &= ~BIT6;
		LCDM15 &= ~BIT3;
		break;
	case btu:
		if (state == ON) {
			LCDM14 &= ~BIT0;
			LCDM14 |= BIT7;
			return;
		}
		LCDM14 &= ~BIT7;
		break;
	case joules:
		if (state == ON) {
			LCDM14 &= ~BIT7;
			LCDM14 |= BIT0;
			return;
		}
		LCDM14 &= ~BIT0;
		break;
	case kelvin:
		if (state == ON) {
			LCDM14 &= ~BIT2;
			LCDM14 |= BIT1;
			return;
		}
		LCDM14 &= ~BIT1;
		break;
	case celcius:
		if (state == ON) {
			LCDM14 &= ~BIT1;
			LCDM14 |= BIT2;
			return;
		}
		LCDM14 &= ~BIT2;
		break;
	default:
		break;
	}
}
/*!
 * @brief Function Name: write_dc_pt
 *
 * Writes a decimal point at the specified digit
 *
 * @param the decimal point specifiying where to write the symbol
 *
 * @return none
 */
void write_dc_pt(uint8_t decimal_pt) {
	switch (decimal_pt) {
	case 1:
		LCDM4 &= ~BIT7;
		LCDM5 &= ~BIT2;
		LCDM6 &= ~BIT3;
		LCDM7 &= ~BIT3;
		LCDM8 |= BIT3;
		LCDM11 &= ~BIT6;
		LCDM13 &= ~BIT3;
		break;
	case 2:
		LCDM4 &= ~BIT7;
		LCDM5 &= ~BIT2;
		LCDM6 &= ~BIT3;
		LCDM7 |= BIT3;
		LCDM8 &= ~BIT3;
		LCDM11 &= ~BIT6;
		LCDM13 &= ~BIT3;
		break;
	case 3:
		LCDM4 &= ~BIT7;
		LCDM5 &= ~BIT2;
		LCDM6 |= BIT3;
		LCDM7 &= ~BIT3;
		LCDM8 &= ~BIT3;
		LCDM11 &= ~BIT6;
		LCDM13 &= ~BIT3;
		break;
	case 4:
		LCDM4 &= ~BIT7;
		LCDM5 |= BIT2;
		LCDM6 &= ~BIT3;
		LCDM7 &= ~BIT3;
		LCDM8 &= ~BIT3;
		LCDM11 &= ~BIT6;
		LCDM13 &= ~BIT3;
		break;
	case 5:
		LCDM4 |= BIT7;
		LCDM5 &= ~BIT2;
		LCDM6 &= ~BIT3;
		LCDM7 &= ~BIT3;
		LCDM8 &= ~BIT3;
		LCDM11 &= ~BIT6;
		LCDM13 &= ~BIT3;
		break;
	case 6:
		LCDM4 &= ~BIT7;
		LCDM5 &= ~BIT2;
		LCDM6 &= ~BIT3;
		LCDM7 &= ~BIT3;
		LCDM8 &= ~BIT3;
		LCDM11 |= BIT6;
		LCDM13 &= ~BIT3;
		break;
	case 7:
		LCDM4 &= ~BIT7;
		LCDM5 &= ~BIT2;
		LCDM6 &= ~BIT3;
		LCDM7 &= ~BIT3;
		LCDM8 &= ~BIT3;
		LCDM11 &= ~BIT6;
		LCDM13 |= BIT3;
		break;
	default:
		break;
	}
}
#endif
