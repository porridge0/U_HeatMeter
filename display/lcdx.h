/*!
 * @file lcdx.h
 *
 *	File containing functions and constant definitions used to abstract the manipulation of the LCD
 *
 *  @date: Oct 16, 2020
 *
 *  @author: Deksios Bekele
 */

#ifndef DISPLAY_LCDX_H_
#define DISPLAY_LCDX_H_

#include <stdint.h>

typedef enum _unit {
	kilowatt_hour,
	megawatt_hour,
	cubic_meters_per_hour,
	liters_per_hour,
	btu,
	joules,
	kelvin,
	celcius,
} unit;
typedef enum _flowDir {
	dir_left = 0, dir_right = 1, dir_none = 2
} flowDir;

#define ON   (uint8_t)0
#define OFF  (uint8_t)!ON
typedef enum _symbol {
	logo,
	signal_bar,
	summation,
	alarm_clk,
	calendar,
	serial_number,
	barcode,
	time_diff,
	flow_dir_right,
	flow_dir_left,
	bus_line,
	warning,
	battery_status,
	dc_point1,
	dc_point2,
	dc_point3,
	dc_point4,
	dc_point5,
	dc_point6,
	dc_point7,
	column1,
	column2
} LCD_symbol;

void LCD_Setup();

uint64_t bin_2_bcd(int32_t bin);

void LCD_WriteDigit(uint8_t digit, uint8_t value);
void write_num(uint32_t digitPlace, uint64_t bcdValue, uint8_t decimalPts,
		uint8_t sign);
void write_minus(uint8_t pos);
void clear_digits();

void LCD_SetBatLevel(uint8_t level);
void LCD_SetRFSigLevel(uint8_t level);
void LCD_SetFlowDir(flowDir dir);
void LCD_SetUnit(unit u);
void write_dc_pt(uint8_t point);

void LCD_SpinnerStep(int8_t spinnerState);

void LCD_TurnOnSym(LCD_symbol sym);
void LCD_TurnOffSym(LCD_symbol sym);

void all_on();
void all_off();

#endif /* DISPLAY_LCDX_H_ */
