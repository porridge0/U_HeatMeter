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
#include "util/clock.h"
#include "config/system.h"
#include "signal/constants.h"
#include "signal/tdc.h"
#include "display/lcdx.h"

/*
 * main.c
 */
/*!-- GLOBALS */
extern volatile snd_req_status c_status;

volatile uint8_t c_page = 0; //default LCD page

/*!-- Temperature */
volatile float t_PT1 = 0;
volatile float t_PT2 = 0;
volatile float t_PT3 = 0;
volatile float t_PT4 = 0;
volatile float t_REF = 0;
volatile float t_COLD = 0; //In degree celcius
volatile float t_HOT = 0; //In degree celcius
volatile float t_DIFF = 0; //In degree celcius

// Correction factor for clock frequency
float CLKHS_freq_corr_fact = 1.000;

const float CLKHS_freq = 4.000;           // Clock frequency in MHz
float CLKHS_freq_cal = 4.000;     // Calibrated Clock frequency in MHz

volatile float Ratio_RT1_Rref = 0;
volatile float Ratio_RT2_Rref = 0;
volatile float R1 = 0;   // Resistance measured in Ohms
volatile float R2 = 0; 	 // Resistance measured in Ohms

/*!--TOF */
float Result_0_up;
float Result_1_up;
float Result_2_up;

float Result_0_down;
float Result_1_down;
float Result_2_down;

float average_Result_up;
float average_Result_down;
float Time_of_flight_diff;
float Time_of_flight_sum;
float TOF_up;
float TOF_down;
float PW1ST;

float TOF_diff_avg = 0;
float TOF_diff_sum = 0;
float TOF_sum_avg = 0;
float TOF_sum_sum = 0;
float TOF_diff_square_sum = 0;
float Std_Dev_of_Diff = 0;
uint16_t sum_counter = 1; // starts with 1

/*!--Flow Rate*/
volatile uint16_t c_flowRate;
uint16_t vf_t; //velocity at temp T
uint16_t vs_t; // speed of sound in water at T


volatile uint64_t c_prevEnReg = 0; //previously logged reg value
/*!--Flags */
typedef struct {
	uint8_t _reedSW;
	uint8_t _powerFail;
	uint8_t _tdcINT;
	uint8_t _pushBTN;
} sys_flags;

#pragma PERSISTENT(_m_index)
/*!-- Track logged month values
 *  Index starts at 0 corresponding to start date(month) of operation.
 */
volatile uint8_t _m_index = 0;

volatile sys_flags flags = { ._reedSW = 0, ._powerFail = 0, ._tdcINT = 0,
		._pushBTN = 0 };
/*!-- Power Sources
 *              \li 0: Bus -- Default
 *              \li 1: Battery
 */
volatile uint8_t _pSource = 0;

extern volatile uint8_t _sampling_timeUp;
extern volatile uint8_t _page_timeUp;

static void gpio_config(void);
void inline scroll();

//#pragma DATA_SECTION(systemData, ".infoC");
//SystemData_T systemData;
int main(void) {
	// Disable the GPIO power-on default high-impedance mode to activate
	// previously configured port settings
	WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
	sysClock_config();
	gpio_config();
	lcd_config();
#ifdef TEST_ON
	all_on();
	all_off();
#endif
	rtc_config();
	ux_config(THREE_HUNDERED);
	//uint8_t *pa_changed_status = (uint8_t *) calloc(1, sizeof(uint8_t));
	start_tdc();

	_enable_interrupt();
	__bis_SR_register(GIE);

	write_symbol(logo, ON);
	while (1) {
		if (flags._powerFail) {
			if (_pSource) {
				write_symbol(bus_line, OFF);
				write_symbol(battery_status, ON);
			} else {
				write_symbol(battery_status, OFF);
				write_symbol(bus_line, ON);
			}
			_pSource ^= _pSource;
			flags._powerFail = 0;
		}
		if (flags._pushBTN) {
			//Handle tampering
			//TODO: When to clear : WAIT FOR A CLEAR COMMNAD
			write_symbol(warning, ON);
		}
		if (flags._reedSW) {
			//Handle LCD scroll
			scroll();
		}
		/*measurement cycle -> 15 seconds sampling*/
		if (_sampling_timeUp) {
			/*
			 * Wait for interrupt before reading registers
			 * --------------------------------------------
			 * 1. Calibrate high speed clock
			 */
			_sampling_timeUp = 0;
			startCalRes();
			while (!flags._tdcINT) { //poll int flag
				LPM0;
			}
			//calculations depend on reigster config settings
			float iv = transfer4B(OPCODE_READ_ADDRESS, 0, 0, 0, 0) / pow(2, 16);
			CLKHS_freq_corr_fact = (61.03515625 / iv) * CLKHS_freq;

			CLKHS_freq_cal = CLKHS_freq * CLKHS_freq_corr_fact; // Calibrated Clock frequency

			//2. Measure temperature
			float dv = CLKHS_freq_cal * pow(2, 16);
			flags._tdcINT = 0; // clear flag
			startTemp();
			while (!flags._tdcINT) { //poll int flag
				LPM0;
			}
			if (!read_error_status()) {
				t_PT1 = transfer4B(OPCODE_READ_ADDRESS, 0, 0, 0, 0) / dv;
				t_PT2 = transfer4B(OPCODE_READ_RES_1, 0, 0, 0, 0) / dv;
				t_PT3 = transfer4B(OPCODE_READ_RES_2, 0, 0, 0, 0) / dv;
				t_PT4 = transfer4B(OPCODE_READ_RES_3, 0, 0, 0, 0) / dv;

				t_REF = (t_PT3 + t_PT4) / 2;
				// Calculate Temp_cold at PT1
				Ratio_RT1_Rref = t_PT1 / t_REF * corr_fact;
				R1 = Ratio_RT1_Rref * R0_at_0C;
				t_COLD = CAL_TEMP(R1);
				// Calculate Temp_hot at PT2
				Ratio_RT2_Rref = t_PT2 / t_REF * corr_fact;
				R1 = Ratio_RT2_Rref * R0_at_0C;
				t_HOT = CAL_TEMP(R2);
				//Change in temp
				t_DIFF = abs(t_HOT - t_COLD);
			}
			//3. Measure time of flight
			uint8_t _ec = 0; //error count
			flags._tdcINT = 0; // clear flag

			startTOF();	// Start TOF measurement

			while (!flags._tdcINT) { //poll int flag
				LPM0;
			}
			_ec += read_error_status();
			// Result_UP, TOF in µs
			// RES_0 to RES_2 to be displayed only for evaluation purposes
			// RES_3 will be used for e.g. flow calculation
			Result_0_up = transfer4B(OPCODE_READ_ADDRESS, 0, 0, 0, 0) / dv;
			Result_1_up = transfer4B(OPCODE_READ_RES_1, 0, 0, 0, 0) / dv;
			Result_2_up = transfer4B(OPCODE_READ_RES_2, 0, 0, 0, 0) / dv;

			average_Result_up = transfer4B(OPCODE_READ_RES_3, 0, 0, 0, 0) / dv;

			flags._tdcINT = 0;

			init();

			while (!flags._tdcINT) { //poll int flag
				LPM0;
			}

			_ec += read_error_status();

			// Result_DOWN, TOF in µs
			// RES_0 to RES_2 to be displayed only for evaluation purposes
			// RES_3 will be used for e.g. flow calculation
			Result_0_down = transfer4B(OPCODE_READ_ADDRESS, 0, 0, 0, 0) / dv;
			Result_1_down = transfer4B(OPCODE_READ_RES_1, 0, 0, 0, 0) / dv;
			Result_2_down = transfer4B(OPCODE_READ_RES_2, 0, 0, 0, 0) / dv;

			average_Result_down = transfer4B(OPCODE_READ_RES_3, 0, 0, 0, 0)
					/ dv;

			if (!_ec) {
				// Result after two measurements (first UPSTREAM then DOWNSTREAM)
				// Calculate UP- / DOWNSTREAM transit time difference with MCU

				// Divider for multihit sum (1..3)
				average_Result_up = AVG_BY_HITS(average_Result_up, 3);
				average_Result_down = AVG_BY_HITS(average_Result_down, 3);

				// discharge time in ns
				Result_0_up = PICO_TO_NANO(Result_0_up);
				Result_1_up = PICO_TO_NANO(Result_1_up);
				Result_2_up = PICO_TO_NANO(Result_2_up);
				average_Result_up = PICO_TO_NANO(average_Result_up);

				Result_0_down = PICO_TO_NANO(Result_0_down);
				Result_1_down = PICO_TO_NANO(Result_1_down);
				Result_2_down = PICO_TO_NANO(Result_2_down);
				average_Result_down = PICO_TO_NANO(average_Result_down);

				TOF_up = average_Result_up;
				TOF_down = average_Result_down;

				float fb = pow(2, 7); // fractional bits
				PW1ST += transfer1B(OPCODE_READ_PW1ST, 0) / fb;

				Time_of_flight_diff = TOF_up - TOF_down;

				Time_of_flight_sum = TOF_up + TOF_down;

				// to add up
				TOF_diff_sum += Time_of_flight_diff;
				TOF_diff_square_sum = TOF_diff_square_sum
						+ (Time_of_flight_diff * Time_of_flight_diff);
				TOF_sum_sum += Time_of_flight_sum;
				sum_counter++;

				if (sum_counter > NUMBER_OF_SAMPLES) // Output after no_of_avg measurements
				{
					TOF_diff_avg = TOF_diff_sum / NUMBER_OF_SAMPLES;
					Std_Dev_of_Diff = STAND_VAR(TOF_diff_square_sum,
							NUMBER_OF_SAMPLES);

					Std_Dev_of_Diff = 0;
					TOF_diff_sum = 0;
					TOF_diff_square_sum = 0;
					sum_counter = 1;
					PW1ST = 0;
				}
				//3. Measure flow rate
				vf_t = FLUID_VELOCITY(vs_t, TOF_up, TOF_down);
				c_flowRate = FLOW_RATE(vf_t);

				//4. Measure cumulative energy
				c_enReg += IN_MILLI_JOULES(HEAT_ENERGY(c_flowRate, t_DIFF));

			}
		}
		if (!_page_timeUp) {
			scroll();
			_page_timeUp = 5;
		}
		if (c_status == response_ready) {
			x_lnk_res();
		}
		if (_dayChanged == 1) {
			//log registers
		}
		if (_monChanged == 1) {
			/*Capture value at end of month */
			_month_reg[_m_index].m_reg = c_enReg - c_prevEnReg;
			_month_reg[_m_index].timeStamp = currentTime;
			_m_index++;
			c_prevEnReg = c_enReg; //copy current value
			if (_m_index > 11) {
				_m_index = 0;
			}
			/*track month and day changes to update running registers*/
			if (_yrChanged == 1) {
				/*todo*/
			}
		}
	}
}

/* Display different measured and instantaneous
 * parameters on the LCD.
 *
 * Delay between consecutive pages set at 5 seconds
 *
 * */
void inline scroll() {
	switch (c_page) {
	case 0: // this is actually the first page

		/*Display energy*/
		write_symbol(page_1, ON);
		write_num(0, c_enReg, 3, 1); //decimal point 3
		write_symbol(joules, ON);
		write_symbol(flow_dir_left, ON);
		write_symbol(flow_dir_right, ON);
		write_symbol(summation, ON);

		write_symbol(alarm_clk, OFF);
		write_symbol(calendar, OFF);
		write_symbol(serial_number, OFF);
		write_symbol(barcode, OFF);
		write_symbol(kilowatt_hour, OFF);
		write_symbol(megawatt_hour, OFF);
		write_symbol(liters_per_hour, OFF);
		write_symbol(btu, OFF);
		write_symbol(kelvin, OFF);
		write_symbol(celcius, OFF);
		write_symbol(time_diff, OFF);

		break;
	case 1:
		/*Display temperature*/
		write_symbol(page_2, ON);
		write_num(0, t_DIFF, 0, 1);
		write_symbol(celcius, ON);

		write_symbol(alarm_clk, OFF);
		write_symbol(calendar, OFF);
		write_symbol(serial_number, OFF);
		write_symbol(barcode, OFF);
		write_symbol(kilowatt_hour, OFF);
		write_symbol(megawatt_hour, OFF);
		write_symbol(liters_per_hour, OFF);
		write_symbol(btu, OFF);
		write_symbol(kelvin, OFF);
		write_symbol(summation, OFF);
		write_symbol(time_diff, OFF);

		break;
	case 2:
		/*Display flow rate*/
		write_symbol(page_3, ON);
		write_num(0, c_flowRate, 1, 1); //decimal point 1
		write_symbol(liters_per_hour, ON);

		write_symbol(alarm_clk, OFF);
		write_symbol(calendar, OFF);
		write_symbol(serial_number, OFF);
		write_symbol(barcode, OFF);
		write_symbol(kilowatt_hour, OFF);
		write_symbol(megawatt_hour, OFF);
		write_symbol(liters_per_hour, OFF);
		write_symbol(btu, OFF);
		write_symbol(kelvin, OFF);
		write_symbol(celcius, OFF);
		write_symbol(summation, OFF);
		write_symbol(time_diff, OFF);

		break;
	case 3:
		/*Display current time*/
		write_symbol(page_4, ON);
		write_symbol(calendar, ON);
		write_time(currentTime.day, currentTime.month, currentTime.year);

		write_symbol(cubic_meters_per_hour, OFF);
		write_symbol(alarm_clk, OFF);
		write_symbol(serial_number, OFF);
		write_symbol(barcode, OFF);
		write_symbol(kilowatt_hour, OFF);
		write_symbol(megawatt_hour, OFF);
		write_symbol(liters_per_hour, OFF);
		write_symbol(btu, OFF);
		write_symbol(kelvin, OFF);
		write_symbol(celcius, OFF);
		write_symbol(summation, OFF);
		write_symbol(time_diff, OFF);

		break;
	case 4:
		/*Display serial number*/
		write_symbol(page_4, OFF);
		write_symbol(serial_number, ON);
		write_symbol(barcode, ON);
		write_num(0, devInfo.serialNumber, 0, 1); //decimal point 1

		write_symbol(cubic_meters_per_hour, OFF);
		write_symbol(alarm_clk, OFF);
		write_symbol(calendar, OFF);
		write_symbol(kilowatt_hour, OFF);
		write_symbol(megawatt_hour, OFF);
		write_symbol(liters_per_hour, OFF);
		write_symbol(btu, OFF);
		write_symbol(kelvin, OFF);
		write_symbol(summation, OFF);
		write_symbol(celcius, OFF);
		write_symbol(time_diff, OFF);

		break;
	default:
		break;
	}
	c_page++;
	if (c_page > 4)
		c_page = 0;

}

static void gpio_config(void) {
	/*CECTL3 = 0xFF00;*/

	PM5CTL0 &= ~LOCKLPM5; /* In initialization, the I/Os are configured before unlocking the I/O ports*/
	/* Port Configuration */

	P1SEL0 = 0x00;
	P1SEL1 = 0x00;
	P2SEL0 = 0x00;
	P2SEL1 = 0x00;
	P3SEL0 = 0x00;
	P3SEL1 = 0x00;
	P4SEL0 = 0x00;
	P4SEL1 = 0x00;
	P5SEL0 = 0x00;
	P5SEL1 = 0x00;
	P6SEL0 = 0x00;
	P6SEL1 = 0x00;
	P7SEL0 = 0x00;
	P7SEL1 = 0x00;
	P9SEL0 = 0x00;
	P9SEL1 = 0x00;

	P1DIR = 0x00;
	P1DIR |= BIT1;
	P2DIR = 0x00;
	P3DIR = 0x00;
	P4DIR = 0x00;
	P5DIR = 0x00;
	P6DIR = 0x00;
	P6DIR |= BIT2;
	P7DIR = 0x00;
	P9DIR = 0x00;
	P9DIR |= BIT3;
	P9DIR |= BIT5;
	P9DIR |= BIT6;
	PJDIR = 0xFF;

	P1REN = 0xFF;
	P2REN = 0xFF;
	P3REN = 0xFF;
	P4REN = 0xFF;
	P4REN &= ~BIT3; /***/
	P5REN = 0xFF;
	P6REN = 0xFF;
	P7REN = 0xFF;
	P9REN = 0xFF;

	PJOUT = 0x00;
	/*-------------------*/
	P1OUT = 0x00; /***/
	/*-------------------*/
	P2OUT = 0x00;
	/*-------------------*/
	P3OUT = 0x00;
	/*-------------------*/
	P4OUT = 0x00;
	/*-------------------*/
	P5OUT = 0x00;
	/*-------------------*/
	P6OUT = 0x00;
	/*-------------------*/
	P7OUT = 0x00;
	/*-------------------*/
	/*-------------------*/
	P9OUT = 0x00;
	/*-------------------*/

	/* Setup port interrupts */
	CNFG_PORT_1_DIR &= ~CNFG_PUSH_BTN_INT_BIT; /* Set input direction */
	CNFG_PORT_1_DIR &= ~CNFG_MBUS_POWER_FAIL_BIT; /* Set input direction */
	CNFG_PORT_1_DIR &= ~CNFG_TDC_INTN_BIT; /* Set input direction */
	CNFG_PORT_1_DIR &= ~CNFG_REED_SWTCH_INT_BIT; /* Set input direction */

	CNFG_PORT_1_IES &= ~CNFG_PUSH_BTN_INT_BIT; /* Select rising edge for pin interrupt */
	CNFG_PORT_1_IE |= CNFG_PUSH_BTN_INT_BIT; /* Enable interrupt */

	CNFG_PORT_1_IES &= ~CNFG_MBUS_POWER_FAIL_BIT; /* Select rising edge for pin interrupt */
	CNFG_PORT_1_IE |= CNFG_MBUS_POWER_FAIL_BIT; /* Enable interrupt */

	CNFG_PORT_1_IES &= ~CNFG_TDC_INTN_BIT; /* Select rising edge for pin interrupt */
	CNFG_PORT_1_IE |= CNFG_TDC_INTN_BIT; /* Enable interrupt */

	CNFG_PORT_1_IES &= ~CNFG_REED_SWTCH_INT_BIT; /* Select rising edge for pin interrupt */
	CNFG_PORT_1_IE |= CNFG_REED_SWTCH_INT_BIT; /* Enable interrupt */

}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT1_VECTOR
__interrupt void ISR_PORT1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT1_VECTOR))) ISR_PORT1 (void)
#else
#error Compiler not supported!
#endif
{
	switch (__even_in_range(P1IV, 0x10)) {
	case 0x00:
		break; /* No Interrupt Pending */
	case 0x02:
		flags._pushBTN = 1;
		break; /* Port 1.0 Interrupt */
	case 0x04:
		flags._powerFail = 1; //set M-Bus power fail flag
		break; /* Port 1.1 Interrupt */
	case 0x06:
		flags._tdcINT = 1; // set int_flag
		break; /* Port 1.2 Interrupt */
	case 0x08: /* Port 1.3 Interrupt */
		flags._reedSW = 1;
		break;
	case 0x0A:
		break; /* Port 1.4 Interrupt */
	case 0x0C: /* Port 1.5 Interrupt */
		break;
	case 0x0E:
		break; /* Port 1.6 Interrupt */
	case 0x10:
		break; /* Port 1.7 Interrupt */
	default:
		break;
	}
}
