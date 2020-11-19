/******************* (C) COPYRIGHT 2012 acam messelectronic GmbH ***************
 ******************** (C) COPYRIGHT 2008 STMicroelectronics *********************
 * File Name          : main.c
 * Author             : acam Support Team
 * Version            :
 * Date               : 10-07-2013
 * Description        : Simple demonstration program to run TDC-GP22 in a
 *                      heatmeter application with ToF measurement, temperature
 *                      measurement and random clock calibration.
 *                      This is a modified version for an MSP430FR6820.
 *******************************************************************************
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH AN EXAMPLE CODING INFORMATION REGARDING OUR PRODUCTS. THE ACAM
 * RECOMMENDATIONS ARE BELIEVED USEFUL AND OPERABLE, NEVERTHELESS IT IS OF THE
 * CUSTOMER'S SOLE RESPONSIBILITY TO MODIFY, TEST AND VALIDATE THEM BEFORE
 * SETTING UP ANY PRODUCTION PROCESS. AS A RESULT, ACAM SHALL NOT BE HELD LIABLE
 * FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS
 * ARISING FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF
 * THE CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <msp430.h>
#include <stdlib.h>
#include "stdio.h"

/* Private variables ---------------------------------------------------------*/
#define LOOP_DLY_100US    2000
#define LOOP_DLY_250ns    2
#define LOOP_DLY_1ms      3299

typedef uint8_t bool;

uint32_t CriticalSecCntr;
bool configured_true = FALSE;

float CLKHS_freq = 4.000;           // Clock frequency in MHz
float CLKHS_freq_cal = 4.000;       // Calibrated Clock frequency in MHz
float CLKHS_freq_corr_fact = 1.000; // Correction factor for Clock frequency

float Result_0_up;
float Result_1_up;
float Result_2_up;

float Result_0_down;
float Result_1_down;
float Result_2_down;

float average_Result_up;
float average_Result_down;
float Time_of_flight_diff, Time_of_flight_sum;
float TOF_up;
float TOF_down;
float PW1ST;

float TOF_diff_avg = 0, TOF_diff_sum = 0, TOF_sum_avg = 0, TOF_sum_sum = 0;
float TOF_diff_square_sum = 0, Std_Dev_of_Diff = 0;
uint16_t no_of_avg = 10;    // numbers of measurements to define the std.dev.
uint16_t sum_counter = 1; // starts with 1

float Temp_PT1 = 0, Temp_PT2 = 0, Temp_PT3 = 0, Temp_PT4 = 0, Temp_REF = 0;
float Temp_cold = 0, Temp_cold_in_Celcius = 0; //PT1 (Sensor)
float Temp_hot = 0, Temp_hot_in_Celcius = 0; //PT2 (Sensor)
float Ratio_RT1_Rref = 0, Ratio_RT2_Rref = 0;

uint8_t Error_Bit = 0;
uint16_t status_byte_UP, status_byte_DOWN;

/* PT1000 --------------------------------------------------------------------*/
//R0, A, B are parameters as specified in EN60 751
float R0_at_0C = 1000;            // R0 is the RTD resistance at 0 캜
float Coeff_A = 3.9083 / 1000;     // A = 3,9083 x 10-3 캜-1
float Coeff_B = -5.775 / 10000000; // B = -5,775 x 10-7 캜-1
float R1 = 0, R2 = 0;             // R1, R2 is resistance measured in Ohm
float corr_fact = 1.0000;   // Corr.-factor for temperature resistance ratio

/* Opcodes -------------------------------------------------------------------*/
uint8_t Init = 0x70;
uint8_t Power_On_Reset = 0x50;
uint8_t Start_TOF = 0x01;
uint8_t Start_Temp = 0x02;
uint8_t Start_Cal_Resonator = 0x03;
uint8_t Start_Cal_TDC = 0x04;
uint8_t Start_TOF_Restart = 0x05;
uint8_t Start_Temp_Restart = 0x06;

uint32_t *sram_memory = ((uint32_t *) (SRAM_BASE + 0xB00));
uint32_t sram_mem_offset = 0x0;

uint32_t Dummy_var = 0;

// For mathematical calculations
int i;
int j;

/* Device functions ----------------------------------------------------------*/
void gp22_send_1byte(void *bus_type, uint8_t gp22_opcode_byte);
void gp22_wr_config_reg(void *bus_type, uint8_t opcode_address,
		uint32_t config_reg_data);
float gp22_read_n_bytes(void *bus_type, uint8_t n, uint8_t read_opcode,
		uint8_t read_addr, uint8_t fractional_bits);

uint16_t gp22_read_status_bytes(void *bus_type);
uint8_t gp22_status_count_error(void *bus_type);
void gp22_analyse_error_bit(void *bus_type);

/* Bus functions -------------------------------------------------------------*/
void SPIx_GPIOs_Init(void* bus_type);
void SPIx_Interface_Init(void* bus_type);

/* Private functions ---------------------------------------------------------*/
void Dly100us(void *arg);
void Dly250ns(void *arg);
void Dly1ms(void *arg);
void Simple_delay_750ns(void *arg);

/*******************************************************************************
 * Function Name  : main
 * Description    : Main program.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void main(void) {
	ENTR_CRT_SECTION();
	/* Setup STM32 system (clock, PLL and Flash configuration) */
	SystemInit();

	EXT_CRT_SECTION();

	// Choose your Slot (SPI1, SPI2)
	void* Bus_Type = SPI1;

	/* controlled loop */
	while (Dummy_var != 11) // To control the loop, e.g. (Dummy_var!=7)
	{
		if (Dummy_var == 10)
			Dummy_var = 0; // Infinite loop

		if (configured_true == FALSE) {
			configured_true = TRUE;
			SPIx_GPIOs_Init(Bus_Type);
			SPIx_Interface_Init(Bus_Type);

			gp22_send_1byte(Bus_Type, Power_On_Reset);
			Dly100us((void*) 5);              // 500 us wait for GP22

			// Writing to the configuration registers (CR)
			// CR0: ANZ_FIRE=d20 DIV_FIRE=d3 ANZ_PER_CALRES=d0 ANZ_PORT=1...
			gp22_wr_config_reg(Bus_Type, 0x80, 0x430BE800);
			// CR1: ...
			gp22_wr_config_reg(Bus_Type, 0x81, 0x21444000);
			// CR2: EN_INT=b0101 RFEDGE1=RFEDGE=0 DELVAL1=d5000 ID2=0
			gp22_wr_config_reg(Bus_Type, 0x82, 0xA0138800);
			// CR3: EN_AUTOCALC=1 EN_FIRST_WAVE=1 DELREL1=d8 DELREL2=d9 DELREL3=d10
			gp22_wr_config_reg(Bus_Type, 0x83, 0xD0A24800);
			// CR4: DIS_PW=0 EDGE_PW=0 OFFSRNG1=0 OFFSRNG2=1 OFFS=0 ID4=0
			gp22_wr_config_reg(Bus_Type, 0x84, 0x10004000);
			// CR5: FIRE_UP=1
			gp22_wr_config_reg(Bus_Type, 0x85, 0x40000000);
			// CR6: ...
			gp22_wr_config_reg(Bus_Type, 0x86, 0xC0C06100);
		}

		// .........................................................................
		// ........................Calibrate High Speed Clock.......................
		// .........................Temperature Measurment..........................
		// .......................TIME OF FLIGHT MEASUREMENT........................
		// ......................Sum result of multihit values......................

		if ((Dummy_var == 0) | (Dummy_var == 10)) {
			//--------------------------------------------------------------------------
			// Start Calibrate High Speed Clock Cycle
			gp22_send_1byte(Bus_Type, Init);
			gp22_send_1byte(Bus_Type, Start_Cal_Resonator);

			// Wait for INT Slot_x
			if (Bus_Type == SPI1)
				while (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_4) == 1)
					;
			if (Bus_Type == SPI2)
				while (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11) == 1)
					;

			//Calculate Correction factor
			//The time interval to be measured is set by ANZ_PER_CALRES
			//which defines the number of periods of the 32.768 kHz clock:
			//2 periods = 61.03515625 탎
			CLKHS_freq_corr_fact = 61.03515625
					/ gp22_read_n_bytes(Bus_Type, 4, 0xB0, 0x00, 16)
					* CLKHS_freq;

			printf("\n Correction factor for clock = %1.4f\n",
					CLKHS_freq_corr_fact);

			CLKHS_freq_cal = CLKHS_freq * CLKHS_freq_corr_fact; // Calibrated Clock frequency
		}

		if ((Dummy_var == 0) | (Dummy_var == 5) | (Dummy_var == 10)) {
			//--------------------------------------------------------------------------
			// Start Temperature Measurement Cycle
			gp22_send_1byte(Bus_Type, Init);
			gp22_send_1byte(Bus_Type, Start_Temp);

			// Wait for INT Slot_x
			if (Bus_Type == SPI1)
				while (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_4) == 1)
					;
			if (Bus_Type == SPI2)
				while (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11) == 1)
					;

			// Read the Status Register
			printf("\nStatus Register = 0x%04X",
					gp22_read_status_bytes(Bus_Type));

			Error_Bit = gp22_status_count_error(Bus_Type);
			if (Error_Bit > 0) {
				gp22_analyse_error_bit(Bus_Type);
				printf("\nTemp-Measurement are skipped\n");
			}

			else {
				//discharge time in 탎
				Temp_PT1 = gp22_read_n_bytes(Bus_Type, 4, 0xB0, 0x00, 16)
						/ CLKHS_freq_cal;
				Temp_PT2 = gp22_read_n_bytes(Bus_Type, 4, 0xB0, 0x01, 16)
						/ CLKHS_freq_cal;
				Temp_PT3 = gp22_read_n_bytes(Bus_Type, 4, 0xB0, 0x02, 16)
						/ CLKHS_freq_cal;
				Temp_PT4 = gp22_read_n_bytes(Bus_Type, 4, 0xB0, 0x03, 16)
						/ CLKHS_freq_cal;

				Temp_REF = (Temp_PT3 + Temp_PT4) / 2;

				// Calculate Temp_cold at PT1
				Ratio_RT1_Rref = Temp_PT1 / Temp_REF * corr_fact;
				R1 = Ratio_RT1_Rref * R0_at_0C;
				Temp_cold_in_Celcius = (-R0_at_0C * Coeff_A
						+ sqrt(
								((R0_at_0C * Coeff_A) * (R0_at_0C * Coeff_A))
										- 4 * R0_at_0C * Coeff_B
												* (R0_at_0C - R1)))
						/ (2 * R0_at_0C * Coeff_B);

				// Calculate Temp_hot at PT2
				Ratio_RT2_Rref = Temp_PT2 / Temp_REF * corr_fact;
				R2 = Ratio_RT2_Rref * R0_at_0C;
				Temp_hot_in_Celcius = (-R0_at_0C * Coeff_A
						+ sqrt(
								((R0_at_0C * Coeff_A) * (R0_at_0C * Coeff_A))
										- 4 * R0_at_0C * Coeff_B
												* (R0_at_0C - R2)))
						/ (2 * R0_at_0C * Coeff_B);

				// Print result
				printf(
						"\n\tPT1= %6.3f탎 / PT2= %6.3f탎 \n\tPT3= %6.3f탎 / PT4= %6.3f탎",
						Temp_PT1, Temp_PT2, Temp_PT3, Temp_PT4);
				printf("\n Ratio RT1/Rref= %2.5f / Ratio RT2/Rref= %2.5f",
						Ratio_RT1_Rref, Ratio_RT2_Rref);
				printf("\n Temp_cold= %3.3f캜 / Temp_hot= %3.3f캜\n",
						Temp_cold_in_Celcius, Temp_hot_in_Celcius);
			}

		}

		//--------------------------------------------------------------------------
		// Start Time Of Flight Measurement Cycle
		gp22_send_1byte(Bus_Type, Init);

		gp22_send_1byte(Bus_Type, Start_TOF_Restart);

		Error_Bit = 0;

		// Wait for INT Slot_x, UPSTREAM
		if (Bus_Type == SPI1)
			while (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_4) == 1)
				;
		if (Bus_Type == SPI2)
			while (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11) == 1)
				;

		// Read the Status Register
		status_byte_UP = gp22_read_status_bytes(Bus_Type);
		Error_Bit += gp22_status_count_error(Bus_Type);

		// Result_UP, TOF in 탎
		// RES_0 to RES_2 to be displayed only for evaluation purposes
		// RES_3 will be used for e.g. flow calculation
		Result_0_up = gp22_read_n_bytes(Bus_Type, 4, 0xB0, 0x00, 16)
				/ CLKHS_freq_cal;
		Result_1_up = gp22_read_n_bytes(Bus_Type, 4, 0xB0, 0x01, 16)
				/ CLKHS_freq_cal;
		Result_2_up = gp22_read_n_bytes(Bus_Type, 4, 0xB0, 0x02, 16)
				/ CLKHS_freq_cal;
		average_Result_up = gp22_read_n_bytes(Bus_Type, 4, 0xB0, 0x03, 16)
				/ CLKHS_freq_cal;

		gp22_send_1byte(Bus_Type, Init);

		// Wait for INT Slot_x, DOWNSTREAM
		if (Bus_Type == SPI1)
			while (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_4) == 1)
				;
		if (Bus_Type == SPI2)
			while (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11) == 1)
				;

		// Read the Status Register
		status_byte_DOWN = gp22_read_status_bytes(Bus_Type);
		Error_Bit += gp22_status_count_error(Bus_Type);

		// Result_DOWN, TOF in 탎
		// RES_0 to RES_2 to be displayed only for evaluation purposes
		// RES_3 will be used for e.g. flow calculation
		Result_0_down = gp22_read_n_bytes(Bus_Type, 4, 0xB0, 0x00, 16)
				/ CLKHS_freq_cal;
		Result_1_down = gp22_read_n_bytes(Bus_Type, 4, 0xB0, 0x01, 16)
				/ CLKHS_freq_cal;
		Result_2_down = gp22_read_n_bytes(Bus_Type, 4, 0xB0, 0x02, 16)
				/ CLKHS_freq_cal;
		average_Result_down = gp22_read_n_bytes(Bus_Type, 4, 0xB0, 0x03, 16)
				/ CLKHS_freq_cal;

		if (Error_Bit > 0) {
			gp22_analyse_error_bit(Bus_Type);
			printf("\nTOF-Measurement are skipped\n");
		}

		else

		{
			//------------------------------------------------------------------------
			// Result after two measurements (first UPSTREAM then DOWNSTREAM)
			// Calculate UP- / DOWNSTREAM transit time difference with MCU

			// Divider for multihit sum (1..3)
			average_Result_up /= 3;
			average_Result_down /= 3;

			// discharge time in ns
			Result_0_up *= 1000;
			Result_1_up *= 1000;
			Result_2_up *= 1000;
			average_Result_up *= 1000;

			Result_0_down *= 1000;
			Result_1_down *= 1000;
			Result_2_down *= 1000;
			average_Result_down *= 1000;

			TOF_up = average_Result_up;
			TOF_down = average_Result_down;
			PW1ST += gp22_read_n_bytes(Bus_Type, 1, 0xB0, 0x08, 7);

			// print results
			printf("\nStatus Byte Up= 0x%04X / Down= 0x%04X", status_byte_UP,
					status_byte_DOWN);

			printf("\n Result_0 \tUp= %6.3f ns / Down= %6.3f ns", Result_0_up,
					Result_0_down);
			printf("\n Result_1 \tUp= %6.3f ns / Down= %6.3f ns", Result_1_up,
					Result_1_down);
			printf("\n Result_2 \tUp= %6.3f ns / Down= %6.3f ns", Result_2_up,
					Result_2_down);
			printf("\n Result_3 \tUp= %6.3f ns / Down= %6.3f ns", TOF_up,
					TOF_down);

			Time_of_flight_diff = TOF_up - TOF_down;
			printf("\n\tDifference RES_3(Up-Down)= %6.3f ns",
					Time_of_flight_diff);

			Time_of_flight_sum = TOF_up + TOF_down;
			printf("\n\tSum        RES_3(Up+Down)= %6.3f ns\n",
					Time_of_flight_sum);

			// to add up
			TOF_diff_sum += Time_of_flight_diff;
			TOF_diff_square_sum = TOF_diff_square_sum
					+ (Time_of_flight_diff * Time_of_flight_diff);
			TOF_sum_sum += Time_of_flight_sum;
			sum_counter++;

			if (sum_counter > no_of_avg) // Output after no_of_avg measurements
					{
				TOF_diff_avg = TOF_diff_sum / no_of_avg;
				printf("\n  Avg. value of difference = %6.3f ns", TOF_diff_avg);
				Std_Dev_of_Diff =
						sqrt(
								(TOF_diff_square_sum
										- (TOF_diff_square_sum / no_of_avg))
										/ (no_of_avg - 1));
				printf("\n  Std.Dev. of Diff. = %6.1f ps",
						Std_Dev_of_Diff * 1000);
				printf("\n  PW1ST = %1.2f\n", PW1ST / (no_of_avg));

				Std_Dev_of_Diff = 0;
				TOF_diff_sum = 0;
				TOF_diff_square_sum = 0;
				sum_counter = 1;
				PW1ST = 0;
			}
		}

		Dummy_var++; // To Control the loop
	} // End while Dummy_var

} //End main

/*******************************************************************************
 * Device Functions
 ******************************************************************************/

/*******************************************************************************
 * Function Name: gp22_send_1byte
 * Parameters: Opcode byte
 *
 * Return: none
 *
 * Description: Writes the Opcode to GP21
 *
 ******************************************************************************/
void gp22_send_1byte(void *bus_type, uint8_t gp22_opcode_byte) {
	// Deactivating Reset SPIx
	if (bus_type == SPI1)
		GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);
	if (bus_type == SPI2)
		GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET);

	while (SPI_I2S_GetFlagStatus(bus_type, SPI_I2S_FLAG_TXE) == 0) {
	}
	SPI_I2S_SendData(bus_type, gp22_opcode_byte);     // OPCODE TO Device
	while (SPI_I2S_GetFlagStatus(bus_type, SPI_I2S_FLAG_TXE) == 0) {
	}
	Simple_delay_750ns((void*) 10); // important delay (16) at SPI freq.=750kHz

	// Reset to device SPIx
	if (bus_type == SPI1)
		GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
	if (bus_type == SPI2)
		GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_SET);
}

/*******************************************************************************
 * Function Name: gp22_wr_config_reg
 * Parameters: Address byte, 4 bytes of Configuration
 *
 * Return: none
 *
 * Description: Writes the config.reg. specified in GP21 with the data
 *
 ******************************************************************************/
void gp22_wr_config_reg(void *bus_type, uint8_t opcode_address,
		uint32_t config_reg_data) {
	uint8_t Data_Byte_Lo = config_reg_data;
	uint8_t Data_Byte_Mid1 = config_reg_data >> 8;
	uint8_t Data_Byte_Mid2 = config_reg_data >> 16;
	uint8_t Data_Byte_Hi = config_reg_data >> 24;

	uint8_t common_delay = 10; // important delay (16) at SPI freq.=750kHz

	// Deactivating Reset SPIx
	if (bus_type == SPI1)
		GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);
	if (bus_type == SPI2)
		GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET);

	while (SPI_I2S_GetFlagStatus(bus_type, SPI_I2S_FLAG_TXE) == 0) {
	}

	SPI_I2S_SendData(bus_type, opcode_address);  // RAM WR OPCODE+ADDRESS
	Simple_delay_750ns((void*) common_delay);

	while (SPI_I2S_GetFlagStatus(bus_type, SPI_I2S_FLAG_TXE) == 0) {
	}
	SPI_I2S_SendData(bus_type, Data_Byte_Hi);  // DATA BYTE HIGH
	Simple_delay_750ns((void*) common_delay);

	while (SPI_I2S_GetFlagStatus(bus_type, SPI_I2S_FLAG_TXE) == 0) {
	}
	SPI_I2S_SendData(bus_type, Data_Byte_Mid2);  // DATA MID - 2
	Simple_delay_750ns((void*) common_delay);

	while (SPI_I2S_GetFlagStatus(bus_type, SPI_I2S_FLAG_TXE) == 0) {
	}
	SPI_I2S_SendData(bus_type, Data_Byte_Mid1);  // DATA MID - 1
	Simple_delay_750ns((void*) common_delay);

	while (SPI_I2S_GetFlagStatus(bus_type, SPI_I2S_FLAG_TXE) == 0) {
	}
	SPI_I2S_SendData(bus_type, Data_Byte_Lo);  // DATA LOW
	Simple_delay_750ns((void*) common_delay);

	while (SPI_I2S_GetFlagStatus(bus_type, SPI_I2S_FLAG_TXE) == 0) {
	}
	Simple_delay_750ns((void*) common_delay);

	// Reset to device SPIx
	if (bus_type == SPI1)
		GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
	if (bus_type == SPI2)
		GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_SET);
}

/*******************************************************************************
 * Function Name: gp22_read_n_bytes
 * Parameters: bus_type = (SPI1, SPI2)
 *             n_bytes = how many bytes should be read
 *             read_opcode = read opcode of the device
 *             read_addr = read address of the device
 *             fractional_bits = number of fractional bits of read data
 *
 * Return: n bytes from the specified read address
 *
 * Description: Reads n bytes from an address in GP21
 *
 ******************************************************************************/
float gp22_read_n_bytes(void *bus_type, uint8_t n_bytes, uint8_t read_opcode,
		uint8_t read_addr, uint8_t fractional_bits) {
	uint32_t Result_read = 0;
	float Result = 0;
	uint8_t read_opcode_addr = read_opcode | read_addr;

	//.............. Result = n Byte = n x 8 bits......................
	if (bus_type == SPI1 | bus_type == SPI2) {
		// Deactivating Reset SPIx
		if (bus_type == SPI1)
			GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);
		if (bus_type == SPI2)
			GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET);

		SPI_I2S_SendData(bus_type, read_opcode_addr);  // READ OPCODE + Address

		while (SPI_I2S_GetFlagStatus(bus_type, SPI_I2S_FLAG_TXE) == RESET) {
		};
		Simple_delay_750ns((void*) 10); // important delay (16) at SPI freq.=750kHz

		//Compulsory reads to DR and SR to clear OVR,
		//so that next incoming data is saved
		SPI_I2S_ReceiveData(bus_type);                     // To clear OVR
		SPI_I2S_GetFlagStatus(bus_type, SPI_I2S_FLAG_TXE); // To clear OVR

		//Reading byte1
		SPI_I2S_SendData(bus_type, 0x00FF);  // DUMMY WRITE
		// Wait until RX buffer is not empty, then read the received data
		while (SPI_I2S_GetFlagStatus(bus_type, SPI_I2S_FLAG_RXNE) == 0) {
		}
		Result_read = SPI_I2S_ReceiveData(bus_type); //  Read

		for (int n = 1; n < n_bytes; n++) {
			//Reading byte2 .. byte.n
			SPI_I2S_SendData(bus_type, 0x00FF);  // DUMMY WRITE
			// Wait until RX buffer is not empty, then read the received data
			while (SPI_I2S_GetFlagStatus(bus_type, SPI_I2S_FLAG_RXNE) == 0) {
			}

			Result_read = Result_read << 8;
			Result_read |= SPI_I2S_ReceiveData(bus_type); //  Read
		}

		// Reset to device SPIx
		if (bus_type == SPI1)
			GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
		if (bus_type == SPI2)
			GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_SET);
	}

	Result = Result_read / pow(2, fractional_bits);

	return Result;
}

/*******************************************************************************
 * Function Name: gp22_read_status_bytes
 * Parameters: bus_type = (SPI1, SPI2)
 *
 * Return: 2 bytes from the status register, address 0x04
 *
 * Description: Reads 2 bytes from an address in GP22
 *
 ******************************************************************************/
uint16_t gp22_read_status_bytes(void *bus_type) {
	uint16_t Result_read = 0;
	uint8_t n_bytes = 2;

	uint8_t read_opcode_addr = 0xB0 | 0x04;

	//.............. Result = 2 Byte = 16 bits......................
	if (bus_type == SPI1 | bus_type == SPI2) {
		// SSN High->Low; Select SPIx device
		if (bus_type == SPI1)
			GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);
		if (bus_type == SPI2)
			GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET);

		SPI_I2S_SendData(bus_type, read_opcode_addr);  // READ OPCODE + Address

		while (SPI_I2S_GetFlagStatus(bus_type, SPI_I2S_FLAG_TXE) == RESET) {
		};
		Simple_delay_750ns((void*) 10); // important delay (16) at SPI freq.=750kHz

		//Compulsory reads to DR and SR to clear OVR,
		//so that next incoming data is saved
		SPI_I2S_ReceiveData(bus_type);                     // To clear OVR
		SPI_I2S_GetFlagStatus(bus_type, SPI_I2S_FLAG_TXE); // To clear OVR

		//Reading byte1
		SPI_I2S_SendData(bus_type, 0x00FF);  // DUMMY WRITE
		// Wait until RX buffer is not empty, then read the received data
		while (SPI_I2S_GetFlagStatus(bus_type, SPI_I2S_FLAG_RXNE) == 0) {
		}
		Result_read = SPI_I2S_ReceiveData(bus_type); //  Read

		for (int n = 1; n < n_bytes; n++) {
			//Reading byte2 .. byte.n
			SPI_I2S_SendData(bus_type, 0x00FF);  // DUMMY WRITE
			// Wait until RX buffer is not empty, then read the received data
			while (SPI_I2S_GetFlagStatus(bus_type, SPI_I2S_FLAG_RXNE) == 0) {
			}

			Result_read = Result_read << 8;
			Result_read |= SPI_I2S_ReceiveData(bus_type); //  Read
		}

		// SSN Low->High; Reset SPIx
		if (bus_type == SPI1)
			GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
		if (bus_type == SPI2)
			GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_SET);
	}

	return Result_read;
}

/*******************************************************************************
 * Function Name: gp22_status_count_error
 * Parameters: bus_type = (SPI1, SPI2)
 *
 * Return: only one Error Bit (9..15) from the status register, address 0x04
 *
 * Description: Reads 2 bytes from an address in GP22
 ******************************************************************************/
uint8_t gp22_status_count_error(void *bus_type) {
	uint16_t STAT_REG = 0x0000;
	uint8_t count_error = 0;

	STAT_REG = gp22_read_status_bytes(bus_type);

	if ((STAT_REG & 0x0200) == 0x0200)
		count_error++; //Bit9: Timeout_TDC
	if ((STAT_REG & 0x0400) == 0x0400)
		count_error++; //Bit10: Timeout_Precounter
	if ((STAT_REG & 0x0800) == 0x0800)
		count_error++; //Bit11: Error_open
	if ((STAT_REG & 0x1000) == 0x1000)
		count_error++; //Bit12: Error_short
	if ((STAT_REG & 0x2000) == 0x2000)
		count_error++; //Bit13: EEPROM_eq_CREG
	if ((STAT_REG & 0x4000) == 0x4000)
		count_error++; //Bit14: EEPROM_DED
	if ((STAT_REG & 0x8000) == 0x8000)
		count_error++; //Bit15: EEPROM_Error

	return count_error;
}

/*******************************************************************************
 * Function Name: gp22_analyse_error_bit
 * Parameters: none
 *
 * Return: none
 *
 * Description: Analyse the bit from the status register
 ******************************************************************************/
void gp22_analyse_error_bit(void *bus_type) {
	uint16_t STAT_REG = 0x0000;

	STAT_REG = gp22_read_status_bytes(bus_type);

	//Bit9: Timeout_TDC
	if ((STAT_REG & 0x0200) == 0x0200)
		printf("\n-Indicates an overflow of the TDC unit");
	//Bit10: Timeout_Precounter
	if ((STAT_REG & 0x0400) == 0x0400)
		printf("\n-Indicates an overflow of the 14 bit precounter in MR 2");
	//Bit11: Error_open
	if ((STAT_REG & 0x0800) == 0x0800)
		printf("\n-Indicates an open sensor at temperature measurement");
	//Bit12: Error_short
	if ((STAT_REG & 0x1000) == 0x1000)
		printf("\n-Indicates a shorted sensor at temperature measurement");
	//Bit13: EEPROM_eq_CREG
	if ((STAT_REG & 0x2000) == 0x2000)
		printf(
				"\n-Indicates whether the content of the configuration registers equals the EEPROM");
	//Bit14: EEPROM_DED
	if ((STAT_REG & 0x4000) == 0x4000)
		printf(
				"\n-Double error detection. A multiple error has been detected whcich can not be corrected.");
	//Bit15: EEPROM_Error
	if ((STAT_REG & 0x8000) == 0x8000)
		printf("\n-Single error in EEPROM which has been corrected");
}

/*******************************************************************************
 * Bus Functions
 ******************************************************************************/

/*******************************************************************************
 * Function Name: SPIx_GPIOs_Init
 * Parameters: Int32U Clk, Int32U Width
 * Return: none
 *
 * Description: Init GPIOs used in SPIx interface
 *
 ******************************************************************************/
void SPIx_GPIOs_Init(void* bus_type) {
	GPIO_InitTypeDef GPIO_InitStructure; // GPIO_InitTypeDef defined in library

	// Enable GPIO clock and release reset
	RCC_APB2PeriphResetCmd(
			RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC
					| RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE
					| RCC_APB2Periph_AFIO, DISABLE);
	RCC_APB2PeriphClockCmd(
			RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC
					| RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE
					| RCC_APB2Periph_AFIO, ENABLE);

	// Configure   SPI1_CLK  - PA5
	//             SPI1_MOSI - PA7
	//             SPI1_MISO - PA6
	// Chip select SPI1_NSS  - PA4
	// External Interrupt Input line PD4

	// Configure   SPI2_CLK  - PB13
	//             SPI2_MOSI - PB15
	//             SPI2_MISO - PB14
	// Chip select SPI2_NSS  - PB12
	// External Interrupt Input line PE11

// SPI1_NSS
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	if (bus_type == SPI1) {
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	} // SPI1 - PA4
	  //  SPI1_CLK
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	if (bus_type == SPI1) {
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	} // SPI1 - PA5
	  //   SPI1_MISO
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	if (bus_type == SPI1) {
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	} // SPI1 - PA6
	  //   SPI1_MOSI
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	if (bus_type == SPI1) {
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	} // SPI1 - PA7

	if (bus_type == SPI2) {
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	} // SPI2 - PB12

	if (bus_type == SPI2) {
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	} // SPI2 - PB13

	if (bus_type == SPI2) {
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	} // SPI2 - PB14

	if (bus_type == SPI2) {
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	} // SPI1 - PB15

	SPI_I2S_DeInit(bus_type);

// External Interrupt Input
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	if (bus_type == SPI1) {
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
		GPIO_Init(GPIOD, &GPIO_InitStructure);
	} // SPI1 - PD4
	if (bus_type == SPI2) {
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
		GPIO_Init(GPIOE, &GPIO_InitStructure);
	} // SPI2 - PE11

// SPI ENABLE Output for the evaluation kit
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	if (bus_type == SPI1) {
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
		GPIO_Init(GPIOD, &GPIO_InitStructure);
	} // SPI1 - PD3
	if (bus_type == SPI2) {
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_Init(GPIOE, &GPIO_InitStructure);
	} // SPI2 - PE10

	// SPIx Enable = RSN for GP22
	if (bus_type == SPI1)
		GPIO_WriteBit(GPIOD, GPIO_Pin_3, Bit_SET); // SPI1 - PD3
	if (bus_type == SPI2)
		GPIO_WriteBit(GPIOE, GPIO_Pin_10, Bit_SET); // SPI2 - PE10
}

/*******************************************************************************
 * Function Name: SPIx_Interface_Init
 * Parameters: Int32U Clk, Int32U Width
 * Return: none
 *
 * Description: Init SPI1 or SPI2 Interface
 *
 ******************************************************************************/
void SPIx_Interface_Init(void* bus_type) {
	// Initialising the SPIx interface
	SPI_InitTypeDef SPI_InitStructure;

	/* Configures the system clock (SYSCLK) */
	RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI); // Source-freq. 8.000MHz
	//RCC_SYSCLKConfig (RCC_SYSCLKSource_HSE); // Source-freq. 20.000MHz
	//RCC_SYSCLKConfig (RCC_SYSCLKSource_PLLCLK); // Source-freq. 57.6MHz ( (72MHz/25MHz) * HSE) )

	/* Adjusts the Internal High Speed oscillator (HSI) calibration value.
	 * @param  HSICalibrationValue: specifies the calibration trimming value.
	 *   This parameter must be a number between 0 and 0x1F. */
	//RCC_AdjustHSICalibrationValue(0x10); //0x00..0x0F // 3.8..4.2MHZ
	/* Configures the AHB clock (HCLK) */
	RCC_HCLKConfig(RCC_SYSCLK_Div1);

	// Clock Enable and Reset release
	if (bus_type == SPI1) {
		RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, DISABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	}

	if (bus_type == SPI2) {
		RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2, DISABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
		RCC_PCLK1Config(RCC_HCLK_Div1); // in order to adapt the clock frequenz
	}

	// All are defined in stm32f10x_spi.h
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	// SPI frequence devider
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;

	SPI_Init(bus_type, &SPI_InitStructure);
	SPI_Cmd(bus_type, ENABLE); // Enabling the SPIx Interface

	// Enabling the NSS Output during transmission
	SPI_SSOutputCmd(bus_type, ENABLE);
	// SPIx - SSN to Device - Set to High for reset
	if (bus_type == SPI1)
		GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
	if (bus_type == SPI2)
		GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_SET);
}

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

/*******************************************************************************
 * Function Name: Dly100us, Dly250ns, Dly1ms, Simple_delay_750ns
 * Parameters: delay multiplier
 *
 * Return: none
 *
 * Description: Delay Dly * (100us, 250ns, 1ms, 750ns)
 *
 ******************************************************************************/
void Dly100us(void *arg)                        // Gives 100us delay with arg 1
{
	uint32_t Dely = (uint32_t) arg;
	while (Dely--) {
		for (int i = LOOP_DLY_100US; i; i--)
			;
	}
}

void Dly250ns(void *arg)                        // Gives 250ns delay with arg 1
{
	uint32_t Dely = (uint32_t) arg;
	while (Dely--) {
		for (int i = LOOP_DLY_250ns; i; i--)
			;
	}
}

void Dly1ms(void *arg)                            // Gives 1ms delay with arg 1
{
	uint32_t Dely = (uint32_t) arg;
	while (Dely--) {
		for (int i = LOOP_DLY_1ms; i; i--)
			;
	}
}

void Simple_delay_750ns(void *arg)             // Gives 750ns delay, with arg 1
{
	uint32_t Dely = (uint32_t) arg;
	for (int i = Dely; (i != 0); i--)
		;

}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
/******************* (C) COPYRIGHT 2012 acam messelectronic GmbH **************/
