/*!
 * @file phy.h
 *
 *	(EN 13757-2) protocol standard-based physical and link-layer implementation prototypes.
 *
 *  @date: Oct 3, 2020
 *
 *  @author: Deksios Bekele
 */

#ifndef COMM_PHY_H_
#define COMM_PHY_H_

#include <stdint.h>
#include <math.h>
#include <msp430.h>
#include "../config/system.h"

/*
 * EN-13757-2 Link Layer Requirements:
 *   - 300 Baud should be supported(default)
 *   - 2400, 9600, 19200 are recommended
 *   - Special buad rates of 600, 1200, 4800 or 38400 could be used
 *   - Current baud rate shall be kept after reset
 */
//------------------------------------------------------------------------------
// FRAME types
//
#define MBUS_FRAME_TYPE_ANY  0x00
#define MBUS_FRAME_TYPE_ACK  0x01
#define MBUS_FRAME_TYPE_SHORT  0x02
#define MBUS_FRAME_TYPE_CONTROL  0x03
#define MBUS_FRAME_TYPE_LONG  0x04

#define MBUS_FRAME_ACK_BASE_SIZE  1
#define MBUS_FRAME_SHORT_BASE_SIZE  5
#define MBUS_FRAME_CONTROL_BASE_SIZE  9
#define MBUS_FRAME_LONG_BASE_SIZE  9

#define MBUS_FRAME_BASE_SIZE_ACK  1
#define MBUS_FRAME_BASE_SIZE_SHORT  5
#define MBUS_FRAME_BASE_SIZE_CONTROL  9
#define MBUS_FRAME_BASE_SIZE_LONG  9

#define MBUS_FRAME_FIXED_SIZE_ACK  1
#define MBUS_FRAME_FIXED_SIZE_SHORT  5
#define MBUS_FRAME_FIXED_SIZE_CONTROL  6
#define MBUS_FRAME_FIXED_SIZE_LONG  6

// Frame start/stop bits
#define MBUS_FRAME_SHORT_START  0x10
#define MBUS_FRAME_CONTROL_OR_LONG_START  0x68
#define MBUS_FRAME_STOP  0x16

#define MBUS_FRAME_CONTROL_SIZE 0x03

#define MBUS_FRAME_MAX_USER_DATA_SIZE 0xF6

// Data information field definitions for possible values
//0. Instantenous values with 0 storage and no extension

#define NO_DATA 0
#define INST_8_BIT_INTEGER  0x01
#define INST_16_BIT_INTEGER 0x02
#define INST_24_BIT_INTEGER 0x03
#define INST_32_BIT_INTEGER 0x04
#define INST_48_BIT_INTEGER 0x06
#define INST_64_BIT_INTEGER 0x07
#define INST_2_DIGIT_BCD 0x09
#define INST_4_DIGIT_BCD 0x0A
#define INST_6_DIGIT_BCD 0x0B
#define INST_8_DIGIT_BCD 0x0C
#define INST_VARIABLE_LENGTH 0x0D
#define INST_12_DIGIT_BCD 0x0E
#define INST_SELECTION_FOR_READOUT 0x08

#define MANUFACTURER_SPECIFIC_DATA 0x0F
#define MANUAFACRURER_SPECIFIC_DATA_EXT 0x1F
#define IDLE_FILLER 0x2F
#define GLOBAL_READOUT_REQUEST 0x7F  // readout all storage, units, tariffs and function fieds

#define BROADCAST_NO_REPLY 		0xFF
/*!-- can result in collisions during reply and should only be used for test purposes*/
#define BROADCAST_WITH_REPLY 	0xFE
/*!-- The address 253 (FDh) indicates that the adressing has been performed in the Network Layer instead of Data Link Layer*/
#define BROADCAST_NETWORK_LAYER 	0xFD
/*!--
 * 		EN 13753-2/ IEC 870-5 Telegram formats
 *
 * 		1. Single Character
 * 		--------
 * 		| 0xE5 |
 * 		--------
 * 		2. Short Frame
 * 		---------------------------------------------------------
 * 		| Start 0x10 | C Field | A Field | Checksum | Stop 0x16 |
 * 		---------------------------------------------------------
 * 		3. Control Frame
 * 		--------------------------------------------------------------------------------------------------------------
 * 		| Start 0x68  | L Field = 3 | L Field = 3 | Start 0x68 | C Field | A Field | CI Field | Checksum | Stop 0x16 |
 * 		--------------------------------------------------------------------------------------------------------------
 * 		4. Long Frame
 * 		-------------------------------------------------------------------------------------------------------------------------------
 * 		| Start 0x68 | L Field | L Field | Start 0x68 | C Field | A Field | CI Field | User Data (0-252 Bytes) | Checksum | Stop 0x16 |
 * 		-------------------------------------------------------------------------------------------------------------------------------
 *
 * */

/*!-- Single digit character acknowledgment from the slave */

#define ACKNOWLEDGMENT (uint8_t)0xE5

/*!-- C Field definitions */

/*!--  	C Field Bits
 * 		-------------------------------------------------------------------------------------------------------------------------------
 * 		| Bit Number |  7  |  6  |  5   |  4  | 3  |  2  | 1  | 0  |
 * 		-------------------------------------------------------------------------------------------------------------------------------
 * 		| Calling 	 |  0  |  1  |  FCB | FCV | F3 |  F2 | F1 | F0 |
 * 		-------------------------------------------------------------------------------------------------------------------------------
 * 		| Reply 	 |  0  |  0  |  ACD | DFC | F3 |  F2 | F1 | F0 |
 * 		-------------------------------------------------------------------------------------------------------------------------------
 *
 *    	Name 		|	C Field 	|	C Field Hex. |	Telegram 			|	Description
 *    				|	Binary      |                |                      |
 *		-----------------------------------------------------------------------------------------------------------------------------------
 *		SND_NKE 	|	0100 0000 	|	40 			 |	Short Frame 		|	Initialization of Slave
 *		SND_UD  	|	01F1 0011 	|	53/73 		 |	Long/Control Frame	|	Send User Data to Slave
 *		REQ_UD2 	|	01F1 1011 	|	5B/7B 		 |	Short	Frame 		|	Request for Class 2 Data
 *		REQ_UD1 	| 	01F1 1010 	|	5A/7A 		 |	Short Frame 		|	Request for Class1 Data (see 8.1: Alarm Protocol)
 *		RSP_UD  	|	00AD 1000 	|	08/18/28/38  |	Long/Control Frame 	|	Data Transfer from Slave to Master after Request
 *
 * */
/*!--
 * The Data Link Layer uses two kinds of transmission services:
 • Send/Confirm : SND/CON
 • Request/Respond : REQ/RSP

 --> After the reception off  a valid telegram the slave has to wait
 between 11 bit times and (330 bit times + 50ms) before answering (see also EN1434-3).
 *
 * */
#define MAX_INTERBYTE_GAP(x)   (uint16_t)ceil(2750/x) /*!-- In milliseconds -- 8 KHz clock
									Any gap of between bytes of
									greater than 22 bit times shall
									be considered as the end of a datagram*/

#define MIN_REQ_RES_DELAY(x)      (uint16_t)ceil(1375/x) /*!-- In milliseconds -- 8 KHz clock
										This is required to clearly distinguish
 	 	 	 	 	 	 	 	 	 between a true isolated datagram and a section
 	 	 	 	 	 	 	 	 	 of a longer datagram */

#define MAX_REQ_RES_DELAY(x)      (uint16_t)ceil((41250/x)+50) /*!-- In milliseconds --- 8 KHz clock
											This is required to clearly distinguish
 	 	 	 	 	 	 	 	 	 	 	 between a true isolated datagram and a section
 	 	 	 	 	 	 	 	 	 	 	 of a longer datagram */
#define AUTO_BAUD_RESET_DELAY	  5000 /*!-- The recommended values are 2-10 mins*/

#define _USER_INT_

#ifdef _USER_INT_
#define DISABLE_UX_INT    (UCA1IE &= ~UCTXIE)
#define ENABLE_UX_INT    (UCA1IE |= UCTXIE)
#endif

#define TRUE_PA  (int8_t)0
#define TRUE_SA (int8_t)1
#define FALSE_NA (int8_t)-1

typedef enum {
	THREE_HUNDERED = 300,
	SIX_HUNDRED = 600,
	TWELVE_HUNDRED = 1200,
	TWENTY_FOUR_HUNDRED = 2400,
	FOURTY_EIGHT_HUNDRED = 4800,
	NINETY_SIX_HUNDRED = 9600,
	NINETEEN_THOUSAND_TWO_HUNDRED = 19200,
	THIRTY_EIGHT_THOUSAND_FOUR_HUNDRED = 38400
} working_baud_rates;

typedef struct {
	uint8_t primaryAddress;
	uint32_t serialNumber;
	uint32_t productNumber;
	uint32_t productDate;
} deviceInfo;

extern deviceInfo devInfo;
/*!-- Default primary address of all slaves at manufacture is 0.
 *  Note: This value can be overwritten(changed) with an application level
 *   master command. */
extern volatile uint8_t PRIMARY_ADDRESS;
/*	Serial Number (4-Bytes)
 * 	Example: SN: 30001234
 * 	0x34 0x12 0x00 0x30 Little endian
 */
extern volatile uint32_t SERIAL_NUMBER;

typedef enum {
	error, valid
} cs_result;

typedef enum {
	waiting_for_master, waiting_for_response, response_ready
} snd_req_status;

typedef enum {
	ack, short_frame, control_frame, long_frame
} frame_type;

typedef struct {

	uint8_t start;
	uint8_t c_field;
	uint8_t a_field;
	uint8_t l_fieldH;
	uint8_t l_fieldL;
	uint8_t startR;
	uint8_t ci_field; // EN 13757-3
	uint8_t * user_data;
	uint8_t checksum;
	uint8_t stop;
	frame_type type;
} lnk_frame;

extern volatile uint16_t current_baud;
extern volatile uint8_t tx_buff[];
extern volatile snd_req_status c_status;
extern volatile lnk_frame *r_frame;
extern volatile lnk_frame *rsp_frame;

void ux_config(uint16_t baudRate);
void inline rx_lnk_frame(uint8_t data);
void tx_lnk_frame(uint8_t length);
void x_lnk_res(void);
void set_baudRate(uint16_t baudRate);
void inline x_reset();
cs_result inline t_validator();
uint8_t inline getCheksum(volatile lnk_frame *frame);
void inline res_delay();
void inline inter_byte_time_out();
void inline autospeed_detect();

/*!-- check addressing type */
int8_t inline c_addressing() {
	uint8_t PA = r_frame->a_field;
	uint64_t SA;
	uint8_t sb_c = 0;

	/*Ensure data integrity*/
	__ATOMIZE();
	if (PA != BROADCAST_NO_REPLY) {
		if (PA == BROADCAST_NETWORK_LAYER) {
			for (; sb_c < sizeof(uint32_t); sb_c++) {
				SA |= *r_frame->user_data;
				SA <<= 8;
				r_frame->user_data++;
			}
			__END_ATOMIC();
			return SA == devInfo.serialNumber ? TRUE_SA : FALSE_NA;
		}
		__END_ATOMIC();
		return PA == devInfo.primaryAddress ? TRUE_PA : FALSE_NA;
	}
	__END_ATOMIC();
	return FALSE_NA;
}

#endif /* COMM_PHY_H_ */
