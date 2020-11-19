/*!
 * @file app.h
 *
 *	(EN 13757-3) protocol standard-based applicaion layer implementation prototypes.
 *
 *  @date: Oct 3, 2020
 *
 *  @author: Deksios Bekele
 */

#ifndef COMM_APP_H_
#define COMM_APP_H_

#include <stdint.h>
#include <string.h>
#include "phy.h"

//API CI-field definitions for EN13757-3
//#ifndef Default_settings
//

/*define message type*/

//#define ACKNOWLEDGMENT 0
//#define DATA	1
//#define ERROR 2
//#endif
/*!-- Note that even if the functions of these optional CI-codes are not
 * implemented in a slave, the link layer protocol requires a proper link
 * layer acknowledge of SND_UD telegrams containing any of these CI-codes
 * */
#define APPLICATION_RESET 0x50
#define SEND_DATA_MASTER_TO_SLAVE 0x51  // meter is assumed to be slave (master to slave)
#define SELECTION_OF_SLAVES 0x52  // master is responsible for selection of slaves using primary/secondary addresses
#define SYNCHRONIZE_ACTION 0x5C // example - clock
#define SLAVE_TO_MASTER_APPLICATION_ERRORS 0x70
#define SLAVE_TO_MASTER_REPORT_OF_ALARMS  0x71
#define SLAVE_TO_MASTER_FULL_HEADER 0x72  // more ideal for radio based communications
#define SLAVE_TO_MASTER_WITHOUT_HEADER 0x78
#define SLAVE_TO_MASTER_SHORT_HEADER 0x7A  // the link layer has the 8bytes of address, no need for them in the application layer
#define SET_BAUD_300   0xB8
#define SET_BAUD_600   0xB9
#define SET_BAUD_1200  0xBA
#define SET_BAUD_2400  0xBB
#define SET_BAUD_4800  0xBC
#define SET_BAUD_9600  0xBD
#define SET_BAUD_19200 0xBE
#define SET_BAUD_38400 0xBF

// function field of the data field specifies the type of data

/*!\enum function_field
 *
 * \brief Definitions for the functions field of the DIF byte as per the EN13757 standard :
 *
 * */

typedef enum {
	INSTANTENOUS = 0, MAXIMUM = 1, MINIMUM = 2, ERROR_VALUE = 3

} function_field;

//#define  SLAVE_ACKNOWLEDGEMENT 0xE5 // the proper reception of a telegram is acknowledged by a slave with this byte

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
//#define SPECEIAL_FUNCTIONS

// Data information field definitions for possible values
//1. Maximum values with 0 storage and no extension

#define MAX_8_BIT_INTEGER 0x11
#define MAX_16_BIT_INTEGER 0x12
#define MAX_24_BIT_INTEGER 0x13
#define MAX_32_BIT_INTEGER 0x14
#define MAX_48_BIT_INTEGER 0x16
#define MAX_64_BIT_INTEGER 0x17
#define MAX_2_DIGIT_BCD 0x19
#define MAX_4_DIGIT_BCD 0x1A
#define MAX_6_DIGIT_BCD 0x1B
#define MAX_VARIABLE_LENGTH 0x1C
#define MAX_SELECTION_FOR_READOUT 0x18

// Data field definitions for possible values
//2. Minimum values with 0 storage and no extension
#define MIN_8_BIT_INTEGER 0x21
#define MIN_16_BIT_INTEGER 0x22
#define MIN_24_BIT_INTEGER 0x23
#define MIN_32_BIT_INTEGER 0x24
#define MIN_48_BIT_INTEGER 0x26
#define MIN_64_BIT_INTEGER 0x27
#define MIN_2_DIGIT_BCD 0x29
#define MIN_4_DIGIT_BCD 0x2A
#define MIN_6_DIGIT_BCD 0x2B
#define MIN_VARIABLE_LENGTH 0x2C
#define MIN_SELECTION_FOR_READOUT 0x28

// Data field definitions for possible values
//3. Error state values with 0 storage and no extension

#define ERR_2_DIGIT_BCD 0x39
#define ERR_4_DIGIT_BCD 0x3A
#define ERR_6_DIGIT_BCD 0x3B
#define ERR_VARIABLE_LENGTH 0x3C
//#define ERR_SEL_FOR_READOUT 0x308   /*! ERR_SELECTION_FOR_READOUT: Case label value exceeds maximum value for type */

#define MANUFACTURER_SPECIFIC_DATA 0x0F
#define MANUAFACRURER_SPECIFIC_DATA_EXT 0x1F
#define IDLE_FILLER 0x2F
#define GLOBAL_READOUT_REQUEST 0x7F  // readout all storage, units, tariffs and function fieds

// VIF definitions

#define VIF_SET_PRIMARY_ADDRESS 0x7A
#define VIF_SET_SECONDARY_ADDRESS 0x79


#define VOLUME_IN_LITER 0x13

#define TIME_IN_SECONDS 0x20  //  on time
#define TIME_IN_MINUTES 0x21  //  on time
#define TIME_IN_HOURS   0x22  //  on time
#define TIME_IN_DAYS    0x23  //  on time

#define TIME_POINT_IN_DATE 0x6C // DATA_TYPE G  // 5 bytes
#define TIME_POINT_IN_TIME_AND_DATE 0x67 // DATA TYPE F , 8 bytes

#define ANY_VIF_READOUT 0xFE // 0X7E
#define ANY_VIF_READOUT_1 0x7E // 0X7E
#define ALL_STORAGE_NUMBERS 0xFE // 0X7E
#define ANY_VIF_READOUT 0xFE // 0X7E

#define VIF_FOR_STRING  0xFC // 0X7C
#define VIF_EXTENSION_WITH_TRUE_1ST_VIFE_0 0xFB
#define VIF_EXTENSION_WITH_TRUE_1ST_VIFE_1 0xFD
#define MANUFACTURER_SPECIFIC_0 0x7F //
#define MANUFACTURER_SPECIFIC_1 0xFF // 0X7F

// VIFE definitions for VIF= FD(primary VIF)-> true value is given in the first VIFE

#define ACCESS_NUMBER 0X08
#define HEAT_METER 0X04  // device type => heat meter/cost allocator
#define MANUFACTURER 0X0A
#define HARDWARE_VERSION 0X0D
#define SOFTWARE_VERSION 0X0E  // firmware/meterology version
#define PASSWORD 0X16
#define BAUDRATE 0X1C //
#define DAY_OF_WEEK 0X63
#define WEEK_NUMBER 0X64
#define OPERATING_TIME_OF_BATTERY_HOUR 0X6C
#define OPERATING_TIME_OF_BATTERY_DAY 0X6D
#define OPERATING_TIME_OF_BATTERY_MONTH 0X6E
#define OPERATING_TIME_OF_BATTERY_YEAR 0X6F
#define DATE_AND_TIME_OF_BATTERY_CHANGE 0X70

// VIFE definitions for VIF= FD(primary VIF)-> true value is given in the first VIFE

#define PER_SECOND 0X20
#define PER_MINUTE 0X21
#define PER_HOUR   0X22
#define PER_DAY    0X23
#define PER_WEEK   0X24
#define PER_MONTH  0x25
#define PER_YEAR   0x26
#define PER_LITER  0x2C

/*!-- Application Reset Subcodes 8 bit binary (upper 4 bit values listed here)
 *  -- maybe ignored for slaves with one type of telegram
 */
#define ALL 0
#define USER_DATA 1
#define SIMPLE_BILLING 2
#define ENHANCED_BILLING 3
#define MULTI_TARIFF_BILLING 4
#define INSTANTANEOUS_VALUES 5
#define LOAD_MANAGEMENT_VALUES_FOR_MANAGEMENT 6
#define INSTALLATION_AND_STARTUP 8
#define TESTING 9
#define CALIBRATION 10
#define MANUFACTURING 11
#define DEVELOPMENT 12
#define SELFTEST 13

/*
 * Manufacturer specific definitions
 * These are not included in the standard but added by the manufacturer
 *
 */
//custom VIFE definitions following VIF = 0x7F/0xFF

#define START_END_TIME

/* Response of the meter */
//extern WMBUSMessage WMBUSResponse;
/*!\struct variable_data
 *
 * \brief Framing for the application layer variable data record included in the packet
 *  sent from the slave to the master.
 *
 * */

typedef struct {
	uint8_t data_information; //! 1 byte the data information field
	uint8_t data_information_ext[10]; //! byte array for the data information extension field
	uint8_t value_information; //! 1 byte value informaion field
	uint8_t value_information_ext[10]; //! byte array for  the value information extension field.

} variable_data;

/*!\struct slave_message_header
 *
 * \brief  Framing for application layer header of the packet sent from the slave to the master.
 *
 * */

typedef struct

{
	uint8_t access_number; //! the number of accesses to the meter (slave)
	uint8_t status_number; //! status number
	uint8_t signature_low; //! signature field lower byte
	uint8_t signature_high; //! signature field higher byte

} slave_message_header;

/*!\struct meter_config_info
 *
 * \brief  Meter configuration information (metering + wireless module).
 *
 * */

/*!\enum   serial number
 *
 * \brief  Meter serial number.
 *
 * */

typedef struct {
	char manufacturer_init[3];
	unsigned long unique_number;

} serial_num;

/*!\enum   communication_modes
 *
 * \brief  WMBUS communication modes.
 *
 * */

typedef enum {
	T2_mode, R2_mode
} comm_mode;

/*!\enum   baudrates
 *
 * \brief  Baudrates.
 *
 * */

typedef enum {
	br_1200,
	br_2400,
	br_4800,
	br_9600,
	br_19200,
	br_38400,
	br_57600,
	br_115200,
	br_230400
} baudrate;

/*!\enum   operating_channels
 *
 * \brief  EMBIT WMBUS module operating channels.
 *
 * */

typedef enum {
	T_meter,
	T_other,
	R2_ch0,
	R2_ch1,
	R2_ch2,
	R2_ch3,
	R2_ch4,
	R2_ch5,
	R2_ch6,
	R2_ch7,
	R2_ch8,
	R2_ch9,
	reserved

} operating_channel;

/*!\enum   output powers
 *
 * \brief  WMBUS transmission ouptut power in dBM.
 *
 * */

typedef enum {
	minus7, zero, plus10, plus15, plus22, plus27
} output_power;

typedef struct {
	serial_num serial_num; //! 7 bytes
	comm_mode comm_mode; //!1 byte
	operating_channel operating_channel; //! 1 byte
	baudrate baudrate; // 1 byte
	output_power output_power; //! 1 byte

} meter_config_info;

typedef struct {
	uint32_t slave_id;
	char* manf_id;
	uint8_t version;
	uint8_t device_type;
} long_header;

//typedef struct
//{
// typedef union{
//	long_header header;
//	short_header head;
// };
// uint8_t access_no;
// uint8_t status;
// uint16_t signature;
//}responseHeader;

// storage number
/*
 *
 * typedef struct
 * {
 *   Timestamp starting_time;
 *   uint8_t first_storage_number;
 *   uint8_t time_spacing;
 *   uint8_t block_length;
 *
 * }storage_block;
 *
 *
 *
 *
 */

/*!\struct link_frame
 *
 * \brief Framing for the link layer packet as defined in the Embit WMBUS package:
 *
 * */
typedef struct {
	uint8_t options_field_hi;  //! 2 bytes options field higher byte
	uint8_t options_field_lo; //! options field lower byte
	uint8_t custom_channel; //! 1 byte selected channel
	uint8_t custom_power; //! 1 byte transmission power selected in dB
	uint8_t timing_options; //! 1 byte timing options
	uint8_t l_field;  //! 1 byte application layer frame length field
	uint8_t c_field; //! 1 byte control field
	//uint8_t address[8];

} link_frame;

/*!\struct EN_13757_3_master_message
 *
 * \brief  Framing for the Application layer packet sent from the master to the slave based on the EN13757_3 Dedicated application layer standard.
 *
 * */

typedef struct {
	uint8_t master_CI; //! the Control information field of the packet sent from the master
					   //! to the slave as specified by the EN13757_3 Standard.
	//uint8_t message_header;
	variable_data data_description; //! data record header description
	uint8_t *actual_data; //! byte array for the actual application layer data transferred

} EN_13757_3_master_message;

/*!\struct EN_13757_3_slave_message
 *
 * \brief  Framing for the Application layer packet sent from the slave to the master based on the EN13757_3 Dedicated application layer standard.
 *
 * */

typedef struct {
	uint8_t slave_CI; //! the Control information field of the packet sent from the slave
					  //! to the master as specified by the EN13757_3 Standard.
	slave_message_header message_header; //! message header of the packet
	variable_data data_description; //! data record header description
	uint8_t *actual_data; //! byte array for the actual application layer data transferred

} EN_13757_3_slave_message;

void t_handler();

int Process_Master_Command(EN_13757_3_master_message *request);

void Send_Slave_Response(const uint8_t *response, uint8_t response_length,
		char response_type);

void Process_master_message(uint8_t *App_Mssg_payload, int payload_length);

void send_ack(void);

void Set_Baud_Rate(unsigned long Baudrate);

int Test_New_Baud(); // check wether the newly configured baudrate is working

meter_config_info get_meter_info();
//int process_frame(dataBlock *messageData);

void prc_snd_ud();
void rq_class_2();
void rq_class_2_px();
//void rq_class_2_sa();
void set_br(uint8_t BAUD);
#endif /* COMM_APP_H_ */
