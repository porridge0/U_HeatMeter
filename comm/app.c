/*
 * @file app.c
 *
 * File for configuring the app layer of the M-Bus communication protocol according to the EN 13757-3 standard.
 *
 * @note Implementation is based on industry recommended practices.
 *
 * @author Deksios Bekele
 *
 * @date Oct 13, 2020
 *
 */

#include "app.h"
#include "phy.h"
#include <assert.h>

volatile uint8_t isSecondaryAddressing = 0;
#pragma PERSISTENT(accessNumber)
uint8_t accessNumber = 0; //should not be resettable

/*!--month registers*/
#pragma PERSISTENT(_month_reg)
month_reg _month_reg[12] = { { .m_reg = 0x00000000 } }; // captured energy values at the end of each month

/*!--Energy in milli Joules - actual result scaled by a factor of 1000 */
#pragma PERSISTENT(c_enReg)
volatile uint64_t c_enReg = 0; //energy register

/*!
 * @brief Function Name: t_handler
 *
 * Callback to handle received message.
 *
 * @param none
 *
 * @return none
 */
void t_handler() {

	if (r_frame->type == short_frame && r_frame->c_field == 0x40) {
		/*!-- TODO
		 * Handle SND_NKE initialization -> ACK or ignore
		 */
		send_ack();
	} else if (r_frame->type == short_frame
			&& (r_frame->c_field == 0x5B || r_frame->c_field == 0x7B)) {
		/*!-- TODO
		 * Handle REQ_UD2 -> request for class 2 data */
		if (r_frame->a_field <= 0xFA) {
			//assert(r_frame->a_field == devInfo.primaryAddress);
			if (r_frame->a_field != devInfo.primaryAddress)
				return; // check if primary address matches the current slave's
		}
		rq_class_2();
	} else if (r_frame->type == short_frame
			&& (r_frame->c_field == 0x5A || r_frame->c_field == 0x7A)) {
		/*!-- TODO
		 *
		 * Future implemetation
		 * Handle REQ_UD1 -> request for class 1(alarm) data
		 *
		 * */
	} else if (r_frame->c_field == 0x53 || r_frame->c_field == 0x73) {
		/*!-- TODO
		 * Handle SND_UD -> Master to slave data trasnsfer */
		isSecondaryAddressing = 0;
		if (FALSE_NA == c_addressing())
			return;
		if (TRUE_SA == c_addressing())
			isSecondaryAddressing = 1;
		send_ack();
		prc_snd_ud();
	}

}

/*!
 * @brief Function Name: send_ack
 *
 * Link-layer acknowledgment from slave to master.
 *
 * @param none
 *
 * @return none
 */
void send_ack(void) {
	/*!-- Insert standard recommended delays before every aknowledgment */
	memcpy((uint8_t *) tx_buff, (uint8_t *) ACKNOWLEDGMENT, 1);
	tx_lnk_frame(1);
}

/*!
 * @brief Function Name: rq_class_2
 *
 * Handles EN-13757-2 class 2 Level requests from the master.
 *
 * @param none
 *
 * @return none
 */
void rq_class_2() {
	rq_class_2_px();
}
/*!
 * @brief Function Name: rq_class_2_px
 *
 * Processes slave response datagrams to master.
 *
 * @param none
 *
 * @return none
 */
void rq_class_2_px() {
	/*!--Construct response header*/
	rsp_frame->start = MBUS_FRAME_CONTROL_OR_LONG_START;
	rsp_frame->startR = MBUS_FRAME_CONTROL_OR_LONG_START;
	rsp_frame->c_field = 0x08; // RSP_UD
	rsp_frame->a_field = devInfo.primaryAddress;
	rsp_frame->ci_field = SLAVE_TO_MASTER_FULL_HEADER;

	//app data
	rsp_frame->user_data[0] = (devInfo.serialNumber & 0xFF);
	rsp_frame->user_data[1] = ((devInfo.serialNumber >> 8) & 0xFF);
	rsp_frame->user_data[2] = ((devInfo.serialNumber >> 16) & 0xFF);
	rsp_frame->user_data[3] = ((devInfo.serialNumber >> 24) & 0xFF);

	rsp_frame->user_data[4] = (MANUFACTURER_ID & 0xFF);
	rsp_frame->user_data[5] = ((MANUFACTURER_ID >> 8) & 0xFF);
	rsp_frame->user_data[6] = VERSION_NUMBER;
	rsp_frame->user_data[7] = HEAT_METER;
	rsp_frame->user_data[8] = ++accessNumber;
	rsp_frame->user_data[9] = 0; //status - update as needed
	rsp_frame->user_data[10] = 0; //signature - lower
	rsp_frame->user_data[11] = 0; //signature - higher

	// read-out data - manufacture date
	rsp_frame->user_data[12] = (devInfo.productDate & 0xFF);
	rsp_frame->user_data[13] = ((devInfo.productDate >> 8) & 0xFF);
	rsp_frame->user_data[14] = ((devInfo.productDate >> 16) & 0xFF);
	rsp_frame->user_data[15] = ((devInfo.productDate >> 24) & 0xFF);

	// read-out data - measurement - total energy
	rsp_frame->user_data[16] = (c_enReg & 0xFF);
	rsp_frame->user_data[17] = ((c_enReg >> 8) & 0xFF);
	rsp_frame->user_data[18] = ((c_enReg >> 16) & 0xFF);
	rsp_frame->user_data[19] = ((c_enReg >> 24) & 0xFF);
	rsp_frame->user_data[20] = ((c_enReg >> 32) & 0xFF);
	rsp_frame->user_data[21] = ((c_enReg >> 40) & 0xFF);
	rsp_frame->user_data[22] = ((c_enReg >> 48) & 0xFF);
	rsp_frame->user_data[23] = ((c_enReg >> 56) & 0xFF);

	uint8_t j = 0;
	uint8_t k = 0;
	for (; j < 12; j++) {
		rsp_frame->user_data[24 + k] = (_month_reg[j].m_reg & 0xFF);
		rsp_frame->user_data[25 + k] = ((_month_reg[j].m_reg >> 8) & 0xFF);
		rsp_frame->user_data[26 + k] = ((_month_reg[j].m_reg >> 16) & 0xFF);
		rsp_frame->user_data[27 + k] = ((_month_reg[j].m_reg >> 24) & 0xFF);
		//include timestamp - only day, month and year values
		rsp_frame->user_data[28 + k] = (_month_reg[j].timeStamp.day);
		rsp_frame->user_data[29 + k] = (_month_reg[j].timeStamp.month);
		rsp_frame->user_data[30 + k] = ((_month_reg[j].timeStamp.year) & 0xFF);
		rsp_frame->user_data[31 + k] = ((_month_reg[j].timeStamp.year >> 8)
				& 0xFF);
		k += 8;
	} // 96 bytes

	rsp_frame->user_data[120] = 0x0F; // DIF - no more data
	//set frame length
	rsp_frame->l_fieldH = 0x7C;
	rsp_frame->l_fieldL = 0x7C; //124  bytes
	//set frame type for checksum
	rsp_frame->checksum = getCheksum(rsp_frame);
	rsp_frame->stop = MBUS_FRAME_STOP;

	memcpy((uint8_t *) tx_buff, (uint8_t *) rsp_frame, rsp_frame->l_fieldH + 6);
	tx_lnk_frame(1);

}

/*!
 * @brief Function Name: prc_snd_ud
 *
 * Processes master to slave data transfer.
 *
 * @param none
 *
 * @return none
 */
void prc_snd_ud() {
	/*!-- will not guarantee atomicity */
	DISABLE_UX_INT;

	if (SET_BAUD_300 <= r_frame->ci_field && r_frame->ci_field <= SET_BAUD_38400) {
		switch_br(r_frame->ci_field);
	} else if (r_frame->ci_field == APPLICATION_RESET) {
		/*!-- TODO -> handle application reset*/
	} else if (r_frame->ci_field == SEND_DATA_MASTER_TO_SLAVE) {
		uint8_t DIF = *r_frame->user_data; //copy first byte
		uint8_t VIF = r_frame->user_data[1]; //copy second byte
		/*!--handle set primary address */
		if (DIF == INST_8_BIT_INTEGER && VIF == VIF_SET_PRIMARY_ADDRESS) {
			uint8_t PA;
			if (isSecondaryAddressing) {
				PA = r_frame->user_data[17]; //18th byte
			} else {
				PA = r_frame->user_data[9]; //10th byte
			}
			assert(0xFA >= PA); // permitted primary address values
			devInfo.primaryAddress = PA;
		} /*!--handle set secondary address */
		else if (DIF == INST_8_DIGIT_BCD && VIF == VIF_SET_SECONDARY_ADDRESS) {
			uint32_t SA;
			if (isSecondaryAddressing) {
				//little endian
				SA = r_frame->user_data[20];
				SA <<= 8;
				SA |= r_frame->user_data[19];
				SA <<= 8;
				SA |= r_frame->user_data[18];
				SA <<= 8;
				SA |= r_frame->user_data[17]; //18th byte
			} else {
				//little endian
				SA = r_frame->user_data[12]; //10th byte
				SA <<= 8;
				SA |= r_frame->user_data[11];
				SA <<= 8;
				SA |= r_frame->user_data[10];
				SA <<= 8;
				SA |= r_frame->user_data[9];
			}
			/*Set the new secondary address of the slave*/
			devInfo.serialNumber = SA;
		}
		//Add any additional manufacturer specific implementations here

	}
	/*!--TODO -- write to memory */
	ENABLE_UX_INT;
}

void switch_br(uint8_t BAUD) {

	uint16_t xb = THREE_HUNDERED;
	switch (BAUD) {

	case SET_BAUD_600:
		xb = SIX_HUNDRED; //not recommended according to the standard
		break;
	case SET_BAUD_1200:
		xb = TWELVE_HUNDRED; //not recommended according to the standard
		break;
	case SET_BAUD_2400:
		xb = TWENTY_FOUR_HUNDRED;
		break;
	case SET_BAUD_4800:
		xb = FOURTY_EIGHT_HUNDRED; //not recommended according to the standard
		break;
	case SET_BAUD_9600:
		xb = NINETY_SIX_HUNDRED;
		break;
	case SET_BAUD_19200:
		xb = NINETEEN_THOUSAND_TWO_HUNDRED; //not recommended according to the standard
		break;
	case SET_BAUD_38400:
		xb = THIRTY_EIGHT_THOUSAND_FOUR_HUNDRED;
		break;
	default:
		break;
	}
	ux_config(xb);
}
