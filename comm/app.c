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

extern volatile lnk_frame *r_frame;

uint8_t secondary_addressing = 0;
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
		secondary_addressing = 0;
		if (FALSE_NA == validate_sa())
			return;
		if (TRUE_SA == validate_sa())
			secondary_addressing = 1;

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
		secondary_addressing = 0;
		if (FALSE_NA == validate_sa())
			return;
		if (TRUE_SA == validate_sa())
			secondary_addressing = 1;
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
 * @brief Function Name: send_ack
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

}
///*!
// * @brief Function Name: set_addressing
// *
// * Determine addressing type based on validated physical or link layer address values.
// *
// * @param none
// *
// * @return none
//
//void set_addressing() {
//	secondary_addressing = 0;
//	if (FALSE_NA == validate_sa()) /*!--addresses do not match - ignore*/
//		return;
//	if (TRUE_SA == validate_sa()) /*!--match secondary address */
//		secondary_addressing = 1;
//}

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
		set_br(r_frame->ci_field);
	} else if (r_frame->ci_field == APPLICATION_RESET) {
		/*!-- TODO -> handle application reset*/
	} else if (r_frame->ci_field == SEND_DATA_MASTER_TO_SLAVE) {
		uint8_t DIF = *r_frame->user_data; //copy first byte
		uint8_t VIF = r_frame->user_data[1]; //copy second byte
		/*!--handle set primary address */
		if (DIF == INST_8_BIT_INTEGER && VIF == VIF_SET_PRIMARY_ADDRESS) {
			uint8_t PA;
			if (secondary_addressing) {
				PA = r_frame->user_data[17]; //18th byte
			} else {
				PA = r_frame->user_data[9]; //10th byte
			}
			assert(0xFA >= PA); // permitted primary address values
		} /*!--handle set secondary address */
		else if (DIF == INST_8_DIGIT_BCD && VIF == VIF_SET_SECONDARY_ADDRESS) {
			uint32_t SA;
			if (secondary_addressing) {
				SA = r_frame->user_data[17]; //18th byte
				SA <<= 8;
				SA |= r_frame->user_data[18];
				SA <<= 8;
				SA |= r_frame->user_data[19];
				SA <<= 8;
				SA |= r_frame->user_data[20];
			} else {
				SA = r_frame->user_data[9]; //10th byte
				SA <<= 8;
				SA |= r_frame->user_data[10];
				SA <<= 8;
				SA |= r_frame->user_data[11];
				SA <<= 8;
				SA |= r_frame->user_data[12];
			}
		}

	}
	/*!--TODO -- write to memory */
	ENABLE_UX_INT;
}

void set_br(uint8_t BAUD) {

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
