/*
 * params.h
 *
 *  Created on: Nov 19, 2020
 *      Author: user
 */
/*!
 * @file params.h
 *
 * Definititions for systemwide measured parameters and running sums.
 *
 * @Author Deksios Bekele
 *
 * @date Nov 19, 2020
 *
 */
#ifndef UTIL_PARAMS_H_
#define UTIL_PARAMS_H_

#include <stdint.h>

typedef struct {
	uint32_t serial_no;
	uint32_t product_no;
	uint32_t manf_date;
	uint32_t current_date;
	uint64_t energy_total;
	uint32_t energy_january;
	uint32_t energy_february;
	uint32_t energy_march;
	uint32_t energy_april;
	uint32_t energy_may;
	uint32_t energy_june;
	uint32_t energy_july;
	uint32_t energy_august;
	uint32_t energy_september;
	uint32_t energy_october;
	uint32_t energy_november;
	uint32_t energy_december;
	uint64_t calib_t1;
	uint64_t calib_t2;
	uint64_t calib_flow;

} stored_list;

#endif /* UTIL_PARAMS_H_ */
