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
#include <stdlib.h>

typedef struct {

	uint32_t serial_no;
	uint32_t product_no;
	uint32_t current_date;
	uint32_t manf_date;
	float energy_total;
	float energy_january;
	float energy_february;
	float energy_march;
	float energy_april;
	float energy_may;
	float energy_june;
	float energy_july;
	float energy_august;
	float energy_september;
	float energy_october;
	float energy_november;
	float energy_december;
	float calib_t1;
	float calib_t2;
	float calib_flow;

} stored_list;

#endif /* UTIL_PARAMS_H_ */
