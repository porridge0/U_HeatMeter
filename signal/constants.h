/*
 * constants.h
 *
 *  File containing mathematical definitions and constants.
 *
 *  Created on: Nov 26, 2020
 *      Author: user
 */

#ifndef SIGNAL_CONSTANTS_H_
#define SIGNAL_CONSTANTS_H_

#include <math.h>
// Liquid water properties
#define STANDARD_HEAT_CAPACITY  4,200 // J/kg°C
#define	STANDARD_DENSITY 997 // Kg/m³

#define SAMPLING_RATE	15 // in seconds

/* PT1000 --------------------------------------------------------------------*/
//R0, A, B are parameters as specified in EN60 751
#define R0_at_0C (float)1000            // R0 is the RTD resistance at 0 °C
#define Coeff_A (float)(3.9083 / 1000)    // A = 3,9083 x 10-3 °C-1
#define Coeff_B (float)(-5.775 / 10000000) // B = -5,775 x 10-7 °C-1
#define corr_fact  (float)1.0000 // Corr.-factor for temperature resistance ratio

#define CAL_TEMP(x)	(-R0_at_0C * Coeff_A \
+ sqrt(((R0_at_0C * Coeff_A) * (R0_at_0C * Coeff_A))\
				- 4 * R0_at_0C * Coeff_B * (R0_at_0C - x)))\
/ (2 * R0_at_0C * Coeff_B)

#define AVG_BY_HITS(x,y) (x/y)
#define PICO_TO_NANO(t)  (t*1000)

#define NUMBER_OF_SAMPLES 	10
#endif /* SIGNAL_CONSTANTS_H_ */
