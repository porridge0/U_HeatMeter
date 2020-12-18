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
#include <stdint.h>
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

/*Speed vs Degree Celcius*/
//const uint16_t SPEED_OF_SOUND_IN_WATER_LUT[11] = { 1403, 1427, 1447, 1481, 1507,
//		1526, 1541, 1552, 1555, 1555, 1550 };

#define FLUID_VELOCITY(x,y,z) 	(uint16_t)(x*((y-z)/(y+z)))

#define PIPE_SIZE    (float)0.0127// diameter in meters(0.5)

#define SCALED_PIPE_SIZE	PIPE_SIZE*10000

#define M_PI acos(-1.0)

#define FLOW_RATE(x)   (float)(x*M_PI*SCALED_PIPE_SIZE*SCALED_PIPE_SIZE)

#define STAND_VAR(x,y)    (sqrt((x- (x / y))/ (y - 1)))

#define HEAT_ENERGY(x,dt)   (x*STANDARD_DENSITY*\
		STANDARD_HEAT_CAPACITY*dt*SAMPLING_RATE)

/* Energy register/accumulator in units of mili Joules for
 * better resolution.*/
#define IN_MILLI_JOULES(x) 	(x*1000)

#endif /* SIGNAL_CONSTANTS_H_ */
