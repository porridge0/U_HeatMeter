/*!
 * @file system.h
 *
 * File for declaring specific system-wide definitions.
 *
 * @note :
 *
 * @author Deksios Bekele
 *
 * @date Nov 2, 2020
 *
 */
#ifndef CONFIG_SYSTEM_H_
#define CONFIG_SYSTEM_H_

#include <stdint.h>
#include <msp430.h>

#define _ENABLE_SYSTEM_WIDE_DEFS_

#ifdef _ENABLE_SYSTEM_WIDE_DEFS_
void __ATOMIZE(){
	__disable_interrupt();
	__no_operation();
}
void __END_ATOMIC(){
	__enable_interrupt();
}
#endif

#endif /* CONFIG_SYSTEM_H_ */
