//******************************************************************************
// delay.h                                                                     *
//******************************************************************************
// Delay functions for STM8 at 16 MHz.                                         *
// By Ed Smallenburg.                                                          *
//******************************************************************************
// History:                                                                    *
// 09-10-2018, ES: First set-up.                                               *
// 27-03-2019, ES: Version for stoflamp software.                              *
//******************************************************************************
#ifndef _UTIL_DELAY_H_
#define _UTIL_DELAY_H_

#include "stm8s.h"

void delay_cycl ( uint16_t ticks ) ;
void delay_us ( uint16_t us ) ;         // Delay for a number of usec
void delay_ms ( uint16_t ms ) ;         // Delay for a number of msec

#endif
 
