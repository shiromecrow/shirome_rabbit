/*
 * PL_timer.h
 *
 *  Created on: Dec 25, 2023
 *      Author: sf199
 */

#ifndef INC_PL_TIMER_H_
#define INC_PL_TIMER_H_

#include "stm32g4xx_hal.h"

#define INTERRUPT_TIME 0.001

extern volatile uint32_t g_timCount;
extern float g_timCount_sec;

void pl_timer_init();

void interrupt_timer();

void tic_timer();
float toc_timer();

void wait_ms(uint32_t);
void wait_ms_NoReset(uint32_t);

#endif /* INC_PL_TIMER_H_ */
