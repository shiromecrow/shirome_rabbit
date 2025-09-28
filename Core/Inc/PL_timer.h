/*
 * PL_timer.h
 *
 *  Created on: Dec 25, 2023
 *      Author: sf199
 */

#ifndef INC_PL_TIMER_H_
#define INC_PL_TIMER_H_

#include <stdint.h>
/* 割り込み時間0.5msのときはINV_INTERRUPT_TIMEを2に設定(0.001/INTERRUPT_TIME)  */
#define INTERRUPT_TIME 0.0005
#define INV_INTERRUPT_TIME 2
#define TIM6LOG_SIZE 13


extern uint16_t tim6_log[TIM6LOG_SIZE];

extern volatile uint32_t g_timCount;
extern float g_timCount_sec;

void pl_timer_init();

void interrupt_timer();

void tic_timer();
float toc_timer();

void wait_ms(uint32_t);
void wait_ms_NoReset(uint32_t);

#endif /* INC_PL_TIMER_H_ */
