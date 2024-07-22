/*
 * PL_timer.c
 *
 *  Created on: Dec 25, 2023
 *      Author: sf199
 */


#include "PL_timer.h"
#include "tim.h"


volatile uint32_t g_timCount;
float g_timCount_sec;
uint8_t count_mode;

void pl_timer_init(void){
	count_mode=0;
	g_timCount_sec=0;
	HAL_TIM_Base_Start_IT(&htim6);//割り込み
}

void interrupt_timer(void){
	g_timCount++;
	if(count_mode==1){
	g_timCount_sec=g_timCount_sec + INTERRUPT_TIME;
	}

}

void tic_timer(void){
	count_mode=1;
	g_timCount_sec=0;
}

float toc_timer(void){
	float timer;
	timer=g_timCount_sec;
	count_mode=0;
	g_timCount_sec=0;
	return timer;
}

void wait_ms(uint32_t waitTime) {

        g_timCount = 0;
        __HAL_TIM_SET_COUNTER(&htim6, 0);
        while ((float)(g_timCount) * 0.001 / INTERRUPT_TIME < waitTime) {
        }

}

void wait_ms_NoReset(uint32_t waitTime) {

        g_timCount = 0;
        while ((float)(g_timCount) * 0.001 / INTERRUPT_TIME < waitTime) {
        }

}
