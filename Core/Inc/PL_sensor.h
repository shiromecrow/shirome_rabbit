/*
 * PL_sensor.h
 *
 *  Created on: Dec 24, 2023
 *      Author: sf199
 */

#ifndef INC_PL_SENSOR_H_
#define INC_PL_SENSOR_H_

#include "stm32g4xx_hal.h"

#define SENSOR_NUM 6

extern uint16_t g_ADCBuffer[SENSOR_NUM+1];
extern uint16_t g_sensor_on[SENSOR_NUM];
extern uint16_t g_sensor_off[SENSOR_NUM];


extern float g_V_batt;

void pl_sensor_init();

float pl_getbatt();

void pl_callback_getSensor();

void pl_interupt_getSensor();

#endif /* INC_PL_SENSOR_H_ */
