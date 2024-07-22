/*
 * mode_select.h
 *
 *  Created on: Jan 9, 2024
 *      Author: sf199
 */

#ifndef INC_MODE_SELECT_H_
#define INC_MODE_SELECT_H_

#include "stm32g4xx_hal.h"

#define MODE_SENSER_DEC (g_sensor[0][0] <= SENSOR_FINGER_0 || g_sensor[2][0] <= SENSOR_FINGER_2 || g_sensor[3][0] <= SENSOR_FINGER_3 || g_sensor[5][0] <= SENSOR_FINGER_5)


extern uint16_t main_mode;
unsigned char mode_decision(unsigned char);

void mode_execution(unsigned char);

void mode_PLtest(unsigned char);

void mode_Running(unsigned char);

void mode_Running2(unsigned char);

void mode_Tuning0(unsigned char);
void mode_Tuning1(unsigned char);
void mode_WallSensorTuning(unsigned char);

void mode_WallSensorTuning_fast(unsigned char);

#endif /* INC_MODE_SELECT_H_ */
