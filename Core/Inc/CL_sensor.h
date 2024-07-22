/*
 * CL_sensor.h
 *
 *  Created on: Jan 11, 2023
 *      Author: sf199
 */

#ifndef INC_CL_SENSOR_H_
#define INC_CL_SENSOR_H_

#include "stm32g4xx_hal.h"
#include "PL_sensor.h"

#define BATTLIMIT 7.5

#define SENSOR_FINGER_0 800
#define SENSOR_FINGER_1 800
#define SENSOR_FINGER_2 800
#define SENSOR_FINGER_3 800
#define SENSOR_FINGER_4 800
#define SENSOR_FINGER_5 800

extern int g_sensor[SENSOR_NUM][20];
extern int g_sensor_diff[SENSOR_NUM];
extern int g_sensor_diff_wallcut[SENSOR_NUM];
extern int g_sensor_diff_wallcut_slant[SENSOR_NUM];
extern int g_sensor_mean[SENSOR_NUM];

extern float g_V_battery[20];
extern float g_V_battery_mean;

void battcheak();

void interupt_calSensor();

void sensor_line();
void sensor_line45();
void sensor_line45_slant();

#endif /* INC_CL_SENSOR_H_ */
