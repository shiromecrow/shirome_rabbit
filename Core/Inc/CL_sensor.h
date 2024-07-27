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

#define BATTLIMIT 7.7

#define SENSOR_FINGER_0 700
#define SENSOR_FINGER_1 700
#define SENSOR_FINGER_2 700
#define SENSOR_FINGER_3 700
#define SENSOR_FINGER_4 700
#define SENSOR_FINGER_5 700

#define	LIN_COEFFICIENT_L3	-4.06268598105427e-09
#define	LIN_COEFFICIENT_L2	2.62911162626936e-05
#define	LIN_COEFFICIENT_L1	-0.0641076004574342
#define	LIN_COEFFICIENT_L0	55.8723520696743
#define	LIN_COEFFICIENT_R3	-3.23460097230354e-09
#define	LIN_COEFFICIENT_R2	2.19413901668708e-05
#define	LIN_COEFFICIENT_R1	-0.0565271572188971
#define	LIN_COEFFICIENT_R0	47.1818638718591


#define	LIN_SLANT_COEFFICIENT90_L_a	22.5384525690320
#define	LIN_SLANT_COEFFICIENT90_L_b	20.1176694270008
#define	LIN_SLANT_COEFFICIENT90_L_c	105.243125086856
#define	LIN_SLANT_COEFFICIENT90_R_a	21.4629938989988
#define	LIN_SLANT_COEFFICIENT90_R_b	34.2456063295782
#define	LIN_SLANT_COEFFICIENT90_R_c	89.9144977432086
#define	LIN_SLANT_COEFFICIENT45_L_a	25.4576573090608
#define	LIN_SLANT_COEFFICIENT45_L_b	60.3074998117289
#define	LIN_SLANT_COEFFICIENT45_L_c	93.7666075748895
#define	LIN_SLANT_COEFFICIENT45_R_a	26.5187892321985
#define	LIN_SLANT_COEFFICIENT45_R_b	13.7571607876818
#define	LIN_SLANT_COEFFICIENT45_R_c	143.626636490865


extern int g_sensor[SENSOR_NUM][20];
extern int g_sensor_diff[SENSOR_NUM];
extern int g_sensor_diff_wallcut[SENSOR_NUM];
extern int g_sensor_diff_wallcut_slant[SENSOR_NUM];
extern int g_sensor_mean[SENSOR_NUM];
extern float g_sensor_distance[SENSOR_NUM];
extern float g_sensor_distance_slant[SENSOR_NUM];

extern float g_V_battery[20];
extern float g_V_battery_mean;

void battcheak();

void interupt_calSensor();


void sensor_line();
void sensor_line_slant();

#endif /* INC_CL_SENSOR_H_ */
