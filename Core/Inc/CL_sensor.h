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

/* 多項式近似 */
#define	LIN_COEFFICIENT_L3	-4.06268598105427e-09
#define	LIN_COEFFICIENT_L2	2.62911162626936e-05
#define	LIN_COEFFICIENT_L1	-0.0641076004574342
#define	LIN_COEFFICIENT_L0	55.8723520696743
#define	LIN_COEFFICIENT_R3	-3.23460097230354e-09
#define	LIN_COEFFICIENT_R2	2.19413901668708e-05
#define	LIN_COEFFICIENT_R1	-0.0565271572188971
#define	LIN_COEFFICIENT_R0	47.1818638718591

#define	LIN_SLANT_COEFFICIENT90_L_n3	750660.954536716
#define	LIN_SLANT_COEFFICIENT90_L_n2	-90860.4967917818
#define	LIN_SLANT_COEFFICIENT90_L_n1	4090.90580483953
#define	LIN_SLANT_COEFFICIENT90_L_00	35.4887963245357
#define	LIN_SLANT_COEFFICIENT90_L_p1	-0.0188980156285247
#define	LIN_SLANT_COEFFICIENT90_R_n3	553872.685725022
#define	LIN_SLANT_COEFFICIENT90_R_n2	-83216.0425487541
#define	LIN_SLANT_COEFFICIENT90_R_n1	4161.12662821295
#define	LIN_SLANT_COEFFICIENT90_R_00	31.7524312757797
#define	LIN_SLANT_COEFFICIENT90_R_p1	-0.0139539310331728
#define	LIN_SLANT_COEFFICIENT45_L_n3	3674089.35556890
#define	LIN_SLANT_COEFFICIENT45_L_n2	-272521.495196959
#define	LIN_SLANT_COEFFICIENT45_L_n1	7442.16137046420
#define	LIN_SLANT_COEFFICIENT45_L_00	32.1725984931874
#define	LIN_SLANT_COEFFICIENT45_L_p1	-0.0146428626991881
#define	LIN_SLANT_COEFFICIENT45_R_n3	20223569.2329503
#define	LIN_SLANT_COEFFICIENT45_R_n2	-812470.731331702
#define	LIN_SLANT_COEFFICIENT45_R_n1	13116.6179246028
#define	LIN_SLANT_COEFFICIENT45_R_00	23.6803994592509
#define	LIN_SLANT_COEFFICIENT45_R_p1	-0.00601171415974540


/* log(1/x)近似 */
#define	LIN_COEFFICIENT90_L_a	20.6024625744798
#define	LIN_COEFFICIENT90_L_b	21.6920846689954
#define	LIN_COEFFICIENT90_L_c	92.7792207710739
#define	LIN_COEFFICIENT90_R_a	18.5276947503951
#define	LIN_COEFFICIENT90_R_b	21.8895235592514
#define	LIN_COEFFICIENT90_R_c	79.5375833599593


#define	LIN_SLANT_COEFFICIENT90_L_a	22.5613740409070
#define	LIN_SLANT_COEFFICIENT90_L_b	14.6583539219103
#define	LIN_SLANT_COEFFICIENT90_L_c	112.460789193969
#define	LIN_SLANT_COEFFICIENT90_R_a	21.4919287916330
#define	LIN_SLANT_COEFFICIENT90_R_b	24.1012697756161
#define	LIN_SLANT_COEFFICIENT90_R_c	97.5398111469422
#define	LIN_SLANT_COEFFICIENT45_L_a	25.4737148838597
#define	LIN_SLANT_COEFFICIENT45_L_b	22.9233958024017
#define	LIN_SLANT_COEFFICIENT45_L_c	118.448289128121
#define	LIN_SLANT_COEFFICIENT45_R_a	26.5320176936574
#define	LIN_SLANT_COEFFICIENT45_R_b	27.5192090983310
#define	LIN_SLANT_COEFFICIENT45_R_c	125.288326521041


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
