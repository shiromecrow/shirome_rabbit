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

/* 多項式近似 *//* 直線は未使用 */
#define	LIN_COEFFICIENT_L3	-4.06268598105427e-09
#define	LIN_COEFFICIENT_L2	2.62911162626936e-05
#define	LIN_COEFFICIENT_L1	-0.0641076004574342
#define	LIN_COEFFICIENT_L0	55.8723520696743
#define	LIN_COEFFICIENT_R3	-3.23460097230354e-09
#define	LIN_COEFFICIENT_R2	2.19413901668708e-05
#define	LIN_COEFFICIENT_R1	-0.0565271572188971
#define	LIN_COEFFICIENT_R0	47.1818638718591

#define	LIN_SLANT_COEFFICIENT90_L_n3	18861609.5233588
#define	LIN_SLANT_COEFFICIENT90_L_n2	-716450.001531274
#define	LIN_SLANT_COEFFICIENT90_L_n1	11221.1264786236
#define	LIN_SLANT_COEFFICIENT90_L_00	23.4939459070913
#define	LIN_SLANT_COEFFICIENT90_L_p1	-0.00751224157437050
#define	LIN_SLANT_COEFFICIENT90_R_n3	9223517.44224534
#define	LIN_SLANT_COEFFICIENT90_R_n2	-471883.852561020
#define	LIN_SLANT_COEFFICIENT90_R_n1	9361.61530694957
#define	LIN_SLANT_COEFFICIENT90_R_00	22.4723479792329
#define	LIN_SLANT_COEFFICIENT90_R_p1	-0.00780138084963177
#define	LIN_SLANT_COEFFICIENT45_L_n3	269259893.200147
#define	LIN_SLANT_COEFFICIENT45_L_n2	-3587323.68515450
#define	LIN_SLANT_COEFFICIENT45_L_n1	23827.8024931774
#define	LIN_SLANT_COEFFICIENT45_L_00	24.0431617739343
#define	LIN_SLANT_COEFFICIENT45_L_p1	-0.00678534925629928
#define	LIN_SLANT_COEFFICIENT45_R_n3	150096621.588299
#define	LIN_SLANT_COEFFICIENT45_R_n2	-2031389.71002010
#define	LIN_SLANT_COEFFICIENT45_R_n1	18359.0636512324
#define	LIN_SLANT_COEFFICIENT45_R_00	36.1046747794003
#define	LIN_SLANT_COEFFICIENT45_R_p1	-0.00929161978509198

/* log(1/x)近似(未使用) */
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
extern float g_sensor_distance_slant[SENSOR_NUM][12];
extern float g_sensor_distance_slant_diff[SENSOR_NUM];

extern float g_V_battery[20];
extern float g_V_battery_mean;

void battcheak();

void interupt_calSensor();


void sensor_line();
void sensor_line_slant();

#endif /* INC_CL_SENSOR_H_ */
