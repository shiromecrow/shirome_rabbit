/*
 * CL_EnoderGyro.h
 *
 *  Created on: Oct 15, 2023
 *      Author: sf199
 */

#ifndef INC_CL_ENODERGYRO_H_
#define INC_CL_ENODERGYRO_H_

#include "stm32g4xx_hal.h"

//#define TIRE_DIAMETER_L 0.0118088*1.1126*396/399.73*400/398.99*737/720*708/720*1342/1350//m
//#define TIRE_DIAMETER_R 0.0118088*1.1126*396/399.73*400/402.66*737/720*708/720*1342/1350///m
#define TIRE_DIAMETER 0.0130672*723/720
//*real_dis/goal_dis    * Lのほうに　*sum_R/sum_L
#define pi 3.1415926535


#define	THETA_COMP_R0	1808.996514
#define	THETA_COMP_R1	31.39056836
#define	THETA_COMP_R2	19.22887379
#define	THETA_COMP_R3	-8.05E+01
#define	THETA_COMP_R4	-93.67762412
#define	THETA_COMP_L0	1938.681712
#define	THETA_COMP_L1	28.8371333
#define	THETA_COMP_L2	-46.53140134
#define	THETA_COMP_L3	-7.824077518
#define	THETA_COMP_L4	236.6062566


#define	ANGLE_MEAN_SIZE 200

#define GYRO_COEFFICIENT 1.001154514*3610/3600*3580/3600
#define ACCEL_COEFFICIENT 1.005204002159305*589.2993/619.7859



extern float yaw_angle,angle_speed;
extern float anglex,angle_speedx,angle_speedx_set;
extern float gf_speed,gf_distance,gf_accel;


extern float E_distanceR,E_distanceL;
extern float E_speedR,E_speedL;

extern float E_lpf_distanceL,E_lpf_distanceR;
extern float E_lpf_speedL,E_lpf_speedR;

extern float G_hpf_distanceL,G_hpf_distanceR;
extern float G_hpf_speedL,G_hpf_speedR;


extern float fusion_distanceL,fusion_distanceR;
extern float fusion_speedL,fusion_speedR;
extern float straight_alpha;

extern float theta_comp_gain;//角度伝達誤差の補償

extern float kalman_speed,kalman_distance,kalman_distance2;


void init_EncoderGyro();

void reset_Kalman();
void reset_gyro();
void reset_EncoderGyro_MeanVariance();
void reset_gyro_integral();
void reset_distance();
void reset_speed();

void interupt_calEncoder();
void interupt_calFusion();
void interupt_calKalman();
void interrupt_calGyro();


#endif /* INC_CL_ENODERGYRO_H_ */
