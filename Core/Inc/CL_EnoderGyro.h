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
#define TIRE_DIAMETER (0.013157715306955*716/720)
//*real_dis/goal_dis    * Lのほうに　*sum_R/sum_L
#define pi 3.1415926535


#define	THETA_COMP_R0	2575.035003
#define	THETA_COMP_R1	19.55126523
#define	THETA_COMP_R2	-2.809210506
#define	THETA_COMP_R3	48.60546803
#define	THETA_COMP_R4	1.656801313
#define	THETA_COMP_R5	62.7300022
#define	THETA_COMP_R6	-0.702476023
#define	THETA_COMP_L0	2645.739809
#define	THETA_COMP_L1	152.3650293
#define	THETA_COMP_L2	-1.8962285
#define	THETA_COMP_L3	395.1428153
#define	THETA_COMP_L4	1.630226213
#define	THETA_COMP_L5	48.02399232
#define	THETA_COMP_L6	-1.295564294




#define	ANGLE_MEAN_SIZE 200

#define GYRO_COEFFICIENT 0.998358079323549*3607/3600
#define ACCEL_COEFFICIENT 0.966332120340700*3837/3020*3428/4021

	//encY_variance = 20.961668;	
	//accelY_mean=-38.457546;
	//accelY_variance=276.637299;

	//encY_variance = 25.358568;
	//accelY_mean=333.726196;
	//accelY_variance=710167296;
#define	ENCY_VAR 25.358568
#define	ACCY_MEAN 333.726196
#define	ACCY_VAR 443236.6891442692

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

extern float g_omegaZ_mean,g_accelY_mean;
extern float g_encY_variance,g_accelY_variance;

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
