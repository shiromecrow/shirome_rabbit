/*
 * PID_EncoderGyro.c
 *
 *  Created on: Jan 14, 2023
 *      Author: sf199
 */

#include "PID_EncoderGyro.h"
#include "PID_wall.h"
#include "fail_safe.h"
#include "CL_EnoderGyro.h"
#include "CL_sensor.h"
#include "PL_LED.h"
#include "Control_motor.h"
#include "define.h"

//#include "fail_safe.h"

#include "stdio.h"
#include "math.h"

struct PID enc;
struct PID Gyro;
struct PID Gyro_angle;

float Ksp, Ksi, Ksd;
float Ktp, Kti, Ktd;
float Ktp_angle, Ktd_angle;
char Encoder_PID_mode,Gyro_PID_mode,Accel_PID_mode;

void PID_Init(void) {

	Ksp = 3.1; //3//P項の制御量直進
	Ksi = 0.08; //80//5//43//I項の制御量直進
	Ksd = 0.00; //D項の制御量直進
	Ktp = 1.6; //295//P項の制御量旋回
	Kti = 0.06; //1//.6//I項の制御量旋回
	Ktd = 0.002; //205//D項の制御量旋回
	Ktp_angle = 0.0; //P項の制御量旋回
	Ktd_angle = 0.000; //D項の制御量旋回
	enc.sigma_error = 0;
	Gyro.sigma_error = 0;
	Encoder_PID_mode=1;
	Gyro_PID_mode=1;
	Accel_PID_mode=1;

}

void clear_Ierror(void) {
	enc.sigma_error = 0;
	Gyro.sigma_error = 0;
}


void EncoderGyro_PID(float *PID_s, float *PID_t,float straight_velocity,float turning_velocity,float turning_displacement) {
	float PID_stra = 0;
	float PID_turn = 0;
	float obs_vel_str;//直線観測値
	float obs_vel_turn;//旋回観測値
	float obs_angle_turn;//旋回観測値



	if(highspeed_mode == 0){//探索用
		Ksp = 3.8; //3//P項の制御量直進
		Ksi = 0.10; //80//5//43//I項の制御量直進
		Ksd = 0.00; //D項の制御量直進
		Ktp = 2.8; //295//P項の制御量旋回
		Kti = 0.122; //1//.6//I項の制御量旋回
		Ktd = 0.002; //205//D項の制御量旋回
		Ktp_angle = 0.0; //P項の制御量旋回
		Ktd_angle = 0.0; //D項の制御量旋回
		if(modeacc == 2){
			Ktp = 3.3; //295//P項の制御量旋回
			Kti = 0.13; //1//.6//I項の制御量旋回
			Ktd = 0.02; //205//D項の制御量旋回
			Ktp_angle = 250; //P項の制御量旋回
			Ktd_angle = 1; //D項の制御量旋回
		}else if(modeacc == 4){
			Ktp_angle = 150; //P項の制御量旋回
			Ktd_angle = 1; //D項の制御量旋回
		}
		obs_vel_str = (fusion_speedR + fusion_speedL) / 2;
		obs_vel_turn = angle_speed;
		obs_angle_turn=yaw_angle;
	}else if(highspeed_mode == 1){//最短用
		Ksp = 3.9; //3//P項の制御量直進
		Ksi = 0.06; //80//5//43//I項の制御量直進
		Ksd = 0.001; //D項の制御量直進
		Ktp = 4.3; //295//P項の制御量旋回
		Kti = 0.132; //1//.6//I項の制御量旋回
		Ktd = 0.1; //205//D項の制御量旋回
		Ktp_angle = 30; //P項の制御量旋回
		Ktd_angle = 1; //D項の制御量旋回
		if(g_WallControl_mode >= 1){
			Ktp_angle = 50; //P項の制御量旋回
			Ktd_angle = -4; //D項の制御量旋回
		}
		if(modeacc == 6){
			Ksp = 4.7; //3//P項の制御量直進
			Ksi = 0.13; //80//5//43//I項の制御量直進
			Ksd = 0.001; //D項の制御量直進
			Ktp = 4.8; //295//P項の制御量旋回
			Kti = 0.132; //1//.6//I項の制御量旋回
			Ktd = 3; //205//D項の制御量旋回
			Ktp_angle = 30; //P項の制御量旋回
			Ktd_angle = 1; //D項の制御量旋回
		}
		if (straight_velocity < 1800){
			obs_vel_str = (fusion_speedR + fusion_speedL) / 2;
		}else{
			obs_vel_str = 0.5*kalman_speed + 0.5*(fusion_speedR + fusion_speedL) / 2;
		}
		obs_vel_turn = angle_speed;
		obs_angle_turn=yaw_angle;
	}

	enc.error = straight_velocity - obs_vel_str;
	enc.delta_error = enc.error - enc.old_error;
	enc.old_error = enc.error;
	enc.sigma_error += enc.error;
	PID_stra = Ksp * enc.error + Ksi * enc.sigma_error + Ksd * enc.delta_error;




	Gyro.error = turning_velocity - obs_vel_turn;
	Gyro.delta_error = Gyro.error - Gyro.old_error;
	Gyro.old_error = Gyro.error;
	Gyro.sigma_error += Gyro.error;
	PID_turn = Ktp * Gyro.error + Kti * Gyro.sigma_error
			+ Ktd * Gyro.delta_error;


	Gyro_angle.error = turning_displacement - obs_angle_turn;
	Gyro_angle.delta_error = Gyro_angle.error - Gyro_angle.old_error;
	Gyro_angle.old_error = Gyro_angle.error;
	PID_turn += Ktp_angle * Gyro_angle.error + Ktd_angle * Gyro_angle.delta_error;




	*PID_s = PID_stra / MAXMOTOR * g_V_battery_mean;
	*PID_t = PID_turn / MAXMOTOR * g_V_battery_mean;

}





