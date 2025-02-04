/*
 * Control_motor.c
 *
 *  Created on: Jan 14, 2023
 *      Author: sf199
 */


#include "Control_motor.h"
#include "CL_sensor.h"
#include "CL_EnoderGyro.h"
#include "PL_motor.h"
#include "PL_LED.h"
#include "PL_timer.h"
//#include "fail_safe.h"
#include "FF_motor.h"

#include "math.h"

#include "PID_wall.h"
#include "PID_EncoderGyro.h"

#include "define.h"
//#include "maze_wall.h"
//#include "maze_strategy.h"

TARGET straight;
TARGET turning;

TRAPEZOID Trapezoid_straight;
TRAPEZOID Trapezoid_turning;

MOLLIFIER Mollifier_turning;

float straight_acceleration_lpf;

uint32_t g_MotorTimCount;
char modeacc;

uint8_t noGoalPillarMode;

float g_V_L,g_V_R;

void Control_mode_Init(void){

	modeacc = 0;
	g_MotorEnd_flag=0;
	mollifier_timer=0;
	noGoalPillarMode=0;
	straight_acceleration_lpf=0;

}

/* ファンも電圧で指定に変更(初期電圧に依存させる。) */
void control_fun(float fun_V){
	int duty_fun ;
	if (fun_V >= 0) {
		duty_fun = (int)(FUN_MAX_DUTY * fun_V / g_V_battery_mean);
	}
	else{
		duty_fun = 0;
	}
	if (duty_fun >= FUN_MAX_DUTY) {
		duty_fun = FUN_MAX_DUTY;
	}

	pl_FunMotor_duty(duty_fun);

}

void get_duty(float V_L, float V_R,int *duty_L,int *duty_R) {
//トルクの方向決定
// BATT_MEANをいつか可変にしたい願望
	if (V_L >= 0) {
		pl_L_DriveMotor_mode(MOTOR_FRONT);
		*duty_L = (int) (V_L / g_V_battery_mean * MAXMOTOR);
	}
	else{
		pl_L_DriveMotor_mode(MOTOR_BACK);
	    *duty_L = (int) (-V_L / g_V_battery_mean * MAXMOTOR);
	}
	if (V_R >= 0) {
		pl_R_DriveMotor_mode(MOTOR_FRONT);
		*duty_R = (int) (V_R / g_V_battery_mean * MAXMOTOR);
	}
	else{
		pl_R_DriveMotor_mode(MOTOR_BACK);
	    *duty_R = (int) (-V_R / g_V_battery_mean * MAXMOTOR);
	}
	//*duty_L=*duty_L;
	//*duty_R=*duty_R;
	//XX
	if (*duty_L >= (int)(MAXMOTOR*MAX_DUTY_RATIO)) {
		*duty_L = (int)(MAXMOTOR*MAX_DUTY_RATIO);
		pl_r_blue_LED(ON);
	}
	if (*duty_R >= (int)(MAXMOTOR*MAX_DUTY_RATIO)) {
		*duty_R = (int)(MAXMOTOR*MAX_DUTY_RATIO);
		pl_l_blue_LED(ON);
	}

}



void interupt_DriveMotor(void){
	int duty_L=0, duty_R=0;
	float V_L=0, V_R=0;
	float PID_s,PID_t;
	float PID_w=0;
	float feedforward_straight=0,feedforward_turning=0;



	if (modeacc == 0) {
		g_acc_flag=4;
		g_WallControl_mode=0;
		g_wallCut_mode=0;
		straight_acceleration_lpf=0;

	}
	if (modeacc == 1) {
		g_wallCut_mode=1;
		g_MotorTimCount++;
		straight.displacement += straight.velocity*INTERRUPT_TIME + straight.acceleration*INTERRUPT_TIME*INTERRUPT_TIME/2;
		straight.velocity += straight.acceleration*INTERRUPT_TIME;
		turning.displacement += turning.velocity*INTERRUPT_TIME + turning.acceleration*INTERRUPT_TIME*INTERRUPT_TIME/2;
		turning.velocity += turning.acceleration*INTERRUPT_TIME;
		cal_table(Trapezoid_straight,&straight);
		EncoderGyro_PID(&PID_s,&PID_t,straight.velocity,turning.velocity,turning.displacement);
		straight_acceleration_lpf=0.9 * straight_acceleration_lpf + (1 - 0.9) * straight.acceleration;
		feedforward_const_accel(&feedforward_straight,(E_lpf_speedL+E_lpf_speedR)/2,
				straight_acceleration_lpf,&feedforward_turning,
					angle_speed,turning.acceleration);
		PID_w = calWallConrol();
		V_L = PID_s-PID_t-PID_w+feedforward_straight-feedforward_turning;
		V_R = PID_s+PID_t+PID_w+feedforward_straight+feedforward_turning;
		if(PID_s+feedforward_straight>g_V_battery_mean*MAX_DUTY_RATIO_ST){
			V_L+=g_V_battery_mean*MAX_DUTY_RATIO_ST-(PID_s+feedforward_straight);
			V_R+=g_V_battery_mean*MAX_DUTY_RATIO_ST-(PID_s+feedforward_straight);
		}else if(PID_s+feedforward_straight<-g_V_battery_mean*MAX_DUTY_RATIO_ST){
			V_L+=-g_V_battery_mean*MAX_DUTY_RATIO_ST-(PID_s+feedforward_straight);
			V_R+=-g_V_battery_mean*MAX_DUTY_RATIO_ST-(PID_s+feedforward_straight);
		}

		get_duty(V_L, V_R,&duty_L,&duty_R);
		pl_DriveMotor_duty(duty_L,duty_R);
	}
	if (modeacc == 2 || modeacc == 4) {//旋回とスラローム
		g_WallControl_mode=0;
		g_wallCut_mode=0;
		g_MotorTimCount++;
		straight.displacement += straight.velocity*INTERRUPT_TIME + straight.acceleration*INTERRUPT_TIME*INTERRUPT_TIME/2;
		straight.velocity += straight.acceleration*INTERRUPT_TIME;
		turning.displacement += turning.velocity*INTERRUPT_TIME + turning.acceleration*INTERRUPT_TIME*INTERRUPT_TIME/2;
		turning.velocity += turning.acceleration*INTERRUPT_TIME;
		cal_table(Trapezoid_turning,&turning);
		EncoderGyro_PID(&PID_s,&PID_t,straight.velocity,turning.velocity,turning.displacement);
		feedforward_const_accel(&feedforward_straight,(E_lpf_speedL+E_lpf_speedR)/2,
				straight.acceleration,&feedforward_turning,
					angle_speed,turning.acceleration);
		V_L = PID_s-PID_t+feedforward_straight-feedforward_turning;
		V_R = PID_s+PID_t+feedforward_straight+feedforward_turning;
		if(PID_s+feedforward_straight>g_V_battery_mean*MAX_DUTY_RATIO_ST){
			V_L+=g_V_battery_mean*MAX_DUTY_RATIO_ST-(PID_s+feedforward_straight);
			V_R+=g_V_battery_mean*MAX_DUTY_RATIO_ST-(PID_s+feedforward_straight);
		}else if(PID_s+feedforward_straight<-g_V_battery_mean*MAX_DUTY_RATIO_ST){
			V_L+=-g_V_battery_mean*MAX_DUTY_RATIO_ST-(PID_s+feedforward_straight);
			V_R+=-g_V_battery_mean*MAX_DUTY_RATIO_ST-(PID_s+feedforward_straight);
		}
		get_duty(V_L, V_R,&duty_L,&duty_R);
		pl_DriveMotor_duty(duty_L,duty_R);
	}
	if (modeacc == 3) {//宴会芸
		g_WallControl_mode=0;
				g_wallCut_mode=0;
		g_MotorTimCount++;
		straight.displacement += straight.velocity*INTERRUPT_TIME + straight.acceleration*INTERRUPT_TIME*INTERRUPT_TIME/2;
		straight.velocity += straight.acceleration*INTERRUPT_TIME;
		turning.displacement += turning.velocity*INTERRUPT_TIME + turning.acceleration*INTERRUPT_TIME*INTERRUPT_TIME/2;
		turning.velocity += turning.acceleration*INTERRUPT_TIME;
		EncoderGyro_PID(&PID_s,&PID_t,straight.velocity,turning.velocity,turning.displacement);
		V_L = PID_s-PID_t+feedforward_straight-feedforward_turning;
		V_R = PID_s+PID_t+feedforward_straight+feedforward_turning;
		get_duty(V_L, V_R,&duty_L,&duty_R);
		pl_DriveMotor_duty(duty_L,duty_R);
	}if (modeacc == 5) {//前壁制御
		g_WallControl_mode=0;
				g_wallCut_mode=0;
		calFrontWallConrol(&V_L,&V_R);
		//EncoderGyro_PID(&PID_s,&PID_t,0,0);
		//V_L = PID_s-PID_t+feedforward_straight-feedforward_turning;
		//V_R = PID_s+PID_t+feedforward_straight+feedforward_turning;
		get_duty(V_L, V_R,&duty_L,&duty_R);
		pl_DriveMotor_duty(duty_L,duty_R);
	}if (modeacc == 6 || modeacc == 9) {//ネイピア加速
		g_WallControl_mode=0;
				g_wallCut_mode=0;
		straight.displacement += straight.velocity*INTERRUPT_TIME + straight.acceleration*INTERRUPT_TIME*INTERRUPT_TIME/2;
		straight.velocity += straight.acceleration*INTERRUPT_TIME;
		turning.displacement += turning.velocity*INTERRUPT_TIME;// + turning.acceleration*INTERRUPT_TIME*INTERRUPT_TIME/2;
		cal_mollifier_table(Mollifier_turning,&turning);//角速度と角加速度はここで決定
		EncoderGyro_PID(&PID_s,&PID_t,straight.velocity,turning.velocity,turning.displacement);
		feedforward_const_accel(&feedforward_straight,(E_lpf_speedL+E_lpf_speedR)/2,
				straight.acceleration,&feedforward_turning,
					angle_speed,turning.acceleration);
		V_L = PID_s-PID_t+feedforward_straight-feedforward_turning;
		V_R = PID_s+PID_t+feedforward_straight+feedforward_turning;
//		if(PID_s+feedforward_straight>g_V_battery_mean*MAX_DUTY_RATIO_ST){
//			V_L+=g_V_battery_mean*MAX_DUTY_RATIO_ST-(PID_s+feedforward_straight);
//			V_R+=g_V_battery_mean*MAX_DUTY_RATIO_ST-(PID_s+feedforward_straight);
//		}else if(PID_s+feedforward_straight<-g_V_battery_mean*MAX_DUTY_RATIO_ST){
//			V_L+=-g_V_battery_mean*MAX_DUTY_RATIO_ST-(PID_s+feedforward_straight);
//			V_R+=-g_V_battery_mean*MAX_DUTY_RATIO_ST-(PID_s+feedforward_straight);
//		}
		get_duty(V_L, V_R,&duty_L,&duty_R);
		pl_DriveMotor_duty(duty_L,duty_R);

	}if (modeacc == 7) {
		g_wallCut_mode=1;
		g_MotorTimCount++;
		straight.displacement += straight.velocity*INTERRUPT_TIME + straight.acceleration*INTERRUPT_TIME*INTERRUPT_TIME/2;
		straight.velocity += straight.acceleration*INTERRUPT_TIME;
		turning.displacement += turning.velocity*INTERRUPT_TIME + turning.acceleration*INTERRUPT_TIME*INTERRUPT_TIME/2;
		turning.velocity += turning.acceleration*INTERRUPT_TIME;
		cal_table_dis(Trapezoid_straight,&straight);
		EncoderGyro_PID(&PID_s,&PID_t,straight.velocity,turning.velocity,turning.displacement);
		feedforward_const_accel(&feedforward_straight,(E_lpf_speedL+E_lpf_speedR)/2,
				straight.acceleration,&feedforward_turning,
					angle_speed,turning.acceleration);
		PID_w = calWallConrol();
		V_L = PID_s-PID_t-PID_w+feedforward_straight-feedforward_turning;
		V_R = PID_s+PID_t+PID_w+feedforward_straight+feedforward_turning;
		if(PID_s+feedforward_straight>g_V_battery_mean*MAX_DUTY_RATIO_ST){
			V_L+=g_V_battery_mean*MAX_DUTY_RATIO_ST-(PID_s+feedforward_straight);
			V_R+=g_V_battery_mean*MAX_DUTY_RATIO_ST-(PID_s+feedforward_straight);
		}else if(PID_s+feedforward_straight<-g_V_battery_mean*MAX_DUTY_RATIO_ST){
			V_L+=-g_V_battery_mean*MAX_DUTY_RATIO_ST-(PID_s+feedforward_straight);
			V_R+=-g_V_battery_mean*MAX_DUTY_RATIO_ST-(PID_s+feedforward_straight);
		}
		get_duty(V_L, V_R,&duty_L,&duty_R);
		pl_DriveMotor_duty(duty_L,duty_R);
	}if (modeacc == 8) {
		g_wallCut_mode=1;
		g_MotorTimCount++;

		straight.displacement += straight.velocity*INTERRUPT_TIME + straight.acceleration*INTERRUPT_TIME*INTERRUPT_TIME/2;
		straight.velocity += straight.acceleration*INTERRUPT_TIME;

		turning.displacement += turning.velocity*INTERRUPT_TIME + turning.acceleration*INTERRUPT_TIME*INTERRUPT_TIME/2;
		turning.velocity += turning.acceleration*INTERRUPT_TIME;
		cal_table_max(Trapezoid_straight,&straight);
		EncoderGyro_PID(&PID_s,&PID_t,straight.velocity,turning.velocity,turning.displacement);
		feedforward_const_accel(&feedforward_straight,(E_lpf_speedL+E_lpf_speedR)/2,
				straight.acceleration,&feedforward_turning,
					angle_speed,turning.acceleration);
		PID_w = calWallConrol();
		V_L = PID_s-PID_t-PID_w+feedforward_straight-feedforward_turning;
		V_R = PID_s+PID_t+PID_w+feedforward_straight+feedforward_turning;
		if(PID_s+feedforward_straight>g_V_battery_mean*MAX_DUTY_RATIO_ST){
			V_L+=g_V_battery_mean*MAX_DUTY_RATIO_ST-(PID_s+feedforward_straight);
			V_R+=g_V_battery_mean*MAX_DUTY_RATIO_ST-(PID_s+feedforward_straight);
		}else if(PID_s+feedforward_straight<-g_V_battery_mean*MAX_DUTY_RATIO_ST){
			V_L+=-g_V_battery_mean*MAX_DUTY_RATIO_ST-(PID_s+feedforward_straight);
			V_R+=-g_V_battery_mean*MAX_DUTY_RATIO_ST-(PID_s+feedforward_straight);
		}
		get_duty(V_L, V_R,&duty_L,&duty_R);
		pl_DriveMotor_duty(duty_L,duty_R);
	}
	if (modeacc == 100) {
		straight.velocity = 0;
		turning.velocity=0;
		EncoderGyro_PID(&PID_s,&PID_t,straight.velocity,turning.velocity,turning.displacement);
		V_L = PID_s-PID_t+feedforward_straight-feedforward_turning;
		V_R = PID_s+PID_t+feedforward_straight+feedforward_turning;
		get_duty(V_L, V_R,&duty_L,&duty_R);
		pl_DriveMotor_duty(duty_L,duty_R);
	}
		g_V_L=(float)(V_L);//V_L;
		g_V_R=(float)(V_R);//V_R;



}

float straight_table_dis(float input_displacement, float input_start_velocity,
	float input_end_velocity, float input_count_velocity, float input_acceleration, float input_deceleration,MOTOR_MODE motor_mode) {

	float MinRequired_displacement=
			(input_end_velocity*input_end_velocity
					-input_start_velocity*input_start_velocity
					)/2/input_acceleration;
	reset_distance();

	// 例外処理
	if (input_acceleration < 0){input_acceleration=-input_acceleration;}//加速が負
	if (input_deceleration < 0){input_deceleration=-input_deceleration;}//減速が負


	Trapezoid_straight.displacement = input_displacement;
	Trapezoid_straight.start_velocity = input_start_velocity;
	Trapezoid_straight.end_velocity = input_end_velocity;
	Trapezoid_straight.count_velocity = input_count_velocity;
	Trapezoid_straight.acceleration = input_acceleration;
	Trapezoid_straight.deceleration = input_deceleration;

	if (input_count_velocity>=0){straight.acceleration = input_acceleration;
	}else{straight.acceleration = -input_acceleration;}
	straight.velocity = input_start_velocity;
	straight.displacement = 0;
	turning.velocity = 0;
	turning.acceleration = 0;
	turning.displacement = 0;
	g_MotorEnd_flag=0;
	g_acc_flag=1;
		if (input_displacement>0 && MinRequired_displacement>input_displacement){g_acc_flag=5;straight.acceleration = input_acceleration;}
		if (input_displacement>0 && MinRequired_displacement<-input_displacement){g_acc_flag=6;straight.acceleration = -input_acceleration;}
		if (input_displacement<0 && MinRequired_displacement>-input_displacement){g_acc_flag=5;straight.acceleration = -input_acceleration;}
		if (input_displacement<0 && MinRequired_displacement<input_displacement){g_acc_flag=6;straight.acceleration = input_acceleration;}
	straight_acceleration_lpf=straight.acceleration;
	modeacc = 7;
	g_WallControl_mode=motor_mode.WallControlMode;
	pl_DriveMotor_start();

	while (g_acc_flag!=4){


	}
	if(input_end_velocity==0){//BREAK
		wait_ms_NoReset(100);
		modeacc = 0;
		pl_R_DriveMotor_mode(MOTOR_BREAK);
		pl_L_DriveMotor_mode(MOTOR_BREAK);
		pl_DriveMotor_stop();//これは必要か？
		wait_ms_NoReset(100);

	}
//	modeacc = 0;

	E_distanceL = E_distanceL - input_displacement;
	E_distanceR = E_distanceR - input_displacement;




	return straight.velocity;



}

float straight_table_max(float input_displacement, float input_start_velocity,
	float input_end_velocity, float input_count_velocity, float input_acceleration, float input_deceleration,MOTOR_MODE motor_mode) {

	float MinRequired_displacement=
			(input_end_velocity*input_end_velocity
					-input_start_velocity*input_start_velocity
					)/2/input_acceleration;
	reset_distance();

	// 例外処理
	if (input_acceleration < 0){input_acceleration=-input_acceleration;}//加速が負
	if (input_deceleration < 0){input_deceleration=-input_deceleration;}//減速が負


	Trapezoid_straight.displacement = input_displacement;
	Trapezoid_straight.start_velocity = input_start_velocity;
	Trapezoid_straight.end_velocity = input_end_velocity;
	Trapezoid_straight.count_velocity = input_count_velocity;
	Trapezoid_straight.acceleration = input_acceleration;
	Trapezoid_straight.deceleration = input_deceleration;

	if (input_count_velocity>=0){straight.acceleration = input_acceleration;
	}else{straight.acceleration = -input_acceleration;}
	straight.velocity = input_start_velocity;
	straight.displacement = 0;
	turning.velocity = 0;
	turning.acceleration = 0;
	turning.displacement = 0;
	g_MotorEnd_flag=0;
	g_acc_flag=1;
		if (input_displacement>0 && MinRequired_displacement>input_displacement){g_acc_flag=5;straight.acceleration = input_acceleration;}
		if (input_displacement>0 && MinRequired_displacement<-input_displacement){g_acc_flag=6;straight.acceleration = -input_acceleration;}
		if (input_displacement<0 && MinRequired_displacement>-input_displacement){g_acc_flag=5;straight.acceleration = -input_acceleration;}
		if (input_displacement<0 && MinRequired_displacement<input_displacement){g_acc_flag=6;straight.acceleration = input_acceleration;}
		straight_acceleration_lpf=straight.acceleration;
		modeacc = 8;
	g_WallControl_mode=motor_mode.WallControlMode;
	pl_DriveMotor_start();

	while (g_acc_flag!=4){


	}
	if(input_end_velocity==0){//BREAK
		wait_ms_NoReset(100);
		modeacc = 0;
		pl_R_DriveMotor_mode(MOTOR_BREAK);
		pl_L_DriveMotor_mode(MOTOR_BREAK);
		pl_DriveMotor_stop();//これは必要か？
		wait_ms_NoReset(100);

	}
//	modeacc = 0;

	E_distanceL = E_distanceL - input_displacement;
	E_distanceR = E_distanceR - input_displacement;




	return straight.velocity;



}



float straight_table2(float input_displacement, float input_start_velocity,
	float input_end_velocity, float input_count_velocity, float input_acceleration,MOTOR_MODE motor_mode) {

	float MinRequired_displacement=
			(input_end_velocity*input_end_velocity
					-input_start_velocity*input_start_velocity
					)/2/input_acceleration;
	// 例外処理
	if (input_acceleration < 0){input_acceleration=-input_acceleration;}//加速が負

	if(noGoalPillarMode==1 && motor_mode.WallCutMode==1){
		motor_mode.WallCutMode=0;
		input_displacement=input_displacement-MAZE_OFFSET;
	}


	Trapezoid_straight.displacement = input_displacement;
	Trapezoid_straight.start_velocity = input_start_velocity;
	Trapezoid_straight.end_velocity = input_end_velocity;
	Trapezoid_straight.count_velocity = input_count_velocity;
	Trapezoid_straight.acceleration = input_acceleration;

	NoWallDisplacementR_safe = 0;//壁切れ安全機能
	NoWallDisplacementL_safe = 0;//壁切れ安全機能
	Nowall_safe_flg = 1;

	if (input_count_velocity>=0){straight.acceleration = input_acceleration;
	}else{straight.acceleration = -input_acceleration;}
	straight.velocity = input_start_velocity;
	straight.displacement = 0;
	turning.velocity = 0;
	turning.acceleration = 0;
	turning.displacement = 0;
	g_MotorEnd_flag=0;
	g_acc_flag=1;
		if (input_displacement>0 && MinRequired_displacement>input_displacement){g_acc_flag=5;straight.acceleration = input_acceleration;}
		if (input_displacement>0 && MinRequired_displacement<-input_displacement){g_acc_flag=6;straight.acceleration = -input_acceleration;}
		if (input_displacement<0 && MinRequired_displacement>-input_displacement){g_acc_flag=5;straight.acceleration = -input_acceleration;}
		if (input_displacement<0 && MinRequired_displacement<input_displacement){g_acc_flag=6;straight.acceleration = input_acceleration;}
		//straight_acceleration_lpf=straight.acceleration;
		modeacc = 1;
	g_WallControl_mode=motor_mode.WallControlMode;
	pl_DriveMotor_start();
	if(motor_mode.WallCutMode==1){
		//左壁or右壁がstart～endの中にあれば抜ける
		g_acc_flag=0;
		straight.acceleration = 0;
		while((NoWallDisplacementR90<CUTPLACE_TO_CENTER_R90 ||
				NoWallDisplacementR90>CUTPLACE_THRESHOLD_END_R90) &&
			  (NoWallDisplacementL90<CUTPLACE_TO_CENTER_L90 ||
			  NoWallDisplacementL90>CUTPLACE_THRESHOLD_END_L90) &&
			  front_wall_break_90==0){}
		enc.sigma_error=0;
		straight.displacement=0;
		if (input_count_velocity>=0){straight.acceleration = input_acceleration;
			}else{straight.acceleration = -input_acceleration;}
		g_acc_flag=1;
		if (input_displacement>0 && MinRequired_displacement>input_displacement){g_acc_flag=5;straight.acceleration = input_acceleration;}
		if (input_displacement>0 && MinRequired_displacement<-input_displacement){g_acc_flag=6;straight.acceleration = -input_acceleration;}
		if (input_displacement<0 && MinRequired_displacement>-input_displacement){g_acc_flag=5;straight.acceleration = -input_acceleration;}
		if (input_displacement<0 && MinRequired_displacement<input_displacement){g_acc_flag=6;straight.acceleration = input_acceleration;}
	}else if(motor_mode.WallCutMode==2){
		//左壁or右壁がstart～endの中にあれば抜ける
		g_acc_flag=0;
		straight.acceleration = 0;
		while((NoWallDisplacementR45<=0 || NoWallDisplacementR45<=CUTPLACE_TO_CENTER_R45 ||
				NoWallDisplacementR45>=CUTPLACE_THRESHOLD_END_R45) &&
			  (NoWallDisplacementL45<=0 || NoWallDisplacementL45<=CUTPLACE_TO_CENTER_L45 ||
			  NoWallDisplacementL45>=CUTPLACE_THRESHOLD_END_L45) &&
			  front_wall_break_45==0){}
//		while((NoWallDisplacementR45<=CUTPLACE_TO_CENTER_R45 ||
//				NoWallDisplacementR45>CUTPLACE_THRESHOLD_END_R45) &&
//			  (NoWallDisplacementL45<=CUTPLACE_TO_CENTER_L45 ||
//			  NoWallDisplacementL45>CUTPLACE_THRESHOLD_END_L45) &&
//			  front_wall_break_45==0){}
//		while((NoWallDisplacementR90<=0 ||
//				NoWallDisplacementR90>CUTPLACE_THRESHOLD_END_R45) &&
//				(NoWallDisplacementL90<=0 ||
//			  NoWallDisplacementL90>CUTPLACE_THRESHOLD_END_L45) &&
//			  front_wall_break_90==0){}
		if (input_count_velocity>=0){straight.acceleration = input_acceleration;
			}else{straight.acceleration = -input_acceleration;}
		g_acc_flag=1;
		if (input_displacement>0 && MinRequired_displacement>input_displacement){g_acc_flag=5;straight.acceleration = input_acceleration;}
		if (input_displacement>0 && MinRequired_displacement<-input_displacement){g_acc_flag=6;straight.acceleration = -input_acceleration;}
		if (input_displacement<0 && MinRequired_displacement>-input_displacement){g_acc_flag=5;straight.acceleration = -input_acceleration;}
		if (input_displacement<0 && MinRequired_displacement<input_displacement){g_acc_flag=6;straight.acceleration = input_acceleration;}
	}else if(motor_mode.WallCutMode==3){
		//斜めの右旋回
		//左壁or右壁がstart～endの中にあれば抜ける

		g_acc_flag=0;
		straight.acceleration = 0;
		while((NoWallDisplacementR45slant2<CUTPLACE_TO_CENTER_R45_SLANT ||
				NoWallDisplacementR45slant2>CUTPLACE_THRESHOLD_END_R45_SLANT) &&
				  front_wall_break_45slant==0){}
		if (input_count_velocity>=0){straight.acceleration = input_acceleration;
			}else{straight.acceleration = -input_acceleration;}
		g_acc_flag=1;
		if (input_displacement>0 && MinRequired_displacement>input_displacement){g_acc_flag=5;straight.acceleration = input_acceleration;}
		if (input_displacement>0 && MinRequired_displacement<-input_displacement){g_acc_flag=6;straight.acceleration = -input_acceleration;}
		if (input_displacement<0 && MinRequired_displacement>-input_displacement){g_acc_flag=5;straight.acceleration = -input_acceleration;}
		if (input_displacement<0 && MinRequired_displacement<input_displacement){g_acc_flag=6;straight.acceleration = input_acceleration;}
	}else if(motor_mode.WallCutMode==4){
		//斜めの左旋回
		//左壁or右壁がstart～endの中にあれば抜ける

		g_acc_flag=0;
		straight.acceleration = 0;
		while((NoWallDisplacementL45slant2<CUTPLACE_TO_CENTER_L45_SLANT ||
			  NoWallDisplacementL45slant2>CUTPLACE_THRESHOLD_END_L45_SLANT) &&
				  front_wall_break_45slant==0){}
		if (input_count_velocity>=0){straight.acceleration = input_acceleration;
			}else{straight.acceleration = -input_acceleration;}
		g_acc_flag=1;
		if (input_displacement>0 && MinRequired_displacement>input_displacement){g_acc_flag=5;straight.acceleration = input_acceleration;}
		if (input_displacement>0 && MinRequired_displacement<-input_displacement){g_acc_flag=6;straight.acceleration = -input_acceleration;}
		if (input_displacement<0 && MinRequired_displacement>-input_displacement){g_acc_flag=5;straight.acceleration = -input_acceleration;}
		if (input_displacement<0 && MinRequired_displacement<input_displacement){g_acc_flag=6;straight.acceleration = input_acceleration;}
	}

	Nowall_safe_flg = 0;//壁切れ距離カウントOFF
	NoWallDisplacementR_safe = 0;//壁切れ安全機能
	NoWallDisplacementL_safe = 0;//壁切れ安全機能

	if(motor_mode.calMazeMode==0){
	while (g_acc_flag!=4){
		if(motor_mode.WallCutMode==1){
			if((NoWallDisplacementL90>=input_displacement+CUTPLACE_TO_CENTER_L90 ||
			   NoWallDisplacementR90>=input_displacement+CUTPLACE_TO_CENTER_R90)){
				g_acc_flag=4;
				break;
			}
		}else if(motor_mode.WallCutMode==2){
			if(NoWallDisplacementL45>=input_displacement+CUTPLACE_TO_CENTER_L45 &&
			   NoWallDisplacementR45>=input_displacement+CUTPLACE_TO_CENTER_R45 ){
				g_acc_flag=4;
				break;
			}
		}else if(motor_mode.WallCutMode==3){
			if(NoWallDisplacementR45slant2>=input_displacement+CUTPLACE_TO_CENTER_R45_SLANT ){
				g_acc_flag=4;
				break;
			}
		}else if(motor_mode.WallCutMode==4){
			if(NoWallDisplacementL45slant2>=input_displacement+CUTPLACE_TO_CENTER_L45_SLANT){
				g_acc_flag=4;
				break;
			}
		}

	}
	if(input_end_velocity==0){//BREAK
		wait_ms_NoReset(100);
		modeacc = 0;
		pl_R_DriveMotor_mode(MOTOR_BREAK);
		pl_L_DriveMotor_mode(MOTOR_BREAK);
		pl_DriveMotor_stop();//これは必要か？
		wait_ms_NoReset(100);
	}
//	modeacc = 0;

	E_distanceL = E_distanceL - input_displacement;
	E_distanceR = E_distanceR - input_displacement;


	}

	return straight.velocity;



}

void End_straight(float input_displacement,MOTOR_MODE motor_mode,_Bool right_wall,_Bool left_wall){
	if(noGoalPillarMode==1){
		motor_mode.WallCutMode=0;
	}
	while (g_acc_flag!=4){
		if(right_wall == 0 || left_wall == 0){
		if(motor_mode.WallCutMode==1){
					if(NoWallDisplacementL90>=input_displacement+CUTPLACE_TO_CENTER_L90 ||
					   NoWallDisplacementR90>=input_displacement+CUTPLACE_TO_CENTER_R90 ){
						g_acc_flag=4;
						break;
					}
				}
		}
//		else if(motor_mode.WallCutMode==2){
//					if(NoWallDisplacementL45>=input_displacement+CUTPLACE_TO_CENTER_L45 ||
//					   NoWallDisplacementR45>=input_displacement+CUTPLACE_TO_CENTER_R45 ){
//						g_acc_flag=4;
//						break;
//			}
//		}

	}
//	while(g_MotorEnd_flag==0){}
	E_distanceL = E_distanceL - input_displacement;
	E_distanceR = E_distanceR - input_displacement;
	pl_DriveMotor_stop();//これは必要か？
}

float turning_table2(float input_displacement, float input_start_velocity,
	float input_end_velocity, float input_count_velocity, float input_acceleration) {

	float MinRequired_displacement=
			(input_end_velocity*input_end_velocity
					-input_start_velocity*input_start_velocity
					)/2/input_acceleration;
	// 例外処理
	if (input_acceleration < 0){input_acceleration=-input_acceleration;}//加速が負

	yaw_angle=0;

	Trapezoid_turning.displacement = input_displacement;
	Trapezoid_turning.start_velocity = input_start_velocity;
	Trapezoid_turning.end_velocity = input_end_velocity;
	Trapezoid_turning.count_velocity = input_count_velocity;
	Trapezoid_turning.acceleration = input_acceleration;

	if (input_count_velocity>=0){turning.acceleration = input_acceleration;
	}else{turning.acceleration = -input_acceleration;}
	turning.velocity = input_start_velocity;
	turning.displacement = 0;
	straight.velocity = 0;
	straight.acceleration = 0;
	straight.displacement = 0;
	g_MotorEnd_flag=0;
	g_acc_flag=1;
		if (input_displacement>0 && MinRequired_displacement>input_displacement){g_acc_flag=5;turning.acceleration = input_acceleration;}
		if (input_displacement>0 && MinRequired_displacement<-input_displacement){g_acc_flag=6;turning.acceleration = -input_acceleration;}
		if (input_displacement<0 && MinRequired_displacement>-input_displacement){g_acc_flag=5;turning.acceleration = -input_acceleration;}
		if (input_displacement<0 && MinRequired_displacement<input_displacement){g_acc_flag=6;turning.acceleration = input_acceleration;}
	modeacc = 2;

	pl_DriveMotor_start();
	while (g_acc_flag!=4){

	}
	if(input_end_velocity==0){//BREAK
		wait_ms_NoReset(300);
		modeacc = 0;
		pl_R_DriveMotor_mode(MOTOR_BREAK);
		pl_L_DriveMotor_mode(MOTOR_BREAK);
		wait_ms_NoReset(100);
	}
//	modeacc = 0;

	yaw_angle = yaw_angle - input_displacement;

	pl_DriveMotor_stop();
	fusion_speedL = E_speedL;
	fusion_speedR = E_speedR;
	gf_speed=0;

	return turning.velocity;
}



float slalom_table2(float input_center_velocity,float input_displacement, float input_start_velocity,
	float input_end_velocity, float input_count_velocity, float input_acceleration) {

	float MinRequired_displacement=
			(input_end_velocity*input_end_velocity
					-input_start_velocity*input_start_velocity
					)/2/input_acceleration;
	// 例外処理
	if (input_acceleration < 0){input_acceleration=-input_acceleration;}//加速が負

	//yaw_angle=0;

	Trapezoid_turning.displacement = input_displacement;
	Trapezoid_turning.start_velocity = input_start_velocity;
	Trapezoid_turning.end_velocity = input_end_velocity;
	Trapezoid_turning.count_velocity = input_count_velocity;
	Trapezoid_turning.acceleration = input_acceleration;

	if (input_count_velocity>=0){turning.acceleration = input_acceleration;
	}else{turning.acceleration = -input_acceleration;}
	turning.velocity = input_start_velocity;
	turning.displacement = 0;
	straight.velocity = input_center_velocity;
	straight.acceleration = 0;
	straight.displacement = 0;
	g_MotorEnd_flag=0;
	g_acc_flag=1;
		if (input_displacement>0 && MinRequired_displacement>input_displacement){g_acc_flag=5;turning.acceleration = input_acceleration;}
		if (input_displacement>0 && MinRequired_displacement<-input_displacement){g_acc_flag=6;turning.acceleration = -input_acceleration;}
		if (input_displacement<0 && MinRequired_displacement>-input_displacement){g_acc_flag=5;turning.acceleration = -input_acceleration;}
		if (input_displacement<0 && MinRequired_displacement<input_displacement){g_acc_flag=6;turning.acceleration = input_acceleration;}
	modeacc = 4;
//	enc.sigma_error=0;
	pl_DriveMotor_start();
	while (g_acc_flag!=4){

	}
//	modeacc = 0;
//	enc.sigma_error=0;

	yaw_angle = yaw_angle - input_displacement;

	pl_DriveMotor_stop();

	return turning.velocity;
}

void no_angle(void){
	turning.acceleration = 0;
	turning.velocity = 0;
	turning.displacement = 0;
	straight.velocity = 0;
	straight.acceleration = 0;
	straight.displacement = 0;

	modeacc = 3;

	pl_DriveMotor_start();
	while (g_sensor[0][0] <= SENSOR_FINGER_0 || g_sensor[2][0] <= SENSOR_FINGER_2 || g_sensor[4][0] <= SENSOR_FINGER_4) {
		HAL_Delay(1);
//		if(record_time >= max_record_time){
//			break;
//		}
	}
	modeacc = 0;

	pl_DriveMotor_stop();
	fusion_speedL = E_speedL;
	fusion_speedR = E_speedR;
	gf_speed=0;

}

void mollifier_turning_table(float input_displacement, float input_max_turning_velocity) {

	// 例外処理

	Mollifier_turning.center_velocity = 0;
	Mollifier_turning.displacement = input_displacement;
	Mollifier_turning.max_turning_velocity = input_max_turning_velocity;

	turning.velocity = 0;
	turning.displacement = 0;
	straight.velocity = 0;
	straight.acceleration = 0;
	straight.displacement = 0;
	g_MotorEnd_flag=0;
	g_acc_flag=1;
	mollifier_timer=-fabs(input_displacement)/MOLLIFIER_INTEGRAL*exp(-1)/input_max_turning_velocity;
	modeacc = 9;

	pl_DriveMotor_start();
	while (g_acc_flag!=4){

	}

	wait_ms_NoReset(100);
	modeacc = 0;
	pl_R_DriveMotor_mode(MOTOR_BREAK);
	pl_L_DriveMotor_mode(MOTOR_BREAK);
	wait_ms_NoReset(50);


	

//	modeacc = 0;

	enc.sigma_error=0;
	yaw_angle = yaw_angle - input_displacement;
	fusion_speedL = E_speedL;
	fusion_speedR = E_speedR;
	gf_speed=0;


	pl_DriveMotor_stop();
}

void mollifier_slalom_table(float input_center_velocity,float input_displacement, float input_max_turning_velocity) {

	// 例外処理

	Mollifier_turning.center_velocity = input_center_velocity;
	Mollifier_turning.displacement = input_displacement;
	Mollifier_turning.max_turning_velocity = input_max_turning_velocity;


	turning.velocity = 0;
	turning.displacement = 0;
	straight.velocity = input_center_velocity;
	straight.acceleration = 0;
	straight.displacement = 0;
	g_MotorEnd_flag=0;
	g_acc_flag=1;
	mollifier_timer=-fabs(input_displacement)/MOLLIFIER_INTEGRAL*exp(-1)/input_max_turning_velocity;
	modeacc = 6;

	pl_DriveMotor_start();
	while (g_acc_flag!=4){

	}
	if(input_center_velocity==0){//BREAK
		wait_ms_NoReset(300);
		modeacc = 0;
		pl_R_DriveMotor_mode(MOTOR_BREAK);
		pl_L_DriveMotor_mode(MOTOR_BREAK);
		wait_ms_NoReset(100);
	}

	if(input_center_velocity<=500){//BREAK
		enc.sigma_error=0;
	}
//	modeacc = 0;


	yaw_angle = yaw_angle - input_displacement;

	pl_DriveMotor_stop();

}


void no_frontwall_straight(void){
	turning.acceleration = 0;
	turning.velocity = 0;
	turning.displacement = 0;
	straight.velocity = 0;
	straight.acceleration = 0;
	straight.displacement = 0;

	g_FrontWallControl_mode=1;
	modeacc = 5;

	pl_DriveMotor_start();
	wait_ms_NoReset(300);
	// int count=0;
	// while((fabsf(g_sensor[SENSOR_FRONT_L][0] - CENTER_FRONT_L)<50 && fabsf(g_sensor[SENSOR_FRONT_R][0] - CENTER_FRONT_R)<50) || count>1000)
	// {
	// 	count++;
	// 	wait_ms_NoReset(1);
	// }

	g_FrontWallControl_mode=0;
	modeacc = 0;

	pl_DriveMotor_stop();
	fusion_speedL = E_speedL;
	fusion_speedR = E_speedR;
	gf_speed=0;
	//yaw_angle=0;
	//clear_Ierror();

}




float straight_table_ff(float input_displacement, float input_start_velocity,
	float input_end_velocity, float input_count_velocity, float input_acceleration) {

	float MinRequired_displacement=
			(input_end_velocity*input_end_velocity
					-input_start_velocity*input_start_velocity
					)/2/input_acceleration;
	// 例外処理
	if (input_acceleration < 0){input_acceleration=-input_acceleration;}//加速が負
	Ksp = 0;//3//P項の制御量直進*****************************************************
	Ksi = 0;//5//43//I項の制御量直進*****************************************************
	Ksd = 0;//D項の制御量直進*****************************************************
	Ktp = 0;//295//P項の制御量旋回*****************************************************
	Kti = 0;//1//.6//I項の制御量旋回*****************************************************
//	Ktifun = 0.01;//1//.6//I項の制御量旋回*****************************************************
	Ktd = 0;


	Trapezoid_straight.displacement = input_displacement;
	Trapezoid_straight.start_velocity = input_start_velocity;
	Trapezoid_straight.end_velocity = input_end_velocity;
	Trapezoid_straight.count_velocity = input_count_velocity;
	Trapezoid_straight.acceleration = input_acceleration;

	if (input_count_velocity>=0){straight.acceleration = input_acceleration;
	}else{straight.acceleration = -input_acceleration;}
	straight.velocity = input_start_velocity;
	straight.displacement = 0;
	turning.velocity = 0;
	turning.acceleration = 0;
	turning.displacement = 0;
	g_MotorEnd_flag=0;
	g_acc_flag=1;
		if (input_displacement>0 && MinRequired_displacement>input_displacement){g_acc_flag=5;straight.acceleration = input_acceleration;}
		if (input_displacement>0 && MinRequired_displacement<-input_displacement){g_acc_flag=6;straight.acceleration = -input_acceleration;}
		if (input_displacement<0 && MinRequired_displacement>-input_displacement){g_acc_flag=5;straight.acceleration = -input_acceleration;}
		if (input_displacement<0 && MinRequired_displacement<input_displacement){g_acc_flag=6;straight.acceleration = input_acceleration;}
	modeacc = 1;
	pl_DriveMotor_start();


	while (g_acc_flag!=4){


	}
	if(input_end_velocity==0){//BREAK
		wait_ms_NoReset(100);
		modeacc = 0;
	}
//	modeacc = 0;

	E_distanceL = E_distanceL - input_displacement;
	E_distanceR = E_distanceR - input_displacement;
	pl_DriveMotor_stop();//これは必要か？



	return straight.velocity;



}



float turning_table_ff(float input_displacement, float input_start_velocity,
	float input_end_velocity, float input_count_velocity, float input_acceleration) {

	float MinRequired_displacement=
			(input_end_velocity*input_end_velocity
					-input_start_velocity*input_start_velocity
					)/2/input_acceleration;
	// 例外処理
	if (input_acceleration < 0){input_acceleration=-input_acceleration;}//加速が負

	Ksp = 0;//3//P項の制御量直進*****************************************************
		Ksi = 0;//5//43//I項の制御量直進*****************************************************
		Ksd = 0;//D項の制御量直進*****************************************************
		Ktp = 0;//295//P項の制御量旋回*****************************************************
		Kti = 0;//1//.6//I項の制御量旋回*****************************************************
	//	Ktifun = 0.01;//1//.6//I項の制御量旋回*****************************************************
		Ktd = 0;

	Trapezoid_turning.displacement = input_displacement;
	Trapezoid_turning.start_velocity = input_start_velocity;
	Trapezoid_turning.end_velocity = input_end_velocity;
	Trapezoid_turning.count_velocity = input_count_velocity;
	Trapezoid_turning.acceleration = input_acceleration;

	if (input_count_velocity>=0){turning.acceleration = input_acceleration;
	}else{turning.acceleration = -input_acceleration;}
	turning.velocity = input_start_velocity;
	turning.displacement = 0;
	straight.velocity = 0;
	straight.acceleration = 0;
	straight.displacement = 0;
	g_MotorEnd_flag=0;
	g_acc_flag=1;
		if (input_displacement>0 && MinRequired_displacement>input_displacement){g_acc_flag=5;turning.acceleration = input_acceleration;}
		if (input_displacement>0 && MinRequired_displacement<-input_displacement){g_acc_flag=6;turning.acceleration = -input_acceleration;}
		if (input_displacement<0 && MinRequired_displacement>-input_displacement){g_acc_flag=5;turning.acceleration = -input_acceleration;}
		if (input_displacement<0 && MinRequired_displacement<input_displacement){g_acc_flag=6;turning.acceleration = input_acceleration;}
	modeacc = 2;

	pl_DriveMotor_start();
	while (g_acc_flag!=4){

	}
	if(input_end_velocity==0){//BREAK
		wait_ms_NoReset(300);
		modeacc = 0;
	}
//	modeacc = 0;

	yaw_angle = yaw_angle - input_displacement;

	pl_DriveMotor_stop();

	return turning.velocity;
}





