/*
 * maze_Turning.c
 *
 *  Created on: 2023/01/31
 *      Author: sf199
 */


/*
 * turning.c
 *
 *  Created on: 2019/10/02
 *      Author: sf199
 */
#include "maze_Turning.h"
#include "Control_motor.h"
#include "PL_motor.h"
#include "fail_safe.h"
#include "PL_timer.h"
#include "define.h"
#include "record.h"
#include "CL_EnoderGyro.h"
#include "PID_EncoderGyro.h"
#include"maze_strategy.h"

#include "math.h"





void test_mollifier_slalomR(parameter turnpara) {
	MOTOR_MODE wallmode;
		highspeed_mode = 1;
		wallmode.WallControlMode=1;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=0;
		wallmode.calMazeMode=0;
			straight_table2(BACK_TO_CENTER_FRONT + 135, 0, turnpara.g_speed, turnpara.g_speed,
					turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		wallmode.WallCutMode=0;
		wallmode.WallControlMode=0;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
									turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		mollifier_slalom_table(turnpara.g_speed,-90,turnpara.t_speed);
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=0;
		straight_table2(45 + turnpara.e_ofset, turnpara.g_speed, 0, turnpara.g_speed,
						turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		highspeed_mode = 0;
}



void backTurn_hitWall(float input_TurningVelocity,float input_TurningAcceleration,_Bool front_wall,_Bool left_wall,_Bool right_wall){
	MOTOR_MODE wallmode;
	wallmode.WallControlMode=0;
	wallmode.WallControlStatus=0;
	wallmode.WallCutMode=0;
	wallmode.calMazeMode=0;
	if(front_wall){
		turning_table2(180,0,0,input_TurningVelocity,input_TurningAcceleration);
		pl_R_DriveMotor_mode(MOTOR_BREAK);
		pl_L_DriveMotor_mode(MOTOR_BREAK);
		wait_ms_NoReset(500);
		straight_table2(-90, 0,0,-200,3000, wallmode);
		pl_R_DriveMotor_mode(MOTOR_BREAK);
		pl_L_DriveMotor_mode(MOTOR_BREAK);
	}else{
		turning_table2(180,0,0,input_TurningVelocity,input_TurningAcceleration);
		pl_R_DriveMotor_mode(MOTOR_BREAK);
		pl_L_DriveMotor_mode(MOTOR_BREAK);
		wait_ms_NoReset(500);
		straight_table2(-BACK_TO_CENTER, 0,0,-200,3000, wallmode);
		pl_R_DriveMotor_mode(MOTOR_BREAK);
		pl_L_DriveMotor_mode(MOTOR_BREAK);
	}
	wait_ms_NoReset(150);

}

void backTurn_controlWall(float input_TurningVelocity,float input_TurningAcceleration,_Bool front_wall,_Bool left_wall,_Bool right_wall){
	if(front_wall){
		no_safty = 1;
		no_frontwall_straight();
		no_safty = 0;
		pl_R_DriveMotor_mode(MOTOR_BREAK);
		pl_L_DriveMotor_mode(MOTOR_BREAK);
		wait_ms_NoReset(50);
	}
	if(left_wall){
		wait_ms_NoReset(50);
		//turning_table2(90,0,0,input_TurningVelocity,input_TurningAcceleration);
		mollifier_turning_table(90,input_TurningVelocity);
		pl_R_DriveMotor_mode(MOTOR_BREAK);
		pl_L_DriveMotor_mode(MOTOR_BREAK);
		wait_ms_NoReset(50);
		no_safty = 1;
		no_frontwall_straight();
		no_safty = 0;
		pl_R_DriveMotor_mode(MOTOR_BREAK);
		pl_L_DriveMotor_mode(MOTOR_BREAK);
		wait_ms_NoReset(50);
		//turning_table2(90,0,0,input_TurningVelocity,input_TurningAcceleration);
		mollifier_turning_table(90,input_TurningVelocity);
	}else if(left_wall==0 && right_wall){
		wait_ms_NoReset(50);
		//turning_table2(-90,0,0,-input_TurningVelocity,input_TurningAcceleration);
		mollifier_turning_table(-90,input_TurningVelocity);
		pl_R_DriveMotor_mode(MOTOR_BREAK);
		pl_L_DriveMotor_mode(MOTOR_BREAK);
		wait_ms_NoReset(50);
		no_safty = 1;
		no_frontwall_straight();
		no_safty = 0;
		pl_R_DriveMotor_mode(MOTOR_BREAK);
		pl_L_DriveMotor_mode(MOTOR_BREAK);
		wait_ms_NoReset(50);
		//turning_table2(-90,0,0,-input_TurningVelocity,input_TurningAcceleration);
		mollifier_turning_table(-90,input_TurningVelocity);
	}else if(left_wall==0 && right_wall==0){
		wait_ms_NoReset(50);
		//turning_table2(90,0,0,input_TurningVelocity,input_TurningAcceleration);
		mollifier_turning_table(90,input_TurningVelocity);
		pl_R_DriveMotor_mode(MOTOR_BREAK);
		pl_L_DriveMotor_mode(MOTOR_BREAK);
		wait_ms_NoReset(50);
		//turning_table2(90,0,0,input_TurningVelocity,input_TurningAcceleration);
		mollifier_turning_table(90,input_TurningVelocity);
	}
	pl_R_DriveMotor_mode(MOTOR_BREAK);
	pl_L_DriveMotor_mode(MOTOR_BREAK);
	wait_ms_NoReset(100);
}


void slalomR(parameter turnpara,char test_mode,char shortest_mode,char mollifier_mode,float end_velocity) {
	MOTOR_MODE wallmode;
	if (test_mode == ON) {
		highspeed_mode = 0;
		wallmode.WallControlMode=1;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=0;
		wallmode.calMazeMode=0;
		if(shortest_mode==0){
			straight_table2(BACK_TO_CENTER_FRONT + 135, 0, turnpara.g_speed, turnpara.g_speed,
					turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		}else{
			straight_table2(BACK_TO_CENTER_FRONT + 135, 0, turnpara.g_speed, turnpara.g_speed,
					turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		}
		wallmode.WallCutMode=1;
		wallmode.WallControlMode=0;
		if(shortest_mode==0){
			straight_table2(MAZE_OFFSET+turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
									turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		}else{
			straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
									turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		}
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,-90,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,-90, 0, 0, -turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=0;
		straight_table2(45 + turnpara.e_ofset, turnpara.g_speed, 0, turnpara.g_speed,
						turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		highspeed_mode = 0;
	} else {
		wallmode.WallControlMode=OFFSET_CONTROL_IN_SLALOM;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=1;
		wallmode.calMazeMode=0;
		if(shortest_mode==0){
			straight_table2(MAZE_OFFSET+turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
														turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		}else{
			straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
														turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		}
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,-90,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,-90, 0, 0, -turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=OFFSET_CONTROL_OUT_SLALOM;
		wallmode.WallCutMode=0;
		straight_table2(turnpara.e_ofset, turnpara.g_speed, end_velocity, end_velocity,
								fabs(turnpara.g_speed * turnpara.g_speed-end_velocity*end_velocity)  / 2 / turnpara.e_ofset+1000,wallmode);
//		straight_table2(turnpara.e_ofset, turnpara.g_speed, end_velocity, end_velocity,
//										fabs(end_velocity*end_velocity-turnpara.g_speed * turnpara.g_speed)  / 2 / turnpara.e_ofset,wallmode);
	}
}

void slalomL(parameter turnpara,char test_mode,char shortest_mode,char mollifier_mode,float end_velocity) {
	MOTOR_MODE wallmode;
	if (test_mode == ON) {
		highspeed_mode = 0;
		wallmode.WallControlMode=1;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=0;
		wallmode.calMazeMode=0;
		if(shortest_mode==0){
			straight_table2(BACK_TO_CENTER_FRONT + 135, 0, turnpara.g_speed, turnpara.g_speed,
					turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		}else{
			straight_table2(BACK_TO_CENTER_FRONT + 135, 0, turnpara.g_speed, turnpara.g_speed,
					turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		}
		wallmode.WallControlMode=OFFSET_CONTROL_IN_SLALOM;
		wallmode.WallCutMode=1;
		if(shortest_mode==0){
			straight_table2(MAZE_OFFSET+turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
									turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		}else{
			straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
									turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		}
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,90,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,90, 0, 0, turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=OFFSET_CONTROL_OUT_SLALOM;
		wallmode.WallCutMode=0;
		straight_table2(45 + turnpara.e_ofset, turnpara.g_speed, 0, turnpara.g_speed,
						turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		highspeed_mode = 0;
	} else {
		wallmode.WallControlMode=0;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=1;
		wallmode.calMazeMode=0;
		if(shortest_mode==0){
			straight_table2(MAZE_OFFSET+turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
														turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		}else{
			straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
														turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		}
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,90,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,90, 0, 0, turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=0;
		straight_table2(turnpara.e_ofset, turnpara.g_speed, end_velocity, end_velocity,
								fabs(turnpara.g_speed * turnpara.g_speed-end_velocity*end_velocity)  / 2 / turnpara.e_ofset+1000,wallmode);

	}
}


void turn90R(parameter turnpara, char test_mode,char mollifier_mode,float end_velocity) {
	MOTOR_MODE wallmode;
	if (test_mode == 1) {
		highspeed_mode = 1;
		wallmode.WallControlMode=1;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=0;
		wallmode.calMazeMode=0;
		straight_table2(BACK_TO_CENTER_FRONT + 90, 0, turnpara.g_speed, turnpara.g_speed,
						turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=2;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,-90,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,-90, 0, 0, -turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=0;
		straight_table2(90 + turnpara.e_ofset, turnpara.g_speed, 0, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		highspeed_mode = 0;
	} else {
		wallmode.WallControlMode=OFFSET_CONTROL_IN;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=2;
		wallmode.calMazeMode=0;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,-90,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,-90, 0, 0, -turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=OFFSET_CONTROL_OUT;
		wallmode.WallCutMode=0;
		straight_table2(turnpara.e_ofset, turnpara.g_speed, end_velocity, end_velocity,
								fabs(turnpara.g_speed * turnpara.g_speed-end_velocity*end_velocity)  / 2 / turnpara.e_ofset+1000,wallmode);

	}

}

void turn90L(parameter turnpara, char test_mode,char mollifier_mode,float end_velocity) {
	MOTOR_MODE wallmode;
	if (test_mode == 1) {
		highspeed_mode = 1;
		wallmode.WallControlMode=1;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=0;
		wallmode.calMazeMode=0;
		straight_table2(BACK_TO_CENTER_FRONT + 90, 0, turnpara.g_speed, turnpara.g_speed,
						turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=2;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,90,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,90, 0, 0, turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=0;
		straight_table2(90 + turnpara.e_ofset, turnpara.g_speed, 0, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		highspeed_mode = 0;
	} else {
		wallmode.WallControlMode=OFFSET_CONTROL_IN;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=2;
		wallmode.calMazeMode=0;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,90,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,90, 0, 0, turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=OFFSET_CONTROL_OUT;
		wallmode.WallCutMode=0;
		straight_table2(turnpara.e_ofset, turnpara.g_speed, end_velocity, end_velocity,
								fabs(turnpara.g_speed * turnpara.g_speed-end_velocity*end_velocity)  / 2 / turnpara.e_ofset+1000,wallmode);

	}

}


void turn180R(parameter turnpara, char test_mode,char mollifier_mode,float end_velocity) {
	MOTOR_MODE wallmode;
	if (test_mode == 1) {
		highspeed_mode = 1;
		wallmode.WallControlMode=1;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=0;
		wallmode.calMazeMode=0;
		straight_table2(BACK_TO_CENTER_FRONT + 90, 0, turnpara.g_speed, turnpara.g_speed,
						turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=2;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,-180,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,-180, 0, 0, -turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=0;
		straight_table2(90 + turnpara.e_ofset, turnpara.g_speed, 0, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		highspeed_mode = 0;
	} else {
		wallmode.WallControlMode=OFFSET_CONTROL_IN;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=2;
		wallmode.calMazeMode=0;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,-180,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,-180, 0, 0, -turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=OFFSET_CONTROL_OUT;
		wallmode.WallCutMode=0;
		straight_table2(turnpara.e_ofset, turnpara.g_speed, end_velocity, end_velocity,
								fabs(turnpara.g_speed * turnpara.g_speed-end_velocity*end_velocity)  / 2 / turnpara.e_ofset+1000,wallmode);

	}

}

void turn180L(parameter turnpara, char test_mode,char mollifier_mode,float end_velocity) {
	MOTOR_MODE wallmode;
	if (test_mode == 1) {
		highspeed_mode = 1;
		wallmode.WallControlMode=1;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=0;
		wallmode.calMazeMode=0;
		straight_table2(BACK_TO_CENTER_FRONT + 90, 0, turnpara.g_speed, turnpara.g_speed,
						turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=2;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,180,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,180, 0, 0, turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=0;
		straight_table2(90 + turnpara.e_ofset, turnpara.g_speed, 0, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		highspeed_mode = 0;
	} else {
		wallmode.WallControlMode=OFFSET_CONTROL_IN;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=2;
		wallmode.calMazeMode=0;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,180,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,180, 0, 0, turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=OFFSET_CONTROL_OUT;
		wallmode.WallCutMode=0;
		straight_table2(turnpara.e_ofset, turnpara.g_speed, end_velocity, end_velocity,
								fabs(turnpara.g_speed * turnpara.g_speed-end_velocity*end_velocity)  / 2 / turnpara.e_ofset+1000,wallmode);

	}

}



void turn45inR(parameter turnpara, char test_mode,char mollifier_mode,float end_velocity) {
	MOTOR_MODE wallmode;
	if (test_mode == 1) {
		highspeed_mode = 1;
		wallmode.WallControlMode=1;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=0;
		wallmode.calMazeMode=0;
		straight_table2(BACK_TO_CENTER_FRONT + 90, 0, turnpara.g_speed, turnpara.g_speed,
					turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=2;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,-45,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,-45, 0, 0, -turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=0;
		straight_table2(90*sqrt(2) + turnpara.e_ofset, turnpara.g_speed, 0, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		highspeed_mode = 0;
	}
	if (test_mode == 0) {
		wallmode.WallControlMode=OFFSET_CONTROL_IN;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=2;
		wallmode.calMazeMode=0;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,-45,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,-45, 0, 0, -turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=OFFSET_CONTROL_OUT_SLANT;
		wallmode.WallCutMode=0;
		straight_table2(turnpara.e_ofset, turnpara.g_speed, end_velocity, end_velocity,
								fabs(turnpara.g_speed * turnpara.g_speed-end_velocity*end_velocity)  / 2 / turnpara.e_ofset+1000,wallmode);

	}
	if (test_mode >= 2) {

		highspeed_mode = 1;
		wallmode.WallControlMode=1;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=0;
		wallmode.calMazeMode=0;
		straight_table2(BACK_TO_CENTER_FRONT + 90, 0, turnpara.g_speed, turnpara.g_speed,
					turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=2;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,-45,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,-45, 0, 0, -turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=0;
		straight_table2(45*sqrt(2)*(test_mode - 2) + turnpara.e_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		highspeed_mode = 0;
	}

}

void turn45inL(parameter turnpara, char test_mode,char mollifier_mode,float end_velocity) {
	MOTOR_MODE wallmode;
	if (test_mode == 1) {
		highspeed_mode = 1;
		wallmode.WallControlMode=1;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=0;
		wallmode.calMazeMode=0;
		straight_table2(BACK_TO_CENTER_FRONT + 90, 0, turnpara.g_speed, turnpara.g_speed,
					turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=2;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,45,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,45, 0, 0, turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=0;
		straight_table2(90*sqrt(2) + turnpara.e_ofset, turnpara.g_speed, 0, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		highspeed_mode = 0;
	}
	if (test_mode == 0) {
		wallmode.WallControlMode=OFFSET_CONTROL_IN;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=2;
		wallmode.calMazeMode=0;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,45,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,45, 0, 0, turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=OFFSET_CONTROL_OUT_SLANT;
		wallmode.WallCutMode=0;
		straight_table2(turnpara.e_ofset, turnpara.g_speed, end_velocity, end_velocity,
								fabs(turnpara.g_speed * turnpara.g_speed-end_velocity*end_velocity)  / 2 / turnpara.e_ofset+1000,wallmode);

	}
	if (test_mode >= 2) {

		highspeed_mode = 1;
		wallmode.WallControlMode=1;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=0;
		wallmode.calMazeMode=0;
		straight_table2(BACK_TO_CENTER_FRONT + 90, 0, turnpara.g_speed, turnpara.g_speed,
					turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=2;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,45,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,45, 0, 0, turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=0;
		straight_table2(45*sqrt(2)*(test_mode - 2) + turnpara.e_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		highspeed_mode = 0;
	}

}

void turn135inR(parameter turnpara, char test_mode,char mollifier_mode,float end_velocity) {
	MOTOR_MODE wallmode;
	if (test_mode == 1) {
		highspeed_mode = 1;
		wallmode.WallControlMode=1;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=0;
		wallmode.calMazeMode=0;
		straight_table2(BACK_TO_CENTER_FRONT + 90, 0, turnpara.g_speed, turnpara.g_speed,
					turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=2;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,-135,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,-135, 0, 0, -turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=0;
		straight_table2(90*sqrt(2) + turnpara.e_ofset, turnpara.g_speed, 0, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		highspeed_mode = 0;
	}
	if (test_mode == 0) {
		wallmode.WallControlMode=OFFSET_CONTROL_IN;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=2;
		wallmode.calMazeMode=0;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,-135,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,-135, 0, 0, -turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=OFFSET_CONTROL_OUT_SLANT;
		wallmode.WallCutMode=0;
		straight_table2(turnpara.e_ofset, turnpara.g_speed, end_velocity, end_velocity,
								fabs(turnpara.g_speed * turnpara.g_speed-end_velocity*end_velocity)  / 2 / turnpara.e_ofset+1000,wallmode);

	}
	if (test_mode >= 2) {

		highspeed_mode = 1;
		wallmode.WallControlMode=1;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=0;
		wallmode.calMazeMode=0;
		straight_table2(BACK_TO_CENTER_FRONT + 90, 0, turnpara.g_speed, turnpara.g_speed,
					turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=2;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,-135,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,-135, 0, 0, -turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=0;
		straight_table2(45*sqrt(2)*(test_mode - 2) + turnpara.e_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		highspeed_mode = 0;
	}

}

void turn135inL(parameter turnpara, char test_mode,char mollifier_mode,float end_velocity) {
	MOTOR_MODE wallmode;
	if (test_mode == 1) {
		highspeed_mode = 1;
		wallmode.WallControlMode=1;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=0;
		wallmode.calMazeMode=0;
		straight_table2(BACK_TO_CENTER_FRONT + 90, 0, turnpara.g_speed, turnpara.g_speed,
					turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=2;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,135,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,135, 0, 0, turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=0;
		straight_table2(90*sqrt(2) + turnpara.e_ofset, turnpara.g_speed, 0, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		highspeed_mode = 0;
	}
	if (test_mode == 0) {
		wallmode.WallControlMode=OFFSET_CONTROL_IN;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=2;
		wallmode.calMazeMode=0;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,135,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,135, 0, 0, turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=OFFSET_CONTROL_OUT_SLANT;
		wallmode.WallCutMode=0;
		straight_table2(turnpara.e_ofset, turnpara.g_speed, end_velocity, end_velocity,
								fabs(turnpara.g_speed * turnpara.g_speed-end_velocity*end_velocity)  / 2 / turnpara.e_ofset+1000,wallmode);

	}
	if (test_mode >= 2) {

		highspeed_mode = 1;
		wallmode.WallControlMode=1;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=0;
		wallmode.calMazeMode=0;
		straight_table2(BACK_TO_CENTER_FRONT + 90, 0, turnpara.g_speed, turnpara.g_speed,
					turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=2;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,135,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,135, 0, 0, turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=0;
		straight_table2(45*sqrt(2)*(test_mode - 2) + turnpara.e_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		highspeed_mode = 0;
	}

}


void turn45outR(parameter turnpara,  char test_mode,char mollifier_mode,float end_velocity) {
	MOTOR_MODE wallmode;

	if (test_mode == 1) {
		highspeed_mode = 1;
		wallmode.WallControlMode=0;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=0;
		wallmode.calMazeMode=0;
		straight_table2(BACK_TO_CENTER_FRONT_SLANT + MAZE_SECTION/2*sqrt(2), 0, turnpara.g_speed, turnpara.g_speed,
					turnpara.g_speed * turnpara.g_speed  / 2 / (MAZE_SECTION/2),wallmode);
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=3;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / (MAZE_SECTION/2),wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,-45,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,-45, 0, 0, -turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=0;
		straight_table2(MAZE_SECTION + turnpara.e_ofset, turnpara.g_speed, 0, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / (MAZE_SECTION/2),wallmode);
		highspeed_mode = 0;
	}else if (test_mode >= 2) {
		highspeed_mode = 1;
		wallmode.WallControlMode=0;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=3;
		wallmode.calMazeMode=0;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,-45,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,-45, 0, 0, -turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=0;
		straight_table2(90 + turnpara.e_ofset, turnpara.g_speed, 0, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		highspeed_mode = 0;
	}else {
		wallmode.WallControlMode=OFFSET_CONTROL_IN_SLANT;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=3;
		wallmode.calMazeMode=0;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,-45,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,-45, 0, 0, -turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=OFFSET_CONTROL_OUT;
		wallmode.WallCutMode=0;
		straight_table2(turnpara.e_ofset, turnpara.g_speed, end_velocity, end_velocity,
								fabs(turnpara.g_speed * turnpara.g_speed-end_velocity*end_velocity)  / 2 / turnpara.e_ofset+1000,wallmode);

	}

}


void turn45outL(parameter turnpara,  char test_mode,char mollifier_mode,float end_velocity) {
	MOTOR_MODE wallmode;
	if (test_mode == 1) {
			highspeed_mode = 1;
			wallmode.WallControlMode=0;
			wallmode.WallControlStatus=0;
			wallmode.WallCutMode=0;
			wallmode.calMazeMode=0;
			straight_table2(BACK_TO_CENTER_FRONT_SLANT + MAZE_SECTION/2*sqrt(2), 0, turnpara.g_speed, turnpara.g_speed,
						turnpara.g_speed * turnpara.g_speed  / 2 / (MAZE_SECTION/2),wallmode);
			wallmode.WallControlMode=0;
			wallmode.WallCutMode=4;
			straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
					turnpara.g_speed * turnpara.g_speed  / 2 / (MAZE_SECTION/2),wallmode);
			if(mollifier_mode == ON){
				mollifier_slalom_table(turnpara.g_speed,45,turnpara.t_speed);
			}else{
				slalom_table2(turnpara.g_speed,45, 0, 0, turnpara.t_speed, turnpara.t_acc);
			}
			wallmode.WallControlMode=0;
			wallmode.WallCutMode=0;
			straight_table2(MAZE_SECTION + turnpara.e_ofset, turnpara.g_speed, 0, turnpara.g_speed,
					turnpara.g_speed * turnpara.g_speed  / 2 / (MAZE_SECTION/2),wallmode);
			highspeed_mode = 0;
		}else if (test_mode >= 2) {
		highspeed_mode = 1;
		wallmode.WallControlMode=0;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=4;
		wallmode.calMazeMode=0;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,45,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,45, 0, 0, turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=0;
		straight_table2(90 + turnpara.e_ofset, turnpara.g_speed, 0, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		highspeed_mode = 0;
	} else {
		wallmode.WallControlMode=OFFSET_CONTROL_IN_SLANT;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=4;
		wallmode.calMazeMode=0;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,45,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,45, 0, 0, turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=OFFSET_CONTROL_OUT;
		wallmode.WallCutMode=0;
		straight_table2(turnpara.e_ofset, turnpara.g_speed, end_velocity, end_velocity,
								fabs(turnpara.g_speed * turnpara.g_speed-end_velocity*end_velocity)  / 2 / turnpara.e_ofset+1000,wallmode);

	}

}



void turn135outR(parameter turnpara,  char test_mode,char mollifier_mode,float end_velocity) {
	MOTOR_MODE wallmode;
	if (test_mode == 1) {
			highspeed_mode = 1;
			wallmode.WallControlMode=0;
			wallmode.WallControlStatus=0;
			wallmode.WallCutMode=0;
			wallmode.calMazeMode=0;
			straight_table2(BACK_TO_CENTER_FRONT_SLANT + MAZE_SECTION/2*sqrt(2), 0, turnpara.g_speed, turnpara.g_speed,
						turnpara.g_speed * turnpara.g_speed  / 2 / (MAZE_SECTION/2),wallmode);
			wallmode.WallControlMode=0;
			wallmode.WallCutMode=3;
			straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
					turnpara.g_speed * turnpara.g_speed  / 2 / (MAZE_SECTION/2),wallmode);
			if(mollifier_mode == ON){
				mollifier_slalom_table(turnpara.g_speed,-135,turnpara.t_speed);
			}else{
				slalom_table2(turnpara.g_speed,-135, 0, 0, -turnpara.t_speed, turnpara.t_acc);
			}
			wallmode.WallControlMode=0;
			wallmode.WallCutMode=0;
			straight_table2(MAZE_SECTION + turnpara.e_ofset, turnpara.g_speed, 0, turnpara.g_speed,
					turnpara.g_speed * turnpara.g_speed  / 2 / (MAZE_SECTION/2),wallmode);
			highspeed_mode = 0;
		}else if (test_mode >= 2) {
		highspeed_mode = 1;
		wallmode.WallControlMode=0;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=3;
		wallmode.calMazeMode=0;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,-135,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,-135, 0, 0, -turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=0;
		straight_table2(90 + turnpara.e_ofset, turnpara.g_speed, 0, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		highspeed_mode = 0;
	} else {
		wallmode.WallControlMode=OFFSET_CONTROL_IN_SLANT;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=3;
		wallmode.calMazeMode=0;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,-135,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,-135, 0, 0, -turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=OFFSET_CONTROL_OUT;
		wallmode.WallCutMode=0;
		straight_table2(turnpara.e_ofset, turnpara.g_speed, end_velocity, end_velocity,
								fabs(turnpara.g_speed * turnpara.g_speed-end_velocity*end_velocity)  / 2 / turnpara.e_ofset+1000,wallmode);

	}

}


void turn135outL(parameter turnpara,  char test_mode,char mollifier_mode,float end_velocity) {
	MOTOR_MODE wallmode;
	if (test_mode == 1) {
		highspeed_mode = 1;
		wallmode.WallControlMode=0;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=0;
		wallmode.calMazeMode=0;
		straight_table2(BACK_TO_CENTER_FRONT_SLANT + MAZE_SECTION/2*sqrt(2), 0, turnpara.g_speed, turnpara.g_speed,
					turnpara.g_speed * turnpara.g_speed  / 2 / (MAZE_SECTION/2),wallmode);
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=4;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / (MAZE_SECTION/2),wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,135,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,135, 0, 0, turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=0;
		straight_table2(MAZE_SECTION + turnpara.e_ofset, turnpara.g_speed, 0, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / (MAZE_SECTION/2),wallmode);
		highspeed_mode = 0;
	}else if (test_mode >= 2) {
		highspeed_mode = 1;
		wallmode.WallControlMode=0;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=4;
		wallmode.calMazeMode=0;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,135,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,135, 0, 0, turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=0;
		straight_table2(90 + turnpara.e_ofset, turnpara.g_speed, 0, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		highspeed_mode = 0;
	} else {
		wallmode.WallControlMode=OFFSET_CONTROL_IN_SLANT;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=4;
		wallmode.calMazeMode=0;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,135,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,135, 0, 0, turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=OFFSET_CONTROL_OUT;
		wallmode.WallCutMode=0;
		straight_table2(turnpara.e_ofset, turnpara.g_speed, end_velocity, end_velocity,
								fabs(turnpara.g_speed * turnpara.g_speed-end_velocity*end_velocity)  / 2 / turnpara.e_ofset+1000,wallmode);

	}

}


void V90R(parameter turnpara,  char test_mode,char mollifier_mode,float end_velocity) {
	MOTOR_MODE wallmode;
	if (test_mode == 1) {
			highspeed_mode = 1;
			wallmode.WallControlMode=0;
			wallmode.WallControlStatus=0;
			wallmode.WallCutMode=0;
			wallmode.calMazeMode=0;
			straight_table2(BACK_TO_CENTER_FRONT_SLANT + MAZE_SECTION/2*sqrt(2), 0, turnpara.g_speed, turnpara.g_speed,
						turnpara.g_speed * turnpara.g_speed  / 2 / (MAZE_SECTION/2),wallmode);
			wallmode.WallControlMode=0;
			wallmode.WallCutMode=3;
			straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
					turnpara.g_speed * turnpara.g_speed  / 2 / (MAZE_SECTION/2),wallmode);
			if(mollifier_mode == ON){
				mollifier_slalom_table(turnpara.g_speed,-90,turnpara.t_speed);
			}else{
				slalom_table2(turnpara.g_speed,-90, 0, 0, -turnpara.t_speed, turnpara.t_acc);
			}
			wallmode.WallControlMode=0;
			wallmode.WallCutMode=0;
			straight_table2(MAZE_SECTION*sqrt(2) + turnpara.e_ofset, turnpara.g_speed, 0, turnpara.g_speed,
					turnpara.g_speed * turnpara.g_speed  / 2 / (MAZE_SECTION/2),wallmode);
			highspeed_mode = 0;
		}else if (test_mode >= 2) {
		highspeed_mode = 1;
		wallmode.WallControlMode=0;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=3;
		wallmode.calMazeMode=0;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,-90,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,-90, 0, 0, -turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=0;
		straight_table2(90*sqrt(2) + turnpara.e_ofset, turnpara.g_speed, 0, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		highspeed_mode = 0;
	} else {
		wallmode.WallControlMode=OFFSET_CONTROL_IN_SLANT;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=3;
		wallmode.calMazeMode=0;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,-90,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,-90, 0, 0, -turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=OFFSET_CONTROL_OUT_SLANT;
		wallmode.WallCutMode=0;
		straight_table2(turnpara.e_ofset, turnpara.g_speed, end_velocity, end_velocity,
								fabs(turnpara.g_speed * turnpara.g_speed-end_velocity*end_velocity)  / 2 / turnpara.e_ofset+1000,wallmode);

	}

}

void V90L(parameter turnpara,  char test_mode,char mollifier_mode,float end_velocity) {
	MOTOR_MODE wallmode;
	if (test_mode == 1) {
				highspeed_mode = 1;
				wallmode.WallControlMode=0;
				wallmode.WallControlStatus=0;
				wallmode.WallCutMode=0;
				wallmode.calMazeMode=0;
				straight_table2(BACK_TO_CENTER_SLANT + MAZE_SECTION/2*sqrt(2), 0, turnpara.g_speed, turnpara.g_speed,
							turnpara.g_speed * turnpara.g_speed  / 2 / (MAZE_SECTION/2),wallmode);
				wallmode.WallControlMode=0;
				wallmode.WallCutMode=4;
				straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
						turnpara.g_speed * turnpara.g_speed  / 2 / (MAZE_SECTION/2),wallmode);
				if(mollifier_mode == ON){
					mollifier_slalom_table(turnpara.g_speed,90,turnpara.t_speed);
				}else{
					slalom_table2(turnpara.g_speed,90, 0, 0, turnpara.t_speed, turnpara.t_acc);
				}
				wallmode.WallControlMode=0;
				wallmode.WallCutMode=0;
				straight_table2(MAZE_SECTION*sqrt(2) + turnpara.e_ofset, turnpara.g_speed, 0, turnpara.g_speed,
						turnpara.g_speed * turnpara.g_speed  / 2 / (MAZE_SECTION/2),wallmode);
				highspeed_mode = 0;
			}else if (test_mode >= 2) {
		highspeed_mode = 1;
		wallmode.WallControlMode=0;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=4;
		wallmode.calMazeMode=0;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,90,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,90, 0, 0, turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=0;
		wallmode.WallCutMode=0;
		straight_table2(90*sqrt(2) + turnpara.e_ofset, turnpara.g_speed, 0, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		highspeed_mode = 0;
	} else {
		wallmode.WallControlMode=OFFSET_CONTROL_IN_SLANT;
		wallmode.WallControlStatus=0;
		wallmode.WallCutMode=4;
		wallmode.calMazeMode=0;
		straight_table2(turnpara.f_ofset, turnpara.g_speed, turnpara.g_speed, turnpara.g_speed,
				turnpara.g_speed * turnpara.g_speed  / 2 / 45,wallmode);
		if(mollifier_mode == ON){
			mollifier_slalom_table(turnpara.g_speed,90,turnpara.t_speed);
		}else{
			slalom_table2(turnpara.g_speed,90, 0, 0, turnpara.t_speed, turnpara.t_acc);
		}
		wallmode.WallControlMode=OFFSET_CONTROL_OUT_SLANT;
		wallmode.WallCutMode=0;
		straight_table2(turnpara.e_ofset, turnpara.g_speed, end_velocity, end_velocity,
								fabs(turnpara.g_speed * turnpara.g_speed-end_velocity*end_velocity)  / 2 / turnpara.e_ofset+1000,wallmode);

	}

}


void testturning(parameter_speed Howspeed,int turnmode,char shortest_mode,char funmode,float fun_V,char mollifier_mode){
	//0=slalomR,1=slalomL,2=90R,3=90L,4=180R,5=180L,6=in45R,7=in45L,8=in135R,9=in135L
	//10=out45R,11=out45L,12=out135R,13=out135L,14=V90R,15=V90L
	MOTOR_MODE wallmode;
	wallmode.WallControlMode=0;
	wallmode.WallControlStatus=0;
	wallmode.WallCutMode=0;
	wallmode.calMazeMode=0;
	pl_DriveMotor_standby(ON);
	pl_L_DriveMotor_mode(MOTOR_BREAK);
	pl_R_DriveMotor_mode(MOTOR_BREAK);
	reset_gyro();
	reset_speed();
	reset_distance();
	clear_Ierror();
	if(turnmode==10 || turnmode==12 || turnmode==14){
		highspeed_mode=0;
		straight_table2(BACK_TO_CENTER2+MAZE_SECTION/2,0,0,200*MAZE_SECTION/90,5000*MAZE_SECTION/90, wallmode);
		turning_table2(45, 0, 0, 300*MAZE_SECTION/90, 3000*MAZE_SECTION/90);
		straight_table2(-BACK_TO_CENTER_FRONT_SLANT,0,0,-100*MAZE_SECTION/90,5000*MAZE_SECTION/90, wallmode);
		highspeed_mode=1;
	}
	if(turnmode==11 || turnmode==13 || turnmode==15){
		highspeed_mode=0;
		straight_table2(BACK_TO_CENTER2+MAZE_SECTION/2,0,0,200*MAZE_SECTION/90,5000*MAZE_SECTION/90, wallmode);
		turning_table2(-45, 0, 0, -300*MAZE_SECTION/90, 3000*MAZE_SECTION/90);
		straight_table2(-BACK_TO_CENTER_FRONT_SLANT,0,0,-100*MAZE_SECTION/90,5000*MAZE_SECTION/90, wallmode);
		highspeed_mode=1;
	}

	if(funmode==ON){
		pl_DriveMotor_standby(ON);
		pl_L_DriveMotor_mode(MOTOR_BREAK);
		control_fun(fun_V);
		pl_FunMotor_start();
		HAL_Delay(600);
		//reset_gyro();
		reset_gyro_integral();
		reset_speed();
		reset_distance();
		clear_Ierror();

	}else{
		pl_DriveMotor_standby(ON);
		pl_L_DriveMotor_mode(MOTOR_BREAK);
		reset_gyro();
		reset_speed();
		reset_distance();
		clear_Ierror();

	}
	record_mode = 2;
	//record_mode = 25;
//	if(turnmode==0){test_mollifier_slalomR(Howspeed.slalom_R);}
	if(turnmode==0){slalomR(Howspeed.slalom_R,ON,shortest_mode,mollifier_mode,-100);}
	if(turnmode==1){slalomL(Howspeed.slalom_L,ON,shortest_mode,mollifier_mode,-100);}
	if(turnmode==2){turn90R(Howspeed.turn90_R,ON,mollifier_mode,-100);}
	if(turnmode==3){turn90L(Howspeed.turn90_L,ON,mollifier_mode,-100);}
	if(turnmode==4){turn180R(Howspeed.turn180_R,ON,mollifier_mode,-100);}
	if(turnmode==5){turn180L(Howspeed.turn180_L,ON,mollifier_mode,-100);}
	if(turnmode==6){turn45inR(Howspeed.turn45in_R,ON,mollifier_mode,-100);}
	if(turnmode==7){turn45inL(Howspeed.turn45in_L,ON,mollifier_mode,-100);}
	if(turnmode==8){turn135inR(Howspeed.turn135in_R,ON,mollifier_mode,-100);}
	if(turnmode==9){turn135inL(Howspeed.turn135in_L,ON,mollifier_mode,-100);}
	if(turnmode==10){turn45outR(Howspeed.turn45out_R,ON,mollifier_mode,-100);}
	if(turnmode==11){turn45outL(Howspeed.turn45out_L,ON,mollifier_mode,-100);}
	if(turnmode==12){turn135outR(Howspeed.turn135out_R,ON,mollifier_mode,-100);}
	if(turnmode==13){turn135outL(Howspeed.turn135out_L,ON,mollifier_mode,-100);}
	if(turnmode==14){V90R(Howspeed.V90_R,ON,mollifier_mode,-100);}
	if(turnmode==15){V90L(Howspeed.V90_L,ON,mollifier_mode,-100);}
//	if(turnmode==10){turn45inL(Howspeed.turn45in_L, CONNECT);turn45outR(Howspeed.turn45out_R,CONNECT);}
//	if(turnmode==11){turn45inR(Howspeed.turn45in_R, CONNECT);turn45outL(Howspeed.turn45out_L,CONNECT);}
//	if(turnmode==12){turn135inL(Howspeed.turn135in_L, CONNECT);turn135outR(Howspeed.turn135out_R,CONNECT);}
//	if(turnmode==13){turn135inR(Howspeed.turn135in_R, CONNECT);turn135outL(Howspeed.turn135out_L,CONNECT);}
//	if(turnmode==14){turn45inL(Howspeed.turn45in_L, CONNECT);V90R(Howspeed.V90_R,CONNECT);}
//	if(turnmode==15){turn45inR(Howspeed.turn45in_R, CONNECT);V90L(Howspeed.V90_L,CONNECT);}

	record_mode=0;
	pl_R_DriveMotor_mode(MOTOR_BREAK);
	pl_L_DriveMotor_mode(MOTOR_BREAK);
	wait_ms_NoReset(500);
	pl_FunMotor_stop();
	wait_ms_NoReset(500);
	pl_DriveMotor_standby(OFF);

}
