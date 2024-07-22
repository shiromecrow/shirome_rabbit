/*
 * fail_safe.c
 *
 *  Created on: 2023/01/20
 *      Author: sf199
 */


#include "fail_safe.h"
#include "CL_EnoderGyro.h"
#include "Control_motor.h"
#include "PID_EncoderGyro.h"
#include"CL_sensor.h"

#include "PL_LED.h"
#include "PL_motor.h"
#include "PID_wall.h"

#include "record.h"
//#include"maze_strategy.h"
//#include"maze_wall.h"
#include "mode_select.h"
#include "math.h"

char no_safty;
unsigned char error_mode;
char highspeed_mode;
float encoder_PID_error;
float gyro_PID_error;
float gyro_x_error;
float encoder_gyro_error;
float encoder_PID_error_highspeed;
float gyro_PID_error_highspeed;
float gyro_x_error_highspeed;
float encoder_gyro_error_highspeed;
int error_count1,error_count2,error_count3,error_count4,error_count5;//エラー回数
int error_time_count;//エラ後のやつ

void init_FailSafe(void){
	error_mode=0;
	no_safty=0;
	highspeed_mode = 0;
	error_count1=0;
	error_count2=0;
	error_count3=0;
	error_count4=0;
	error_count5=0;
	error_time_count=0;
	encoder_PID_error=500;
	gyro_PID_error=500;
	gyro_x_error=300;
	encoder_gyro_error=1000;

	encoder_PID_error_highspeed=500;//3000
	gyro_PID_error_highspeed=800;
	gyro_x_error_highspeed=400;
	encoder_gyro_error_highspeed=2500;
}


void interrupt_FailSafe(void){
	float encoder_PID_error_in;
	float gyro_PID_error_in;
	float gyro_x_error_in;
	float encoder_gyro_error_in;
//	float wallcut_error_in;

	if (highspeed_mode == 0) {
		encoder_PID_error_in=encoder_PID_error;
		gyro_PID_error_in=gyro_PID_error;
		gyro_x_error_in=gyro_x_error;
		encoder_gyro_error_in=encoder_gyro_error;
	}else{
		encoder_PID_error_in=encoder_PID_error_highspeed;
		gyro_PID_error_in=gyro_PID_error_highspeed;
		gyro_x_error_in=gyro_x_error_highspeed;
		encoder_gyro_error_in=encoder_gyro_error_highspeed;
	}
//	wallcut_error_in=1000;

	if (modeacc != 0 && modeacc != 3){
		if (no_safty == 0 && error_mode == 0) {
			//ジャイロの誤差が一定以上
					if (fabs(turning.velocity - angle_speed) >= gyro_PID_error_in ) {
						error_count1++;
						if(error_count1>=90){
							pl_FunMotor_stop();
							g_WallControl_mode =0;
							pl_yellow_LED_count(1);
							main_mode=1;
							error_mode = 1;
							clear_Ierror();
						}
					}else{
						error_count1=0;
					}
					if (angle_speedx_set >= gyro_x_error_in) {
						error_count2++;
						if(error_count2>=40){
							pl_FunMotor_stop();
							g_WallControl_mode =0;
							pl_yellow_LED_count(2);
							error_mode = 2;
							main_mode=2;
							clear_Ierror();
						}
					}else{
						error_count2=0;
					}
					if(fabs(straight.velocity - gf_speed) >= encoder_gyro_error_in && modeacc==1){
						error_count3++;
						if(error_count3>=500){
							pl_FunMotor_stop();
							g_WallControl_mode =0;
							pl_yellow_LED_count(4);
							main_mode=4;
							error_mode = 3;
							clear_Ierror();
						}
					}else{
						error_count3=0;
					}

					if(fabs(straight.velocity - (E_speedR+E_speedL)/2) >= encoder_PID_error_in && modeacc==1){
						error_count4++;
						if(error_count4>=40){
							pl_FunMotor_stop();
							g_WallControl_mode =0;
							pl_yellow_LED_count(8);
							main_mode=8;
							error_mode = 4;
							clear_Ierror();
						}
					}else{
						error_count4=0;
					}

					if(fabs(gf_speed - (E_speedR+E_speedL)/2) >= encoder_gyro_error_in && modeacc==1){
						error_count5++;
						if(error_count5>=400){
							pl_FunMotor_stop();
							g_WallControl_mode =0;
							pl_yellow_LED_count(16);
							main_mode=16;
							error_mode = 5;
							clear_Ierror();
						}
					}else{
						error_count5=0;
					}

//					if(NoWallDisplacementL45>wallcut_error_in || NoWallDisplacementR45>wallcut_error_in || NoWallDisplacementL90>wallcut_error_in || NoWallDisplacementR90>wallcut_error_in || NoWallDisplacementL45slant>wallcut_error_in || NoWallDisplacementR45slant>wallcut_error_in){
//						error_count5++;
//						if(error_count5>=50){
//							pl_FunMotor_stop();
//							g_WallControl_mode =0;
//							pl_yellow_LED_count(16);
//							main_mode=16;
//							error_mode = 5;
//							clear_Ierror();
//						}
//					}else{
//						error_count5=0;
//					}

				}

	}

	if(error_mode>=1){

		pl_yellow_LED_count(error_mode);

		record_mode=0;
		error_time_count++;
		if(error_time_count<=1000){
			init_WallControl();
			modeacc=100;//エラー用
			//pl_R_DriveMotor_mode(MOTOR_BREAK);
			//pl_L_DriveMotor_mode(MOTOR_BREAK);
		}else{
			pl_DriveMotor_standby(OFF);
			pl_DriveMotor_stop();
			pl_FunMotor_stop();
			modeacc=0;
		}
					NoWallCountL90 = 4294967295;
					NoWallCountR90 = 4294967295;
					NoWallCountL45 = 4294967295;
					NoWallCountR45 = 4294967295;
					NoWallCountL45slant = 4294967295;
					NoWallCountR45slant = 4294967295;
					NoWallDisplacementL90 = 50;
					NoWallDisplacementR90 = 50;
					NoWallDisplacementL45 = 20;
					NoWallDisplacementR45 = 20;
					NoWallDisplacementL45slant = 35;
					NoWallDisplacementR45slant = 35;
					NoWallDisplacementL45slant2 = 35;
					NoWallDisplacementR45slant2 = 35;
					g_acc_flag=4;
					g_wallCut_mode = 0;
					//maze_mode=0;

	}else{
		error_time_count=0;
	}


}
