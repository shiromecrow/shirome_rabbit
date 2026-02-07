/*
 * Control_motor.h
 *
 *  Created on: Jan 14, 2023
 *      Author: sf199
 */

#ifndef INC_CONTROL_MOTOR_H_
#define INC_CONTROL_MOTOR_H_

#include <stdint.h>
#include "cal_acceleration.h"

#define MAX_DUTY_RATIO 0.999
#define MAX_DUTY_RATIO_ST 0.999

#define TURNAFTER_WALLCONTROLOFF_RATIO 0.7

typedef struct {
	uint8_t WallControlMode;//0で壁制御なし、1で通常の壁制御、2で斜めの制御
	uint8_t WallControlStatus;
	uint8_t calMazeMode;
	uint8_t WallCutMode;
	//uint8_T BreakMode;

}MOTOR_MODE;

extern TARGET straight;
extern TARGET turning;

extern TRAPEZOID Trapezoid_straight;
extern TRAPEZOID Trapezoid_turning;

extern MOLLIFIER Mollifier_turning;

extern float straight_acceleration_lpf;

extern float g_V_L,g_V_R,g_Vol1,g_Vol2;

extern float V_cmd_sysid;

extern char modeacc;
extern uint8_t noGoalPillarMode;

void Control_mode_Init();
void start_fun(float);
void stop_fun(void);
void control_fun(float);
void get_duty(float , float ,int *,int *);

void interupt_DriveMotor();

void End_straight(float,MOTOR_MODE,_Bool,_Bool);
float straight_table2(float,float,float,float,float,MOTOR_MODE);
float straight_table_dis(float,float,float,float,float,float,MOTOR_MODE);
float straight_table_max(float,float,float,float,float,float,MOTOR_MODE);
float turning_table2(float,float,float,float,float);
float slalom_table2(float,float,float,float,float,float);
void mollifier_turning_table(float, float);
void mollifier_slalom_table(float,float, float,char);

void no_frontwall_straight();

float straight_table_ff(float,float,float,float,float);
float turning_table_ff(float,float,float,float,float);

void no_angle();


#endif /* INC_CONTROL_MOTOR_H_ */
