/*
 * PL_motor.h
 *
 *  Created on: Dec 23, 2023
 *      Author: sf199
 */

#ifndef INC_PL_MOTOR_H_
#define INC_PL_MOTOR_H_

#define MOTOR_STOP 0
#define MOTOR_FRONT 1
#define MOTOR_BACK 2
#define MOTOR_BREAK 3

#define L_MOTOR_FRONT 1
#define L_MOTOR_BACK 0
#define R_MOTOR_FRONT 0
#define R_MOTOR_BACK 1

#define FUN_MAX_DUTY 200

void pl_motor_init();

void pl_DriveMotor_standby(int);

void pl_L_DriveMotor_mode(int);
void pl_R_DriveMotor_mode(int);

void pl_DriveMotor_start(void);
void pl_DriveMotor_stop(void);
void pl_DriveMotor_duty(int,int);


void pl_FunMotor_start(void);
void pl_FunMotor_stop(void);
void pl_FunMotor_duty(int);



#endif /* INC_PL_MOTOR_H_ */
