/*
 * PL_motor.c
 *
 *  Created on: Dec 23, 2023
 *      Author: sf199
 */


/*
 * PL_motor.c
 *
 *  Created on: Jan 12, 2023
 *      Author: sf199
 */


#include "PL_motor.h"
#include "tim.h"
#include "gpio.h"
#include "define.h"

void pl_motor_init(void){
	  HAL_TIM_Base_Start_IT(&htim8);//モータ
	  HAL_TIM_PWM_MspInit(&htim8);//モータ
	  HAL_TIM_Base_Start_IT(&htim16);//吸
	  HAL_TIM_PWM_MspInit(&htim16);//吸

	pl_L_DriveMotor_mode(MOTOR_FRONT);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3,90);
	pl_R_DriveMotor_mode(MOTOR_FRONT);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1,90);

	__HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1,30);
}

void pl_DriveMotor_standby(int pin){
//	HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, pin);
}

void pl_L_DriveMotor_mode(int l_motor_mode){

switch (l_motor_mode){
case MOTOR_STOP:
	//pin_mode変更で実装したい
break;
case MOTOR_FRONT:
	HAL_GPIO_WritePin(MOTOR_L_CWCCW_GPIO_Port,MOTOR_L_CWCCW_Pin,GPIO_PIN_RESET);
break;
case MOTOR_BACK:
	HAL_GPIO_WritePin(MOTOR_L_CWCCW_GPIO_Port,MOTOR_L_CWCCW_Pin,GPIO_PIN_SET);
break;
case MOTOR_BREAK:
	//pin_mode変更で実装したい
break;
}

}

void pl_R_DriveMotor_mode(int r_motor_mode){

switch (r_motor_mode){
case MOTOR_STOP:
	//pin_mode変更で実装したい
break;
case MOTOR_FRONT:
	HAL_GPIO_WritePin(MOTOR_R_CWCCW_GPIO_Port,MOTOR_R_CWCCW_Pin,GPIO_PIN_SET);
break;
case MOTOR_BACK:
	HAL_GPIO_WritePin(MOTOR_R_CWCCW_GPIO_Port,MOTOR_R_CWCCW_Pin,GPIO_PIN_RESET);
break;
case MOTOR_BREAK:
	//pin_mode変更で実装したい
break;
}

}



void pl_DriveMotor_start(void){
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
}

void pl_DriveMotor_stop(void){
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
}

void pl_DriveMotor_duty(int duty_l,int duty_r){
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3,duty_r);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1,duty_l);
}



void pl_FunMotor_start(void){
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
}

void pl_FunMotor_stop(void){
	HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
}

void pl_FunMotor_duty(int duty_fun){
	__HAL_TIM_SET_AUTORELOAD(&htim16, FUN_MAX_DUTY);
	__HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1,duty_fun);

}

