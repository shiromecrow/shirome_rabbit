/*
 * PL_LED.c
 *
 *  Created on: Dec 23, 2023
 *      Author: sf199
 */


/*
 * PL_LED.c
 *
 *  Created on: Jan 11, 2023
 *      Author: sf199
 */


/*
 * PL_LED.c
 *
 *  Created on: 2022/05/13
 *      Author: sf199
 */
// LEDのPeripheral．

#include "PL_LED.h"
#include "gpio.h"
#include "main.h"



void pl_yellow_LED_1(int pin){
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,pin);
}
void pl_yellow_LED_2(int pin){
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,pin);
}
void pl_yellow_LED_3(int pin){
	HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,pin);
}
void pl_yellow_LED_4(int pin){
	HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,pin);
}
void pl_yellow_LED_5(int pin){
	HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,pin);
}
void pl_yellow_LED_6(int pin){
	HAL_GPIO_WritePin(LED6_GPIO_Port,LED6_Pin,pin);
}
void pl_yellow_LED_7(int pin){
	HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,pin);
}
void pl_yellow_LED_8(int pin){
	HAL_GPIO_WritePin(LED8_GPIO_Port,LED8_Pin,pin);
}




void pl_yellow_LED_off(void){
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED6_GPIO_Port,LED6_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED8_GPIO_Port,LED8_Pin,GPIO_PIN_RESET);
}

void pl_yellow_LED_on(void){
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED6_GPIO_Port,LED6_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED8_GPIO_Port,LED8_Pin,GPIO_PIN_SET);
}


void pl_yellow_LED_count(unsigned char yy){

unsigned char yy1,yy2,yy3,yy4,yy5,yy6,yy7,yy8;

yy1 = yy & 1;
yy2 = yy & 2;
yy3 = yy & 4;
yy4 = yy & 8;
yy5 = yy & 16;
yy6 = yy & 32;
yy7 = yy & 64;
yy8 = yy & 128;

if(yy1 >= 1){
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
}else{
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
}
if(yy2 >= 1){
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);
}
else{
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
}
if(yy3 >= 1){
	HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_SET);
}else{
	HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_RESET);
}
if(yy4 >= 1){
	HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_SET);
}else{
	HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_RESET);
}
if(yy5 >= 1){
	HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,GPIO_PIN_SET);
}else{
	HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,GPIO_PIN_RESET);
}
if(yy6 >= 1){
	HAL_GPIO_WritePin(LED6_GPIO_Port,LED6_Pin,GPIO_PIN_SET);
}else{
	HAL_GPIO_WritePin(LED6_GPIO_Port,LED6_Pin,GPIO_PIN_RESET);
}
if(yy7 >= 1){
	HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,GPIO_PIN_SET);
}else{
	HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,GPIO_PIN_RESET);
}
if(yy8 >= 1){
	HAL_GPIO_WritePin(LED8_GPIO_Port,LED8_Pin,GPIO_PIN_SET);
}else{
	HAL_GPIO_WritePin(LED8_GPIO_Port,LED8_Pin,GPIO_PIN_RESET);
}
}


void pl_r_blue_LED(int pin){
	HAL_GPIO_WritePin(BLUE_R_LED_GPIO_Port,BLUE_R_LED_Pin,pin);
}

void pl_l_blue_LED(int pin){
	HAL_GPIO_WritePin(BLUE_L_LED_GPIO_Port,BLUE_L_LED_Pin,pin);
}








