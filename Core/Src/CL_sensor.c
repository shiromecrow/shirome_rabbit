/*
 * CL_sensor.c
 *
 *  Created on: Jan 11, 2023
 *      Author: sf199
 */


#include "CL_sensor.h"
#include "PL_sensor.h"
#include "PL_timer.h"
#include "PL_LED.h"
//#include "record.h"
#include "define.h"
#include "stdio.h"
#include "math.h"

int g_sensor[SENSOR_NUM][20];
int g_sensor_diff[SENSOR_NUM];
int g_sensor_diff_wallcut[SENSOR_NUM];
int g_sensor_diff_wallcut_slant[SENSOR_NUM];
int g_sensor_mean[SENSOR_NUM];
float g_sensor_distance[SENSOR_NUM];
float g_sensor_distance_slant[SENSOR_NUM];

float g_V_battery[20];
float g_V_battery_mean;

/*******************************************************************/
/*	バッテリーの確認				(battcheak)	*/
/*******************************************************************/
/*	バッテリーの残電圧のチェックし、小さい場合は機能の停止を行う．	．						*/
/*******************************************************************/
void battcheak(void){
int i;
	for(i = 0;i <= 20-1;i++){
		g_V_battery[i]=pl_getbatt();
		wait_ms(10);
	}

	g_V_battery_mean=0;
	for(i = 0;i <= 20-1;i++){
	g_V_battery_mean+=g_V_battery[i];
	}
	g_V_battery_mean/=20;
	//printf("BATT=%f\n",g_V_battery_mean);
	//printf("%f",g_V_battery_mean);
	if(g_V_battery_mean <= BATTLIMIT){
		//pl_stop_Sound();
		while(1){
		pl_r_blue_LED(ON);
		pl_l_blue_LED(ON);
		wait_ms(500);
		pl_r_blue_LED(OFF);
		pl_l_blue_LED(OFF);
		wait_ms(500);
		}
	}
}

/*******************************************************************/
/*	割り込み用動作関数(センサー処理)			(interupt_calSensor)	*/
/*******************************************************************/
/*	センサーの情報を処理する割り込み関数．						*/
/*******************************************************************/
void interupt_calSensor(void){
	int j;

	pl_interupt_getSensor();

	for (j = 19; j >= 1; j--) {
		//g_V_battery[j] = g_V_battery[j - 1];
		for(int k=0;k < SENSOR_NUM; k++){
			g_sensor[k][j] = g_sensor[k][j - 1];
		}
	}
	//g_V_battery[0] = g_V_batt;
	for(int k=0;k < SENSOR_NUM; k++){
		g_sensor[k][0] = g_sensor_on[k] - g_sensor_off[k];
	}

	for (j = 0; j < SENSOR_NUM; j++) {
		g_sensor_diff[j]=g_sensor[j][0]-g_sensor[j][11];
		//g_sensor_diff_wallcut[j]=g_sensor[j][0]-g_sensor[j][6];
		g_sensor_mean[j] = (g_sensor[j][0] + g_sensor[j][1] + g_sensor[j][2]) / 3;
	}

// 多項式近似
	g_sensor_distance[SENSOR_LEFT] = LIN_COEFFICIENT_L3*(double)(g_sensor[SENSOR_LEFT][0]*g_sensor[SENSOR_LEFT][0]*g_sensor[SENSOR_LEFT][0])
									  + LIN_COEFFICIENT_L2*(double)(g_sensor[SENSOR_LEFT][0]*g_sensor[SENSOR_LEFT][0])
									  + LIN_COEFFICIENT_L1*(double)(g_sensor[SENSOR_LEFT][0]) + LIN_COEFFICIENT_L0;
	g_sensor_distance[SENSOR_RIGHT] = LIN_COEFFICIENT_R3*(double)(g_sensor[SENSOR_RIGHT][0]*g_sensor[SENSOR_RIGHT][0]*g_sensor[SENSOR_RIGHT][0])
									  + LIN_COEFFICIENT_R2*(double)(g_sensor[SENSOR_RIGHT][0]*g_sensor[SENSOR_RIGHT][0])
									  + LIN_COEFFICIENT_R1*(double)(g_sensor[SENSOR_RIGHT][0]) + LIN_COEFFICIENT_R0;

	g_sensor_distance_slant[SENSOR_LEFT] = LIN_SLANT_COEFFICIENT90_L_n3 / ((double)(g_sensor[SENSOR_LEFT][0]*g_sensor[SENSOR_LEFT][0]*g_sensor[SENSOR_LEFT][0])+0.1)
											+ LIN_SLANT_COEFFICIENT90_L_n2 / ((double)(g_sensor[SENSOR_LEFT][0]*g_sensor[SENSOR_LEFT][0])+0.1)
											+ LIN_SLANT_COEFFICIENT90_L_n1 / ((double)(g_sensor[SENSOR_LEFT][0])+0.1)
											+ LIN_SLANT_COEFFICIENT90_L_00
											+ LIN_SLANT_COEFFICIENT90_L_p1 * (double)(g_sensor[SENSOR_LEFT][0]);
	g_sensor_distance_slant[SENSOR_RIGHT] = LIN_SLANT_COEFFICIENT90_R_n3 / ((double)(g_sensor[SENSOR_RIGHT][0]*g_sensor[SENSOR_RIGHT][0]*g_sensor[SENSOR_RIGHT][0])+0.1)
											+ LIN_SLANT_COEFFICIENT90_R_n2 / ((double)(g_sensor[SENSOR_RIGHT][0]*g_sensor[SENSOR_RIGHT][0])+0.1)
											+ LIN_SLANT_COEFFICIENT90_R_n1 / ((double)(g_sensor[SENSOR_RIGHT][0])+0.1)
											+ LIN_SLANT_COEFFICIENT90_R_00
											+ LIN_SLANT_COEFFICIENT90_R_p1 * (double)(g_sensor[SENSOR_RIGHT][0]);

	g_sensor_distance_slant[SENSOR_FRONT_LEFT] = LIN_SLANT_COEFFICIENT45_L_n3 / ((double)(g_sensor[SENSOR_FRONT_LEFT][0]*g_sensor[SENSOR_FRONT_LEFT][0]*g_sensor[SENSOR_FRONT_LEFT][0])+0.1)
											+ LIN_SLANT_COEFFICIENT45_L_n2 / ((double)(g_sensor[SENSOR_FRONT_LEFT][0]*g_sensor[SENSOR_FRONT_LEFT][0])+0.1)
											+ LIN_SLANT_COEFFICIENT45_L_n1 / ((double)(g_sensor[SENSOR_FRONT_LEFT][0])+0.1)
											+ LIN_SLANT_COEFFICIENT45_L_00
											+ LIN_SLANT_COEFFICIENT45_L_p1 * (double)(g_sensor[SENSOR_FRONT_LEFT][0]);
	g_sensor_distance_slant[SENSOR_FRONT_RIGHT] = LIN_SLANT_COEFFICIENT45_R_n3 / ((double)(g_sensor[SENSOR_FRONT_RIGHT][0]*g_sensor[SENSOR_FRONT_RIGHT][0]*g_sensor[SENSOR_FRONT_RIGHT][0])+0.1)
											+ LIN_SLANT_COEFFICIENT45_R_n2 / ((double)(g_sensor[SENSOR_FRONT_RIGHT][0]*g_sensor[SENSOR_FRONT_RIGHT][0])+0.1)
											+ LIN_SLANT_COEFFICIENT45_R_n1 / ((double)(g_sensor[SENSOR_FRONT_RIGHT][0])+0.1)
											+ LIN_SLANT_COEFFICIENT45_R_00
											+ LIN_SLANT_COEFFICIENT45_R_p1 * (double)(g_sensor[SENSOR_FRONT_RIGHT][0]);


/*
	g_sensor_distance[SENSOR_LEFT] = LIN_COEFFICIENT90_L_a * log(LIN_COEFFICIENT90_L_b / (fabs((double)g_sensor[SENSOR_LEFT][0])+0.1)) + LIN_COEFFICIENT90_L_c;
	g_sensor_distance[SENSOR_RIGHT] = LIN_COEFFICIENT90_R_a * log(LIN_COEFFICIENT90_R_b / (fabs((double)g_sensor[SENSOR_RIGHT][0])+0.1)) + LIN_COEFFICIENT90_R_c;


	g_sensor_distance_slant[SENSOR_LEFT] = LIN_SLANT_COEFFICIENT90_L_a * log(LIN_SLANT_COEFFICIENT90_L_b / (fabs((double)g_sensor[SENSOR_LEFT][0])+0.1)) + LIN_SLANT_COEFFICIENT90_L_c;
	g_sensor_distance_slant[SENSOR_RIGHT] = LIN_SLANT_COEFFICIENT90_R_a * log(LIN_SLANT_COEFFICIENT90_R_b / (fabs((double)g_sensor[SENSOR_RIGHT][0])+0.1)) + LIN_SLANT_COEFFICIENT90_R_c;
	g_sensor_distance_slant[SENSOR_FRONT_LEFT] = LIN_SLANT_COEFFICIENT45_L_a * log(LIN_SLANT_COEFFICIENT45_L_b / (fabs((double)g_sensor[SENSOR_FRONT_LEFT][0])+0.1)) + LIN_SLANT_COEFFICIENT45_L_c;
	g_sensor_distance_slant[SENSOR_FRONT_RIGHT] = LIN_SLANT_COEFFICIENT45_R_a * log(LIN_SLANT_COEFFICIENT45_R_b / (fabs((double)g_sensor[SENSOR_FRONT_RIGHT][0])+0.1)) + LIN_SLANT_COEFFICIENT45_R_c;
*/
	//	g_V_battery_mean=0;
//	for(j = 0;j <= 20-1;j++){
//	g_V_battery_mean+=g_V_battery[j];
//	}
//	g_V_battery_mean/=20;



}


/*******************************************************************/
/*	センサーの線形補間用データの測定して出力する．						*/
/*******************************************************************/
void sensor_line(void){
	int len=9;
	float disL[len],disR[len];
	int senL90[len],senR90[len];
	int senL45[len],senR45[len];

	for(int d=0;d<len;d++){
	while (g_sensor[SENSOR_FRONT_L][0] <= SENSOR_FINGER_0 || g_sensor[SENSOR_FRONT_R][0] <= SENSOR_FINGER_5) {
					HAL_Delay(1);
				}
	pl_yellow_LED_count(d+1);
	printf("SEN1=%d,SEN2=%d,SEN3=%d,SEN4=%d,SEN5=%d,SEN6=%d\n", g_sensor[0][0],
											g_sensor[1][0], g_sensor[2][0], g_sensor[3][0], g_sensor[4][0], g_sensor[5][0]);
	HAL_Delay(500);
	while (g_sensor[SENSOR_FRONT_L][0] > SENSOR_FINGER_0 && g_sensor[SENSOR_FRONT_R][0] > SENSOR_FINGER_5) {
					HAL_Delay(1);
	}
	HAL_Delay(2000);
	disL[d]= -20 + 5*d;
	disR[d]= 20 - 5*d;
	senL90[d] = g_sensor_mean[SENSOR_LEFT];
	senR90[d] = g_sensor_mean[SENSOR_RIGHT];
	senL45[d] = g_sensor_mean[SENSOR_FRONT_LEFT];
	senR45[d] = g_sensor_mean[SENSOR_FRONT_RIGHT];
	pl_yellow_LED_count(0);

	}

	while(1){
	while (g_sensor[SENSOR_FRONT_L][0] <= SENSOR_FINGER_0 || g_sensor[SENSOR_FRONT_R][0] <= SENSOR_FINGER_5) {
							HAL_Delay(1);
						}


	printf("senL90,disL,senR90,disR,senL45,disL,senR45,disR\n");
	for(int d=0;d<len;d++){
	printf("%d,%f,%d,%f,%d,%f,%d,%f\n",senL90[d],disL[d], senR90[d], disR[d], senL45[d],disL[d], senR45[d], disR[d]);
	while (g_sensor[SENSOR_FRONT_L][0] > SENSOR_FINGER_0 && g_sensor[SENSOR_FRONT_R][0] > SENSOR_FINGER_5) {
					HAL_Delay(1);
				}
	}
	}
}

/*******************************************************************/
/*	センサーの線形補間用データの測定して出力する．						*/
/*******************************************************************/
void sensor_line_slant(void){
	int len=23;
	float disL[len],disR[len];
	int senL90[len],senR90[len];
	int senL45[len],senR45[len];

	for(int d=0;d<len;d++){
	while (g_sensor[SENSOR_FRONT_L][0] <= SENSOR_FINGER_0 || g_sensor[SENSOR_FRONT_R][0] <= SENSOR_FINGER_5) {
					HAL_Delay(1);
				}
	pl_yellow_LED_count(d+1);
	printf("SEN1=%d,SEN2=%d,SEN3=%d,SEN4=%d,SEN5=%d,SEN6=%d\n", g_sensor[0][0],
											g_sensor[1][0], g_sensor[2][0], g_sensor[3][0], g_sensor[4][0], g_sensor[5][0]);
	HAL_Delay(500);
	while (g_sensor[SENSOR_FRONT_L][0] > SENSOR_FINGER_0 && g_sensor[SENSOR_FRONT_R][0] > SENSOR_FINGER_5) {
					HAL_Delay(1);
	}
	HAL_Delay(2000);
	disL[d]= 5*d;
	senL90[d] = g_sensor_mean[SENSOR_LEFT];
	senL45[d] = g_sensor_mean[SENSOR_FRONT_LEFT];
	pl_yellow_LED_count(0);

	}


	for(int d=0;d<len;d++){
	while (g_sensor[SENSOR_FRONT_L][0] <= SENSOR_FINGER_0 || g_sensor[SENSOR_FRONT_R][0] <= SENSOR_FINGER_5) {
					HAL_Delay(1);
				}
	pl_yellow_LED_count(d+1);
	printf("SEN1=%d,SEN2=%d,SEN3=%d,SEN4=%d,SEN5=%d,SEN6=%d\n", g_sensor[0][0],
											g_sensor[1][0], g_sensor[2][0], g_sensor[3][0], g_sensor[4][0], g_sensor[5][0]);
	HAL_Delay(500);
	while (g_sensor[SENSOR_FRONT_L][0] > SENSOR_FINGER_0 && g_sensor[SENSOR_FRONT_R][0] > SENSOR_FINGER_5) {
					HAL_Delay(1);
	}
	HAL_Delay(2000);
	disR[d]= 5*d;
	senR90[d] = g_sensor_mean[SENSOR_RIGHT];
	senR45[d] = g_sensor_mean[SENSOR_FRONT_RIGHT];
	pl_yellow_LED_count(0);

	}


	while(1){
	while (g_sensor[SENSOR_FRONT_L][0] <= SENSOR_FINGER_0 || g_sensor[SENSOR_FRONT_R][0] <= SENSOR_FINGER_5) {
							HAL_Delay(1);
						}


	printf("senL90,disL,senR90,disR,senL45,disL,senR45,disR\n");
	for(int d=0;d<len;d++){
	printf("%d,%f,%d,%f,%d,%f,%d,%f\n",senL90[d],disL[d], senR90[d], disR[d], senL45[d],disL[d], senR45[d], disR[d]);
	while (g_sensor[SENSOR_FRONT_L][0] > SENSOR_FINGER_0 && g_sensor[SENSOR_FRONT_R][0] > SENSOR_FINGER_5) {
					HAL_Delay(1);
				}
	}
	}
}
