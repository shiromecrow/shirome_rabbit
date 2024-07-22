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

int g_sensor[SENSOR_NUM][20];
int g_sensor_diff[SENSOR_NUM];
int g_sensor_diff_wallcut[SENSOR_NUM];
int g_sensor_diff_wallcut_slant[SENSOR_NUM];
int g_sensor_mean[SENSOR_NUM];

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

//	g_V_battery_mean=0;
//	for(j = 0;j <= 20-1;j++){
//	g_V_battery_mean+=g_V_battery[j];
//	}
//	g_V_battery_mean/=20;



}

/*******************************************************************/
/*	センサーの線形補間用データの測定			(sensor_line)	*/
/*******************************************************************/
/*	センサーの線形補間用データの測定して出力する．						*/
/*******************************************************************/
void sensor_line(void){
	float L_dis[19],R_dis[19];
	int L_g_sensor[19],R_g_sensor[19];
int ee;
	for(ee=0;ee<19;ee++){
	while (g_sensor[2][0] <= SENSOR_FINGER_2) {
					HAL_Delay(1);
				}
	HAL_Delay(100);
	while (g_sensor[2][0] <= SENSOR_FINGER_2) {
						HAL_Delay(1);
					}
	HAL_Delay(500);
	printf("SEN1=%d,SEN2=%d,SEN3=%d,SEN4=%d,SEN5=%d\n", g_sensor[0][0],
											g_sensor[1][0], g_sensor[2][0], g_sensor[3][0], g_sensor[4][0]);
	L_dis[ee]=ee*5+40-85;
	R_dis[18-ee]=130-ee*5-85;
	L_g_sensor[ee]=g_sensor[4][0];
	R_g_sensor[18-ee]=g_sensor[0][0];
	pl_yellow_LED_count(ee);
	pl_play_oneSound(ee);
	}
	while (g_sensor[0][0] <= SENSOR_FINGER_0 || g_sensor[2][0] <= SENSOR_FINGER_2 || g_sensor[4][0] <= SENSOR_FINGER_4) {
							HAL_Delay(1);
						}


	printf("L_SEN,L_dis,R_SEN,R_dis\n");
	for(ee=0;ee<19;ee++){
	printf("%d,%f,%d,%f\n",L_g_sensor[ee],L_dis[ee], R_g_sensor[ee], R_dis[ee]);
	}
}



/*******************************************************************/
/*	センサーの線形補間用データの測定			(sensor_line)	*/
/*******************************************************************/
/*	センサーの線形補間用データの測定して出力する．						*/
/*******************************************************************/
void sensor_line45(void){
	float L_dis[19],R_dis[19];
	int L_g_sensor[19],R_g_sensor[19];
int ee;
	for(ee=0;ee<19;ee++){
	while (g_sensor[2][0] <= SENSOR_FINGER_2) {
					HAL_Delay(1);
				}
	HAL_Delay(100);
	while (g_sensor[2][0] <= SENSOR_FINGER_2) {
						HAL_Delay(1);
					}
	HAL_Delay(500);
	printf("SEN1=%d,SEN2=%d,SEN3=%d,SEN4=%d,SEN5=%d\n", g_sensor[0][0],
											g_sensor[1][0], g_sensor[2][0], g_sensor[3][0], g_sensor[4][0]);
	L_dis[ee]=ee*5+40-85;
	R_dis[18-ee]=130-ee*5-85;
	L_g_sensor[ee]=g_sensor[3][0];
	R_g_sensor[18-ee]=g_sensor[1][0];
	pl_yellow_LED_count(ee);
	pl_play_oneSound(ee);
	}
	while (g_sensor[0][0] <= SENSOR_FINGER_0 || g_sensor[2][0] <= SENSOR_FINGER_2 || g_sensor[4][0] <= SENSOR_FINGER_4) {
							HAL_Delay(1);
						}


	printf("L_SEN,L_dis,R_SEN,R_dis\n");
	for(ee=0;ee<19;ee++){
	printf("%d,%f,%d,%f\n",L_g_sensor[ee],L_dis[ee], R_g_sensor[ee], R_dis[ee]);
	}
}


/*******************************************************************/
/*	センサーの線形補間用データの測定			(sensor_line)	*/
/*******************************************************************/
/*	センサーの線形補間用データの測定して出力する．						*/
/*******************************************************************/
void sensor_line45_slant(void){
	float L_dis[11],R_dis[11];
	int L_g_sensor[11],R_g_sensor[11];
int ee;
	for(ee=0;ee<11;ee++){
	while (g_sensor[2][0] <= SENSOR_FINGER_2) {
					HAL_Delay(1);
				}
	HAL_Delay(100);
	while (g_sensor[2][0] <= SENSOR_FINGER_2) {
						HAL_Delay(1);
					}
	HAL_Delay(500);
	printf("SEN1=%d,SEN2=%d,SEN3=%d,SEN4=%d,SEN5=%d\n", g_sensor[0][0],
											g_sensor[1][0], g_sensor[2][0], g_sensor[3][0], g_sensor[4][0]);
	L_dis[ee]=ee*5-25;
	R_dis[10-ee]=25-ee*5;
	L_g_sensor[ee]=g_sensor[3][0];
	R_g_sensor[10-ee]=g_sensor[1][0];
	pl_yellow_LED_count(ee);
	pl_play_oneSound(ee);
	}
	while (g_sensor[0][0] <= SENSOR_FINGER_0 || g_sensor[2][0] <= SENSOR_FINGER_2 || g_sensor[4][0] <= SENSOR_FINGER_4) {
							HAL_Delay(1);
						}


	printf("L_SEN,L_dis,R_SEN,R_dis\n");
	for(ee=0;ee<11;ee++){
	printf("%d,%f,%d,%f\n",L_g_sensor[ee],L_dis[ee], R_g_sensor[ee], R_dis[ee]);
	}
}
