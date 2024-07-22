/*
 * record.c
 *
 *  Created on: Jan 12, 2023
 *      Author: sf199
 */


/*
 * record.c
 *
 *  Created on: 2019/07/06
 *      Author: sf199
 */

#include "record.h"
#include "PL_timer.h"
#include "PL_encoder.h"
#include "PL_sensor.h"
#include "CL_sensor.h"
#include "CL_EnoderGyro.h"
#include "Control_motor.h"
#include "PID_wall.h"
#include "stdio.h"
#include "define.h"


//#include "motor_control.h"
//#include "PID_wall.h"

float record_value[max_record_num][max_record_time];

char record_mode;

int sample_time;/* サンプリング時間 [ms] */
int sample_count;/* サンプリング観測用のカウント値 */

int record_time;
int record_end_point;
char record_rupe_flag;

//int SEN_record[5][15];
//int SEN_recordD[5][15];

void record_reset(void) {
	record_mode = 0;
	record_time = 0;
	record_rupe_flag = 0;
	sample_time = 1;
	sample_count = 0;
}

void record_data(float *input_record_data, int numlen) {

	if( (sample_count % sample_time) == 0 ){
		for (int record_count = 0; record_count < numlen; record_count++) {
			record_value[record_count][record_time] =
					input_record_data[record_count];
		}
		if (record_rupe_flag == 1) {
			record_end_point = record_time;
		}
		record_time++;
		if (record_time >= max_record_time) {
			record_time = 0;
			record_rupe_flag = 1;
		}

	}
	sample_count++;

}

void record_print(void) {
	int a, time_index;
	if (record_rupe_flag == 0) {
		for (a = 0; a <= record_time - 1; a++) {

			printf("%f", (float)(a*sample_time)*INTERRUPT_TIME);
			for (int record_count = 0; record_count < max_record_num;
					record_count++) {
				printf(",%f", record_value[record_count][a]);
			}
			printf("\n");
		}
	} else {
		for (a = 0; a <= max_record_time - 1; a++) {
			time_index = record_end_point + 1 + a;
			if (time_index >= max_record_time) {
				time_index -= max_record_time;
			}
			printf("%d", a);
			for (int record_count = 0; record_count < max_record_num;
					record_count++) {
				printf(",%f", record_value[record_count][time_index]);
			}
			printf("\n");
		}
	}

}

void interrupt_record(void) {

	float r_data[4];

	if (record_mode == 1) {
			r_data[0] = E_speedR;
			r_data[1] = E_speedL;
			r_data[2] = E_distanceR;
			r_data[3] = E_distanceL;
			record_data(r_data, 4);
		}
	if (record_mode == 2) {
		r_data[0] = turning.velocity;
		r_data[1] = angle_speed;
		r_data[2] = straight.velocity;
		r_data[3] = (fusion_speedR + fusion_speedL) / 2;
				record_data(r_data, 4);
		}
	if (record_mode == 3) {
			r_data[0] = straight.velocity;
			r_data[1] = straight.displacement;
			r_data[2] = (fusion_speedL + fusion_speedR) / 2;
			r_data[3] = (fusion_distanceL + fusion_distanceR) / 2;
			record_data(r_data, 4);
		}
	if (record_mode == 4) { //距離の比較
			r_data[0] = straight.displacement;
			r_data[1] = (E_distanceR + E_distanceL) / 2;
			r_data[2] = gf_distance;
			r_data[3] = (fusion_distanceR + fusion_distanceL) / 2;
			record_data(r_data, 4);
		}
	if (record_mode == 5) { //距離の比較
			r_data[0] = straight.velocity;
			r_data[1] = kalman_distance;
			r_data[2] = kalman_distance2;
			r_data[3] = kalman_speed;
			record_data(r_data, 4);
		}
	if (record_mode == 6) { //距離の比較
				r_data[0] = straight.velocity;
				r_data[1] = E_lpf_speedL;
				r_data[2] = E_lpf_speedR;
				r_data[3] = gf_speed;
				record_data(r_data, 4);
			}
	if (record_mode == 7) {
			r_data[0] = (float) g_sensor[SENSOR_LEFT][0];
			r_data[1] = (float) g_sensor_diff[SENSOR_LEFT];
			r_data[2] = (float) g_sensor[SENSOR_RIGHT][0];
			r_data[3] = (float) g_sensor_diff[SENSOR_RIGHT];
			record_data(r_data, 4);
		}
	if (record_mode == 8) {
			r_data[0] = (float) g_sensor[SENSOR_FRONT_LEFT][0];
			r_data[1] = (float) g_sensor_diff_wallcut[SENSOR_FRONT_LEFT];
			r_data[2] = (float) g_sensor[SENSOR_FRONT_RIGHT][0];
			r_data[3] = (float) g_sensor_diff_wallcut[SENSOR_FRONT_RIGHT];
			record_data(r_data, 4);
		}
	if (record_mode == 9) { //90
			r_data[0] = (float) g_sensor[SENSOR_LEFT][0];
			r_data[1] = (float) g_sensor[SENSOR_RIGHT][0];
			r_data[2] = NoWallDisplacementL45slant;
			r_data[3] = NoWallDisplacementR45slant;
			record_data(r_data, 4);
		}
	if (record_mode == 10) { //90
			r_data[0] = (float) g_sensor[SENSOR_FRONT_LEFT][0];
			r_data[1] = (float) g_sensor[SENSOR_FRONT_RIGHT][0];
			r_data[2] = NoWallDisplacementL45slant;
			r_data[3] = NoWallDisplacementR45slant;
			record_data(r_data, 4);
		}
	if (record_mode == 11) { //距離の比較
				r_data[0] = E_speedL;
				r_data[1] = encoder_L;
				r_data[2] = E_speedR;
				r_data[3] = encoder_R;
				record_data(r_data, 4);
			}
	if (record_mode == 12) {
		r_data[0] = turning.velocity;
		r_data[1] = angle_speed;
		r_data[2] = g_V_L;
		r_data[3] = g_V_R;
				record_data(r_data, 4);
		}
	if (record_mode == 13) {
			r_data[0] = straight.velocity;
			r_data[1] = E_speedR;
			r_data[2] = E_speedL;
			r_data[3] = gf_speed;
					record_data(r_data, 4);
			}
	if (record_mode == 14) {
		r_data[0] = turning.velocity;
		r_data[1] = g_V_batt;
		r_data[2] = g_V_L;
		r_data[3] = g_V_R;
					record_data(r_data, 4);
			}

	if (record_mode == 15) {
			r_data[0] = (float) g_sensor[SENSOR_FRONT_LEFT][0];
			r_data[1] = (float) g_sensor_diff_wallcut[SENSOR_FRONT_LEFT];
			r_data[2] = (float) NoWallDisplacementL45;
			r_data[3] = (float) NoWallCountL45;
			record_data(r_data, 4);
		}

	if (record_mode == 16) {
			r_data[0] = (float) g_sensor[SENSOR_FRONT_RIGHT][0];
			r_data[1] = (float) g_sensor_diff_wallcut_slant[SENSOR_FRONT_RIGHT];
			r_data[2] = (float) NoWallDisplacementR45slant;
			r_data[3] = (float) NoWallDisplacementR45slant2;
			record_data(r_data, 4);
		}
	if (record_mode == 17) {
			r_data[0] = (float) g_sensor[SENSOR_FRONT_LEFT][0];
			r_data[1] = (float) g_sensor_diff_wallcut_slant[SENSOR_FRONT_LEFT];
			r_data[2] = (float) NoWallDisplacementL45slant;
			r_data[3] = (float) NoWallDisplacementL45slant2;
			record_data(r_data, 4);
		}
/*	if (record_mode == 1) {
		r_data[0] = straight.velocity;
		r_data[1] = straight.displacement;
		r_data[2] = (E_speedR + E_speedL) / 2;
		r_data[3] = (E_distanceR + E_distanceL) / 2;
		record_data(r_data, 4);
	} else if (record_mode == 2) {
		r_data[0] = turning.velocity;
		r_data[1] = turning.acceleration;
		r_data[2] = angle_speed;
		r_data[3] = angle; //g_V_R;
		record_data(r_data, 4);
	} else if (record_mode == 3) {
		r_data[0] = (float) g_sensor[0][0];
		r_data[1] = (float) g_sensor[1][0];
		r_data[2] = (float) g_sensor[3][0];
		r_data[3] = (float) g_sensor[4][0];
		record_data(r_data, 4);
	} else if (record_mode == 4) {
		r_data[0] = (float) g_sensor[0][0];
		r_data[1] = (float) g_sensor_diff[0];
		r_data[2] = (float) g_sensor[4][0];
		r_data[3] = (float) g_sensor_diff[4];
		record_data(r_data, 4);
	} else if (record_mode == 5) {
		r_data[0] = (E_speedR + E_speedL) / 2;
		r_data[1] = enc.error;
		r_data[2] = enc.sigma_error;
		r_data[3] = enc.delta_error;
		record_data(r_data, 4);
	} else if (record_mode == 6) {
		r_data[0] = angle_speed;
		r_data[1] = Gyro.error;
		r_data[2] = Gyro.sigma_error;
		r_data[3] = Gyro.delta_error;
		record_data(r_data, 4);
	} else if (record_mode == 7) { //90
		r_data[0] = (float) g_sensor[0][0];
		r_data[1] = (float) g_sensor[4][0];
		r_data[2] = NoWallDisplacementR45slant;
		r_data[3] = NoWallDisplacementL45slant;
		record_data(r_data, 4);
	} else if (record_mode == 8) {
		r_data[0] = (float) g_sensor_diff[1];
		r_data[1] = (float) g_sensor_diff[3];
		r_data[2] = NoWallDisplacementR45slant2;
		r_data[3] = NoWallDisplacementL45slant2;
		record_data(r_data, 4);
	} else if (record_mode == 9) { //45
		r_data[0] = (float) g_sensor[1][0];
		r_data[1] = (float) g_sensor[3][0];
		r_data[2] = NoWallDisplacementR45slant;
		r_data[3] = NoWallDisplacementL45slant;
		record_data(r_data, 4);
	} else if (record_mode == 10) { //45
		r_data[0] = NoWallDisplacementR45slant;
		r_data[1] = NoWallDisplacementL45slant;
		r_data[2] = wall_slant90.error;
		r_data[3] = wall_slant45.error;
		record_data(r_data, 4);
	} else if (record_mode == 11) { //90
		r_data[0] = g_log_CenterSlantR90;
		r_data[1] = g_sensor[SENSOR_RIGHT][0];
		r_data[2] = g_log_CenterSlantL90;
		r_data[3] = g_sensor[SENSOR_LEFT][0];
		record_data(r_data, 4);
	} else if (record_mode == 12) { //45
		r_data[0] = g_log_CenterSlantR45;
		r_data[1] = g_sensor[SENSOR_FRONT_RIGHT][0];
		r_data[2] = g_log_CenterSlantL45;
		r_data[3] = g_sensor[SENSOR_FRONT_LEFT][0];
		record_data(r_data, 4);
	} else if (record_mode == 13) {
		r_data[0] = (float) g_sensor[1][0];
		r_data[1] = (float) g_sensor_diff_wallcut[1];
		r_data[2] = (float) g_sensor[3][0];
		r_data[3] = (float) g_sensor_diff_wallcut[3];
		record_data(r_data, 4);
	} else if (record_mode == 14) {
		r_data[0] = straight.velocity;
		r_data[1] = (g_V_R + g_V_L) / 2;
		r_data[2] = (E_speedR + E_speedL) / 2;
		r_data[3] = (E_distanceR + E_distanceL) / 2;
		record_data(r_data, 4);
	} else if (record_mode == 15) { //速度の比較
		r_data[0] = straight.velocity;
		r_data[1] = (E_speedR + E_speedL) / 2;
		r_data[2] = gf_speed;
		r_data[3] = (fusion_speedR + fusion_speedL) / 2;
		record_data(r_data, 4);
	} else if (record_mode == 16) { //速度の比較
		r_data[0] = straight.displacement;
		r_data[1] = (E_distanceR + E_distanceL) / 2;
		r_data[2] = gf_distance;
		r_data[3] = (fusion_distanceR + fusion_distanceL) / 2;
		record_data(r_data, 4);
	} else if (record_mode == 17) { //速度の比較
		r_data[0] = straight.velocity;
		r_data[1] = (E_speedR + E_speedL) / 2;
		r_data[2] = gf_accel;
		r_data[3] = angle_speed;
		record_data(r_data, 4);
	} else if (record_mode == 18) { //速度の比較
		r_data[0] = straight.velocity;
		r_data[1] = (fusion_speedR + fusion_speedL) / 2;
		r_data[2] = g_V_R;
		r_data[3] = g_V_L;
		record_data(r_data, 4);
	}
*/

}

