/*
 * PID_wall.c
 *
 *  Created on: 2023/01/17
 *      Author: sf199
 */

#include "PID_wall.h"
#include "PID_EncoderGyro.h"
#include "CL_sensor.h"
#include "CL_EnoderGyro.h"
#include "PL_motor.h"
#include "PL_timer.h"
#include "PL_LED.h"
#include "tim.h"
#include "define.h"
#include "Control_motor.h"//やりたくなかったけどしょうがない
#include "fail_safe.h"

#include "stdio.h"
#include "math.h"


PIDW wall_normal;
PIDW wall_slant90;
PIDW wall_slant45;
PIDW wall_front_l;
PIDW wall_front_r;

uint8_t g_WallControl_mode; //0で壁制御なし、1で通常の壁制御、2で斜めの制御
uint8_t g_FrontWallControl_mode;
uint8_t g_wallCut_mode;
uint8_t g_WallControlStatus;
uint8_t g_WallControlStatus45;
uint8_t StabilityCount_reset;
uint8_t StabilityCount_L, StabilityCount_R;
float Stabilitydisplacement_L, Stabilitydisplacement_R;

float g_sensor_max_l;
float g_sensor_max_r;

float g_skewer_displacement;

float g_log_CenterSlantR45, g_log_CenterSlantL45; //log用
float g_log_CenterSlantR90, g_log_CenterSlantL90; //log用

volatile uint32_t NoWallCountR90, NoWallCountL90, NoWallCountR45,
		NoWallCountL45;
volatile uint32_t NoWallCountR45slant, NoWallCountL45slant;

volatile float NoWallDisplacementR90, NoWallDisplacementL90,
		NoWallDisplacementR45, NoWallDisplacementL45;
volatile float NoWallDisplacementR45slant, NoWallDisplacementL45slant;
volatile float NoWallDisplacementR45slant2, NoWallDisplacementL45slant2;

uint8_t Nowall_safe_flg =0;
float NoWallDisplacementR_safe, NoWallDisplacementL_safe;//fail_safe用、壁切れ処理が異様に長い場合に静止

uint8_t NoWallCountL45_flag, NoWallCountR45_flag, NoWallCountL45slant_flag,
		NoWallCountR45slant_flag, slantWallControlL_flag,
		slantWallControlR_flag;

volatile uint8_t front_wall_break_90, front_wall_break_45,
		front_wall_break_45slant; //前壁が閾値以上でだ出

float g_sensor_max_fl, g_sensor_max_fr, g_sensor_max_fl_slant,
		g_sensor_max_fr_slant;

float g_CenterSlantR90[12], g_CenterSlantL90[12], g_CenterSlantR45[12],
		g_CenterSlantL45[12];
float g_CenterSlantR90_diff, g_CenterSlantL90_diff, g_CenterSlantR45_diff,
		g_CenterSlantL45_diff;



void init_WallControl(void) {
	g_WallControl_mode = 0;
	g_FrontWallControl_mode = 0;
	g_WallControlStatus = 0;
	g_wallCut_mode = 0;
	StabilityCount_reset = 0;
	StabilityCount_L = 0;
	StabilityCount_R = 0;
	Stabilitydisplacement_L = 0;
	Stabilitydisplacement_R = 0;
	g_sensor_max_l = CENTER_L_PILLAR;
	g_sensor_max_r = CENTER_R_PILLAR;
	g_skewer_displacement = SKEWER_LIMIT;

	NoWallCountL90 = 0;
	NoWallCountR90 = 0;
	NoWallCountL45 = 0;
	NoWallCountR45 = 0;
	NoWallCountL45slant = 0;
	NoWallCountR45slant = 0;

	NoWallDisplacementL90 = 0;
	NoWallDisplacementR90 = 0;
	NoWallDisplacementL45 = CUTPLACE_THRESHOLD_END_L45;
	NoWallDisplacementR45 = CUTPLACE_THRESHOLD_END_R45;
	NoWallDisplacementL45slant = 0;
	NoWallDisplacementR45slant = 0;
	NoWallDisplacementL45slant2 = 0;
	NoWallDisplacementR45slant2 = 0;

	NoWallCountL45_flag = 0;
	NoWallCountR45_flag = 0;
	NoWallCountL45slant_flag = 0;
	NoWallCountR45slant_flag = 0;
	slantWallControlL_flag = 0;
	slantWallControlR_flag = 0;

	front_wall_break_90 = 0;
	front_wall_break_45 = 0;
	front_wall_break_45slant = 0;

	g_sensor_max_fl = 0;
	g_sensor_max_fr = 0;
	g_sensor_max_fl_slant = 0;
	g_sensor_max_fr_slant = 0;

	Nowall_safe_flg = 0;
	NoWallDisplacementR_safe = 0;//壁切れ安全機能
	NoWallDisplacementL_safe = 0;//壁切れ安全機能

}

void init2_WallControl(void){
	/* 串制御対策で作成 */
		g_WallControlStatus = 0;
		StabilityCount_reset = 0;
		wall_normal.old_error = 0;
		StabilityCount_L = 0;
		StabilityCount_R = 0;
		Stabilitydisplacement_L = 0;
		Stabilitydisplacement_R = 0;
		g_sensor_max_l = CENTER_L_PILLAR;
		g_sensor_max_r = CENTER_R_PILLAR;
		g_skewer_displacement = SKEWER_LIMIT;
}




float calWallConrol(void) {
	float PID_wall,PID_w;
	float sensor_gain_p, sensor_gain_d;
	float sensorWall_L, sensorWall_R;
	float wallcut_threshold_L, wallcut_threshold_R;
	float skewer_gain;
	float Skewer_limit;
	float skewer_lpf;
	float Stability_limit;

	if (highspeed_mode == 1) {
		wallcut_threshold_L = CONTROLWALLCUT_THRESHOLD_SHORT_L;
		wallcut_threshold_R = CONTROLWALLCUT_THRESHOLD_SHORT_R;

		if (straight.velocity > 2000) {
			sensor_gain_p = SENSOR_GAIN_SHORT_P * 2000;
			sensor_gain_d = SENSOR_GAIN_SHORT_D * 2000;
		} else {
			sensor_gain_p = SENSOR_GAIN_SHORT_P * straight.velocity;
			sensor_gain_d = SENSOR_GAIN_SHORT_D * straight.velocity;
		}
		Stability_limit = STABILITY_LIMIT_SHORT;
	} else {
		wallcut_threshold_L = CONTROLWALLCUT_THRESHOLD_L;
		wallcut_threshold_R = CONTROLWALLCUT_THRESHOLD_R;
		sensor_gain_p = SENSOR_GAIN_P * straight.velocity;
		sensor_gain_d = SENSOR_GAIN_D * straight.velocity;
		Stability_limit = STABILITY_LIMIT;
	}

	if (g_sensor[SENSOR_LEFT][0] < SENSOR_L_MIN) {
		sensorWall_L = SENSOR_L_MIN;
	} else if (g_sensor[SENSOR_LEFT][0] > SENSOR_L_MAX) {
		sensorWall_L = SENSOR_L_MAX;
	} else {
		sensorWall_L = (float) (g_sensor[SENSOR_LEFT][0]);
	}
	if (g_sensor[SENSOR_RIGHT][0] < SENSOR_R_MIN) {
		sensorWall_R = SENSOR_R_MIN;
	} else if (g_sensor[SENSOR_RIGHT][0] > SENSOR_R_MAX) {
		sensorWall_R = SENSOR_R_MAX;
	} else {
		sensorWall_R = (float) (g_sensor[SENSOR_RIGHT][0]);
	}


	PID_wall = 0;
	PID_w = 0;
	if (g_WallControl_mode == 0) {
		g_WallControlStatus = 0;
		StabilityCount_reset = 0;
		StabilityCount_L = 0;
		StabilityCount_R = 0;
		Stabilitydisplacement_L = 0;
		Stabilitydisplacement_R = 0;
		g_sensor_max_l = CENTER_L_PILLAR;
		g_sensor_max_r = CENTER_R_PILLAR;
		g_skewer_displacement = SKEWER_LIMIT;
		PID_wall = 0;
		pl_yellow_LED_off();

	} else if (g_WallControl_mode == 1) {

		// 左壁の有無の判定
		if (((g_WallControlStatus >> 0) & 1) == 1) {
			//前回左壁あり
			//閾値を下回る　or 変化量の急増
			if (g_sensor[SENSOR_LEFT][0] < CONTROLWALL_THRESHOLD_L
					|| fabs(g_sensor_diff[SENSOR_LEFT]) > wallcut_threshold_L) {
				g_WallControlStatus = g_WallControlStatus - 1;
				// g_sensor_max_l = 0;
				// for (int i = 0; i <= 19; i++) {
				// 	if (g_sensor_max_l < (float) (g_sensor[SENSOR_LEFT][i])) {
				// 		g_sensor_max_l = (float) (g_sensor[SENSOR_LEFT][i]);
				// 	}
				// }
				//g_skewer_displacement = 0;
			}
			StabilityCount_L = 0;
			Stabilitydisplacement_L = 0;
		} else {
			//前回左壁なし
			//閾値を上回る　and 変化量が落ち着く　+ その安定な状態が数回続く
			if (g_sensor[SENSOR_LEFT][0] > CONTROLWALL_THRESHOLD_L
					&& fabs(g_sensor_diff[SENSOR_LEFT]) < wallcut_threshold_L) {
				// g_WallControlStatus = g_WallControlStatus + 1;
				StabilityCount_L++;
				Stabilitydisplacement_L += straight.velocity * INTERRUPT_TIME;
			} else {
				StabilityCount_L = 0;
				Stabilitydisplacement_L = 0;
			}
			if ( Stabilitydisplacement_L >= Stability_limit ) {
				g_WallControlStatus = g_WallControlStatus + 1;
			}
		}
		// 右壁の有無の判定
		if (((g_WallControlStatus >> 1) & 1) == 1) {
			//前回右壁あり
			//閾値を下回る　or 変化量の急増
			if (g_sensor[SENSOR_RIGHT][0] < CONTROLWALL_THRESHOLD_R
					|| fabs(g_sensor_diff[SENSOR_RIGHT])
							> wallcut_threshold_R) {
				g_WallControlStatus = g_WallControlStatus - 2;
				// g_sensor_max_r = 0;
				// for (int i = 0; i <= 19; i++) {
				// 	if (g_sensor_max_r < (float) (g_sensor[SENSOR_RIGHT][i])) {
				// 		g_sensor_max_r = (float) (g_sensor[SENSOR_RIGHT][i]);
				// 	}
				// }
				//g_skewer_displacement = 0;
			}
			StabilityCount_R = 0;
			Stabilitydisplacement_R = 0;
		} else {
			//前回右壁なし
			//閾値を上回る　and 変化量が落ち着く　+ その安定な状態が数回続く
			if (g_sensor[SENSOR_RIGHT][0] > CONTROLWALL_THRESHOLD_R
					&& fabs(g_sensor_diff[SENSOR_RIGHT])
							< wallcut_threshold_R) {
				StabilityCount_R++;
				Stabilitydisplacement_R += straight.velocity * INTERRUPT_TIME;
				// g_WallControlStatus = g_WallControlStatus + 2;			//安定消す
			} else {
				StabilityCount_R = 0;
				Stabilitydisplacement_R = 0;
			}
			if ( Stabilitydisplacement_R >= Stability_limit ) {
				g_WallControlStatus = g_WallControlStatus + 2;
			}
		}
/* 制御量計算 */
		switch (g_WallControlStatus) {
		case 0:			//両壁なし
			if(highspeed_mode==1){
				skewer_gain=SKEWER_GAIN_SHORT;
				Skewer_limit = SKEWER_LIMIT_SHORT;
				skewer_lpf = 0.1;
			}else{
				skewer_gain=SKEWER_GAIN;
				Skewer_limit = SKEWER_LIMIT*straight.velocity/300;
				skewer_lpf = 0.1;
			}

			g_skewer_displacement += straight.velocity * INTERRUPT_TIME;
			if(g_sensor[SENSOR_LEFT][0] > PILLAR_THRESHOLD_L && g_sensor[SENSOR_RIGHT][0] > PILLAR_THRESHOLD_R){
				g_sensor_max_l = 0;
				for (int i = 0; i <= 19; i++) {
					if (g_sensor_max_l < (float) (g_sensor[SENSOR_LEFT][i])) {
						g_sensor_max_l = (float) (g_sensor[SENSOR_LEFT][i]);
					}
				}
				g_sensor_max_r = 0;
				for (int i = 0; i <= 19; i++) {
					if (g_sensor_max_r < (float) (g_sensor[SENSOR_RIGHT][i])) {
						g_sensor_max_r = (float) (g_sensor[SENSOR_RIGHT][i]);
					}
				}
				g_skewer_displacement=0;
				skewer_gain=0;
			}
			if (g_skewer_displacement < Skewer_limit) {
				// wall_normal.error = skewer_lpf * wall_normal.error + (1 - skewer_lpf) * skewer_gain
				// 		* (-(g_sensor_max_l - CENTER_L_PILLAR) / g_sensor_max_l
				// 				+ (g_sensor_max_r - CENTER_R_PILLAR) / g_sensor_max_r);
				wall_normal.error =	skewer_gain * (-(g_sensor_max_l - CENTER_L_PILLAR) / g_sensor_max_l
								+ (g_sensor_max_r - CENTER_R_PILLAR) / g_sensor_max_r);
				wall_normal.delta_error = wall_normal.error - wall_normal.old_error;
				wall_normal.old_error = wall_normal.error;
				PID_wall = sensor_gain_p * wall_normal.error
					+ sensor_gain_d * wall_normal.delta_error;								
			} else {
				wall_normal.error = 0 ;
				wall_normal.old_error = 0;
				PID_wall = 0;
			}

			pl_yellow_LED_count(0);
			break;
		case 1:			//左壁のみ
			wall_normal.error = (-2 * (float) (sensorWall_L - CENTER_L)
					/ (float) (sensorWall_L));
			wall_normal.delta_error = wall_normal.error - wall_normal.old_error;
			wall_normal.old_error = wall_normal.error;
			PID_wall = sensor_gain_p * wall_normal.error
					+ sensor_gain_d * wall_normal.delta_error;
			pl_yellow_LED_count(128);
			break;
		case 2:			//右壁のみ
			wall_normal.error = (2 * (float) (sensorWall_R - CENTER_R)
					/ (float) (sensorWall_R));
			wall_normal.delta_error = wall_normal.error - wall_normal.old_error;
			wall_normal.old_error = wall_normal.error;
			PID_wall = sensor_gain_p * wall_normal.error
					+ sensor_gain_d * wall_normal.delta_error;
			pl_yellow_LED_count(2);
			break;
		case 3:			//両壁あり
			wall_normal.error =
					(-(float) (sensorWall_L - CENTER_L) / (float) (sensorWall_L)
							+ (float) (sensorWall_R - CENTER_R)
									/ (float) (sensorWall_R));
			wall_normal.delta_error = wall_normal.error - wall_normal.old_error;
			wall_normal.old_error = wall_normal.error;
			PID_wall = sensor_gain_p * wall_normal.error
					+ sensor_gain_d * wall_normal.delta_error;
			pl_yellow_LED_count(129);
			if ((fabs(g_sensor_diff[SENSOR_LEFT]) < 30)
					&& (fabs(g_sensor_diff[SENSOR_RIGHT]) < 30)) {
				StabilityCount_reset++;
			} else {
				StabilityCount_reset = 0;
			}
			if (StabilityCount_reset >= 25) {
				Gyro.sigma_error = 0;
				yaw_angle=0;
				StabilityCount_reset = 0;
				pl_yellow_LED_on();
			}
			break;
		}

	} else if (g_WallControl_mode == 2) {
// 斜めの制御


	} else if (g_WallControl_mode == 3) {
		// 斜めの制御(平松さん式)
		//CONTROLWALL_THRESHOLD_Lを基準からの閾値に可変式にする
		//g_WallControlStatus=3;
		float CenterSlantR, CenterSlantL;
		float coefficientR[4];
		float coefficientL[4];
		float sensor_gain_slant90_p, sensor_gain_slant90_d;
		if (straight.velocity > 6000) {
			sensor_gain_slant90_p = SENSOR_GAIN_SLANT90_P * 6000;
			sensor_gain_slant90_d = SENSOR_GAIN_SLANT90_D * 6000;
		} else {
			sensor_gain_slant90_p = SENSOR_GAIN_SLANT90_P * straight.velocity;
			sensor_gain_slant90_d = SENSOR_GAIN_SLANT90_D * straight.velocity;
		}

		if (NoWallDisplacementR45slant > AREAMIN_R0
				&& NoWallDisplacementR45slant <= AREAMAX_R0) {
			coefficientR[0] = COEFFICIENT_LIN_R0_0;
			coefficientR[1] = COEFFICIENT_LIN_R0_1;
			coefficientR[2] = COEFFICIENT_LIN_R0_2;
			coefficientR[3] = COEFFICIENT_LIN_R0_3;
		} else if (NoWallDisplacementR45slant > AREAMIN_R1
				&& NoWallDisplacementR45slant <= AREAMAX_R1) {
			coefficientR[0] = COEFFICIENT_LIN_R1_0;
			coefficientR[1] = COEFFICIENT_LIN_R1_1;
			coefficientR[2] = COEFFICIENT_LIN_R1_2;
			coefficientR[3] = COEFFICIENT_LIN_R1_3;
		} else if (NoWallDisplacementR45slant > AREAMIN_R2
				&& NoWallDisplacementR45slant <= AREAMAX_R2) {
			coefficientR[0] = COEFFICIENT_LIN_R2_0;
			coefficientR[1] = COEFFICIENT_LIN_R2_1;
			coefficientR[2] = COEFFICIENT_LIN_R2_2;
			coefficientR[3] = COEFFICIENT_LIN_R2_3;
		} else {
			//制御の無効化
			if (g_sensor[SENSOR_RIGHT][0] > CONTROLWALL_THRESHOLD_SLANT_R) {
				g_WallControlStatus = g_WallControlStatus | (1 << 1);
			} else {
				g_WallControlStatus = g_WallControlStatus & ~(1 << 1);
			}
			coefficientR[0] = (float) (g_sensor_distance_slant[SENSOR_RIGHT][0]);
			coefficientR[1] = 0;
			coefficientR[2] = 0;
			coefficientR[3] = 0;
		}

		if (NoWallDisplacementL45slant > AREAMIN_L0
				&& NoWallDisplacementL45slant <= AREAMAX_L0) {
			coefficientL[0] = COEFFICIENT_LIN_L0_0;
			coefficientL[1] = COEFFICIENT_LIN_L0_1;
			coefficientL[2] = COEFFICIENT_LIN_L0_2;
			coefficientL[3] = COEFFICIENT_LIN_L0_3;
		} else if (NoWallDisplacementL45slant > AREAMIN_L1
				&& NoWallDisplacementL45slant <= AREAMAX_L1) {
			coefficientL[0] = COEFFICIENT_LIN_L1_0;
			coefficientL[1] = COEFFICIENT_LIN_L1_1;
			coefficientL[2] = COEFFICIENT_LIN_L1_2;
			coefficientL[3] = COEFFICIENT_LIN_L1_3;
		} else if (NoWallDisplacementL45slant > AREAMIN_L2
				&& NoWallDisplacementL45slant <= AREAMAX_L2) {
			coefficientL[0] = COEFFICIENT_LIN_L2_0;
			coefficientL[1] = COEFFICIENT_LIN_L2_1;
			coefficientL[2] = COEFFICIENT_LIN_L2_2;
			coefficientL[3] = COEFFICIENT_LIN_L2_3;
		} else {
			//制御の無効化
			if (g_sensor[SENSOR_LEFT][0] > CONTROLWALL_THRESHOLD_SLANT_L) {
				g_WallControlStatus = g_WallControlStatus | (1 << 0);
			} else {
				g_WallControlStatus = g_WallControlStatus & ~(1 << 0);
			}
			coefficientL[0] = (float) (g_sensor_distance_slant[SENSOR_LEFT][0]);
			coefficientL[1] = 0;
			coefficientL[2] = 0;
			coefficientL[3] = 0;
		}

		CenterSlantR = coefficientR[0]
				+ coefficientR[1] * NoWallDisplacementR45slant
				+ coefficientR[2] * NoWallDisplacementR45slant
						* NoWallDisplacementR45slant
				+ coefficientR[3] * NoWallDisplacementR45slant
						* NoWallDisplacementR45slant
						* NoWallDisplacementR45slant;

		CenterSlantL = coefficientL[0]
				+ coefficientL[1] * NoWallDisplacementL45slant
				+ coefficientL[2] * NoWallDisplacementL45slant
						* NoWallDisplacementL45slant
				+ coefficientL[3] * NoWallDisplacementL45slant
						* NoWallDisplacementL45slant
						* NoWallDisplacementL45slant;
		for (int j = 11; j >= 1; j--) {
			g_CenterSlantR90[j] = g_CenterSlantR90[j - 1];
			g_CenterSlantL90[j] = g_CenterSlantL90[j - 1];
		}
		g_CenterSlantR90[0] = CenterSlantR;
		g_CenterSlantL90[0] = CenterSlantL;

		g_CenterSlantR90_diff = g_CenterSlantR90[0] - g_CenterSlantR90[4];
		g_CenterSlantL90_diff = g_CenterSlantL90[0] - g_CenterSlantL90[4];

		if (g_sensor_distance_slant[SENSOR_RIGHT][0] < CONTROLWALL_THRESHOLD_SLANT_R
				&& fabs(g_sensor_distance_slant_diff[SENSOR_RIGHT] - g_CenterSlantR90_diff)
						< CONTROLWALLCUT_THRESHOLD_SLANT90_R) {
			g_WallControlStatus = g_WallControlStatus | (1 << 1);
		}
		if (g_sensor_distance_slant[SENSOR_LEFT][0] < CONTROLWALL_THRESHOLD_SLANT_L
				&& fabs(g_sensor_distance_slant_diff[SENSOR_LEFT] - g_CenterSlantL90_diff)
						< CONTROLWALLCUT_THRESHOLD_SLANT90_L) {
			g_WallControlStatus = g_WallControlStatus | (1 << 0);
		}
		if (g_sensor_distance_slant[SENSOR_RIGHT][0] > CONTROLWALL_THRESHOLD_SLANT_R
				|| fabs(g_sensor_distance_slant_diff[SENSOR_RIGHT] - g_CenterSlantR90_diff)
						> CONTROLWALLCUT_THRESHOLD_SLANT90_R) {
			g_WallControlStatus = g_WallControlStatus & ~(1 << 1);
		}
		if (g_sensor_distance_slant[SENSOR_LEFT][0] > CONTROLWALL_THRESHOLD_SLANT_L
				|| fabs(g_sensor_distance_slant_diff[SENSOR_LEFT] - g_CenterSlantL90_diff)
						> CONTROLWALLCUT_THRESHOLD_SLANT90_L) {
			g_WallControlStatus = g_WallControlStatus & ~(1 << 0);
		}
		g_log_CenterSlantR90 = CenterSlantR;
		g_log_CenterSlantL90 = CenterSlantL;

		switch (g_WallControlStatus) {
		case 0:			//両壁なし
			PID_wall = 0;
			pl_yellow_LED_1(0);
			pl_yellow_LED_8(0);
			wall_slant90.error = 0;
			wall_slant90.delta_error = 0;
			wall_slant90.old_error = 0;
			break;
		case 1:			//左壁のみ
			pl_yellow_LED_1(0);
			pl_yellow_LED_8(1);
			wall_slant90.error = ((float) (g_sensor_distance_slant[SENSOR_LEFT][0]
					- CenterSlantL));
			wall_slant90.delta_error = wall_slant90.error
					- wall_slant90.old_error;
			wall_slant90.old_error = wall_slant90.error;
			PID_wall = sensor_gain_slant90_p * wall_slant90.error
					+ sensor_gain_slant90_d * wall_slant90.delta_error;
			break;
		case 2:			//右壁のみ
			pl_yellow_LED_1(1);
			pl_yellow_LED_8(0);
			wall_slant90.error = (-(float) (g_sensor_distance_slant[SENSOR_RIGHT][0]
					- CenterSlantR));
			wall_slant90.delta_error = wall_slant90.error
					- wall_slant90.old_error;
			wall_slant90.old_error = wall_slant90.error;
			PID_wall = sensor_gain_slant90_p * wall_slant90.error
					+ sensor_gain_slant90_d * wall_slant90.delta_error;
			break;
		case 3:			//両壁あり
			pl_yellow_LED_1(1);
			pl_yellow_LED_8(1);
			wall_slant90.error = ((float) (g_sensor_distance_slant[SENSOR_LEFT][0]
					- CenterSlantL)
					- (float) (g_sensor_distance_slant[SENSOR_RIGHT][0] - CenterSlantR)
							);
			wall_slant90.delta_error = wall_slant90.error
					- wall_slant90.old_error;
			wall_slant90.old_error = wall_slant90.error;
			PID_wall = sensor_gain_slant90_p * wall_slant90.error
					+ sensor_gain_slant90_d * wall_slant90.delta_error;

			if ((fabs(g_sensor_distance_slant_diff[SENSOR_LEFT]) - g_CenterSlantL90_diff < 30)
					&& (fabs(g_sensor_distance_slant_diff[SENSOR_RIGHT]) - g_CenterSlantR90_diff < 30)) {
				StabilityCount_reset++;
			} else {
				StabilityCount_reset = 0;
			}
			if (StabilityCount_reset >= 40) {
				Gyro.sigma_error = 0;
				yaw_angle=0;
				StabilityCount_reset = 0;
				pl_yellow_LED_on();
			}


			break;
		}
// 斜め45度
		float CenterSlantR45, CenterSlantL45;
		float coefficientR45[4];
		float coefficientL45[4];
		float sensor_gain_slant45_p, sensor_gain_slant45_d;
		if (straight.velocity > 6000) {
			sensor_gain_slant45_p = SENSOR_GAIN_SLANT45_P * 6000;
			sensor_gain_slant45_d = SENSOR_GAIN_SLANT45_D * 6000;
		} else {
			sensor_gain_slant45_p = SENSOR_GAIN_SLANT45_P * straight.velocity;
			sensor_gain_slant45_d = SENSOR_GAIN_SLANT45_D * straight.velocity;
		}

		if (NoWallDisplacementR45slant > AREAMIN45_R0
				&& NoWallDisplacementR45slant <= AREAMAX45_R0) {
			coefficientR45[0] = COEFFICIENT45_LIN_R0_0;
			coefficientR45[1] = COEFFICIENT45_LIN_R0_1;
			coefficientR45[2] = COEFFICIENT45_LIN_R0_2;
			coefficientR45[3] = COEFFICIENT45_LIN_R0_3;
		} else {
			//制御の無効化
			if (g_sensor[SENSOR_FRONT_RIGHT][0]
					> CONTROLWALL_THRESHOLD_SLANT45_R) {
				g_WallControlStatus45 = g_WallControlStatus45 | (1 << 1);
			} else {
				g_WallControlStatus45 = g_WallControlStatus45 & ~(1 << 1);
			}
			coefficientR45[0] = g_sensor_distance_slant[SENSOR_FRONT_RIGHT][0];
			coefficientR45[1] = 0;
			coefficientR45[2] = 0;
			coefficientR45[3] = 0;
		}

		if (NoWallDisplacementL45slant > AREAMIN45_L0
				&& NoWallDisplacementL45slant <= AREAMAX45_L0) {
			coefficientL45[0] = COEFFICIENT45_LIN_L0_0;
			coefficientL45[1] = COEFFICIENT45_LIN_L0_1;
			coefficientL45[2] = COEFFICIENT45_LIN_L0_2;
			coefficientL45[3] = COEFFICIENT45_LIN_L0_3;
		} else {
			//制御の無効化
			if (g_sensor[SENSOR_FRONT_LEFT][0] > CONTROLWALL_THRESHOLD_SLANT45_L) {
				g_WallControlStatus45 = g_WallControlStatus45 | (1 << 0);
			} else {
				g_WallControlStatus45 = g_WallControlStatus45 & ~(1 << 0);
			}
			coefficientL45[0] = g_sensor_distance_slant[SENSOR_FRONT_LEFT][0];
			coefficientL45[1] = 0;
			coefficientL45[2] = 0;
			coefficientL45[3] = 0;
		}
		CenterSlantR45 = coefficientR45[0]
				+ coefficientR45[1] * NoWallDisplacementR45slant
				+ coefficientR45[2] * NoWallDisplacementR45slant
						* NoWallDisplacementR45slant
				+ coefficientR45[3] * NoWallDisplacementR45slant
						* NoWallDisplacementR45slant
						* NoWallDisplacementR45slant;
		CenterSlantL45 = coefficientL45[0]
				+ coefficientL45[1] * NoWallDisplacementL45slant
				+ coefficientL45[2] * NoWallDisplacementL45slant
						* NoWallDisplacementL45slant
				+ coefficientL45[3] * NoWallDisplacementL45slant
						* NoWallDisplacementL45slant
						* NoWallDisplacementL45slant;

		for (int j = 11; j >= 1; j--) {
			g_CenterSlantR45[j] = g_CenterSlantR45[j - 1];
			g_CenterSlantL45[j] = g_CenterSlantL45[j - 1];
		}
		g_CenterSlantR45[0] = CenterSlantR45;
		g_CenterSlantL45[0] = CenterSlantL45;

		g_CenterSlantR45_diff = g_CenterSlantR45[0] - g_CenterSlantR45[4];
		g_CenterSlantL45_diff = g_CenterSlantL45[0] - g_CenterSlantL45[4];

		if (g_sensor_distance_slant[SENSOR_FRONT_RIGHT][0] < CONTROLWALL_THRESHOLD_SLANT45_R
				&& fabs(
						g_sensor_distance_slant_diff[SENSOR_FRONT_RIGHT]
								- g_CenterSlantR45_diff)
						< CONTROLWALLCUT_THRESHOLD_SLANT45_R) {
			g_WallControlStatus45 = g_WallControlStatus45 | (1 << 1);
		}
		if (g_sensor_distance_slant[SENSOR_FRONT_LEFT][0] < CONTROLWALL_THRESHOLD_SLANT45_L
				&& fabs(
						g_sensor_distance_slant_diff[SENSOR_FRONT_LEFT]
								- g_CenterSlantL45_diff)
						< CONTROLWALLCUT_THRESHOLD_SLANT45_L) {
			g_WallControlStatus45 = g_WallControlStatus45 | (1 << 0);
		}
		if (g_sensor_distance_slant[SENSOR_FRONT_RIGHT][0] > CONTROLWALL_THRESHOLD_SLANT45_R
				|| fabs(
						g_sensor_distance_slant_diff[SENSOR_FRONT_RIGHT]
								- g_CenterSlantR45_diff)
						> CONTROLWALLCUT_THRESHOLD_SLANT45_R) {
			g_WallControlStatus45 = g_WallControlStatus45 & ~(1 << 1);
		}
		if (g_sensor_distance_slant[SENSOR_FRONT_LEFT][0] > CONTROLWALL_THRESHOLD_SLANT45_L
				|| fabs(
						g_sensor_distance_slant_diff[SENSOR_FRONT_LEFT]
								- g_CenterSlantL45_diff)
						> CONTROLWALLCUT_THRESHOLD_SLANT45_L) {
			g_WallControlStatus45 = g_WallControlStatus45 & ~(1 << 0);
		}

		g_log_CenterSlantR45 = CenterSlantR45;
		g_log_CenterSlantL45 = CenterSlantL45;

		switch (g_WallControlStatus45) {
		case 0:			//両壁なし
			PID_wall += 0;
			pl_yellow_LED_3(0);
			pl_yellow_LED_6(0);
			wall_slant45.error = 0;
			wall_slant45.delta_error = 0;
			wall_slant45.old_error = 0;
			break;
		case 1:			//左壁のみ
			pl_yellow_LED_3(0);
			pl_yellow_LED_6(1);
			wall_slant45.error =
					((float) (g_sensor_distance_slant[SENSOR_FRONT_LEFT][0] - CenterSlantL45)
							);
			wall_slant45.delta_error = wall_slant45.error
					- wall_slant45.old_error;
			wall_slant45.old_error = wall_slant45.error;
			PID_wall += sensor_gain_slant45_p * wall_slant45.error
					+ sensor_gain_slant45_d * wall_slant45.delta_error;
			break;
		case 2:			//右壁のみ
			pl_yellow_LED_3(1);
			pl_yellow_LED_6(0);
			wall_slant45.error = (-(float) (g_sensor_distance_slant[SENSOR_FRONT_RIGHT][0]
					- CenterSlantR45)
					);
			wall_slant45.delta_error = wall_slant45.error
					- wall_slant45.old_error;
			wall_slant45.old_error = wall_slant45.error;
			PID_wall += sensor_gain_slant45_p * wall_slant45.error
					+ sensor_gain_slant45_d * wall_slant45.delta_error;
			break;
		case 3:			//両壁あり
			pl_yellow_LED_3(1);
			pl_yellow_LED_6(1);
			wall_slant45.error = ((float) (g_sensor_distance_slant[SENSOR_FRONT_LEFT][0]
					- CenterSlantL45)
					- (float) (g_sensor_distance_slant[SENSOR_FRONT_RIGHT][0] - CenterSlantR45)
							);
			wall_slant45.delta_error = wall_slant45.error
					- wall_slant45.old_error;
			wall_slant45.old_error = wall_slant45.error;
			PID_wall += sensor_gain_slant45_p * wall_slant45.error
					+ sensor_gain_slant45_d * wall_slant45.delta_error;

			if ((fabs(g_sensor_distance_slant_diff[SENSOR_FRONT_LEFT] - g_CenterSlantL45_diff) < 30)
					&& (fabs(g_sensor_distance_slant_diff[SENSOR_FRONT_RIGHT] - g_CenterSlantR45_diff) < 30)) {
				StabilityCount_reset++;
			} else {
				StabilityCount_reset = 0;
			}
			if (StabilityCount_reset >= 35) {
				Gyro.sigma_error = 0;
				yaw_angle=0;
				StabilityCount_reset = 0;
				pl_yellow_LED_on();
			}

			break;
		}

		//柱を避ける制御
		if (g_sensor[SENSOR_FRONT_L][0] > CONTROLWALL_THRESHOLD_FRONT_L
		 && g_sensor[SENSOR_FRONT_R][0] > CONTROLWALL_THRESHOLD_FRONT_R) {

		} else if (g_sensor[SENSOR_FRONT_L][0] > CONTROLWALL_THRESHOLD_FRONT_L
				&& g_sensor[SENSOR_FRONT_R][0] <= CONTROLWALL_THRESHOLD_FRONT_R) {
//			if(g_sensor[SENSOR_FRONT_L][0] - CONTROLWALL_THRESHOLD_FRONT_L<200){
			PID_wall += SENSOR_GAIN_SLANT
					* ((float) (g_sensor[SENSOR_FRONT_L][0]
							- CONTROLWALL_THRESHOLD_FRONT_L));
//			}else{
//				PID_wall += SENSOR_GAIN_SLANT * 200;
//			}
		} else if (g_sensor[SENSOR_FRONT_L][0] <= CONTROLWALL_THRESHOLD_FRONT_L
				&& g_sensor[SENSOR_FRONT_R][0] > CONTROLWALL_THRESHOLD_FRONT_R) {
//			if(g_sensor[SENSOR_FRONT_R][0] - CONTROLWALL_THRESHOLD_FRONT_R<200){
				PID_wall += SENSOR_GAIN_SLANT
									* (-(float) (g_sensor[SENSOR_FRONT_R][0]
											- CONTROLWALL_THRESHOLD_FRONT_R));
//			}else{
//							PID_wall -= SENSOR_GAIN_SLANT * 200;
//			}
		} else if (g_sensor[SENSOR_FRONT_L][0] <= CONTROLWALL_THRESHOLD_FRONT_L
				&& g_sensor[SENSOR_FRONT_R][0] <= CONTROLWALL_THRESHOLD_FRONT_R) {

		}

	}
	PID_w = PID_wall / MAXMOTOR * g_V_battery_mean;

	return PID_w;
}

void calFrontWallConrol_one(float *PID_s) {

	if (g_FrontWallControl_mode == 0) {
		*PID_s = 0;
	} else if (g_FrontWallControl_mode == 1) {
		if (g_sensor[SENSOR_FRONT_L][0] > F_PRESENCE2 && g_sensor[SENSOR_FRONT_R][0] > F_PRESENCE2) {
			*PID_s =
					SENSOR_FRONT_GAIN_P
							* ((float) (-g_sensor[SENSOR_FRONT_L][0]-g_sensor[SENSOR_FRONT_R][0])/2 + CENTER_FRONT_S);///??だめ

		} else {
			*PID_s = 0;
		}
	}
}


void calFrontWallConrol(float *PID_frontwall_l, float *PID_frontwall_r) {

	if (g_FrontWallControl_mode == 0) {
		*PID_frontwall_l = 0;
		*PID_frontwall_r = 0;
		wall_front_l.old_error = 0;
		wall_front_r.old_error = 0;
		wall_front_l.sigma_error = 0;
		wall_front_r.sigma_error = 0;
	} else if (g_FrontWallControl_mode == 1) {

		if( g_sensor[SENSOR_FRONT_L][0] > F_PRESENCE2 && g_sensor[SENSOR_FRONT_R][0] > F_PRESENCE2 ){
		wall_front_l.error = (-(float) (g_sensor[SENSOR_FRONT_L][0] - CENTER_FRONT_L))/((float) g_sensor[SENSOR_FRONT_L][0]+0.1);
		wall_front_r.error = (-(float) (g_sensor[SENSOR_FRONT_R][0] - CENTER_FRONT_R))/((float) g_sensor[SENSOR_FRONT_R][0]+0.1);
		wall_front_l.delta_error = wall_front_l.error - wall_front_l.old_error;
		wall_front_r.delta_error = wall_front_r.error - wall_front_r.old_error;
		wall_front_l.old_error = wall_front_l.error;
		wall_front_r.old_error = wall_front_r.error;
		wall_front_l.sigma_error += wall_front_l.error;
		wall_front_r.sigma_error += wall_front_r.error;
		*PID_frontwall_l =
				SENSOR_FRONT_GAIN_P * wall_front_l.error + SENSOR_FRONT_GAIN_I * wall_front_l.sigma_error + SENSOR_FRONT_GAIN_D * wall_front_l.delta_error; 
		*PID_frontwall_r =
				SENSOR_FRONT_GAIN_P * wall_front_r.error + SENSOR_FRONT_GAIN_I * wall_front_r.sigma_error + SENSOR_FRONT_GAIN_D * wall_front_r.delta_error; 
		}else{
			*PID_frontwall_l=0;
			*PID_frontwall_r=0;
			wall_front_l.old_error = 0;
			wall_front_r.old_error = 0;
			wall_front_l.sigma_error = 0;
			wall_front_r.sigma_error = 0;
		}


	}
}

void interrupt_WallCut(void) {

/* 速度依存でサンプリング時間を変更 */
	int del_time=8;
	int del_time_slant=ceil(3.5/INTERRUPT_TIME/fabs(straight.velocity));
	if(del_time_slant>=11){del_time_slant=11;}
	if(del_time_slant<=4){del_time_slant=4;}
	for (int j = 0; j <= 4; j++) {
		g_sensor_diff_wallcut[j]=g_sensor[j][0]-g_sensor[j][del_time];
		g_sensor_diff_wallcut_slant[j]=g_sensor[j][0]-g_sensor[j][del_time_slant];
	}


	if (g_wallCut_mode == 1) {
/* 安全機能：壁切れ距離計測オーバーの確認 */
		if(Nowall_safe_flg == 1){
			NoWallDisplacementR_safe+=kalman_speed * INTERRUPT_TIME;
			NoWallDisplacementL_safe+=kalman_speed * INTERRUPT_TIME;
		}
/* 前壁により強制オフセット除去(ほぼ未使用) */
		if (g_sensor[SENSOR_FRONT_L][0] > F_BREAK_THRESHOLD_L_90 && g_sensor[SENSOR_FRONT_R][0] > F_BREAK_THRESHOLD_R_90) {
			front_wall_break_90 = 1;
		} else {
			front_wall_break_90 = 0;
		}
		if (g_sensor[SENSOR_FRONT_L][0] > F_BREAK_THRESHOLD_L_45 && g_sensor[SENSOR_FRONT_R][0] > F_BREAK_THRESHOLD_R_45) {
			front_wall_break_45 = 1;
		} else {
			front_wall_break_45 = 0;
		}
		if (g_sensor[SENSOR_FRONT_L][0] > F_BREAK_THRESHOLD_L_45SLANT && g_sensor[SENSOR_FRONT_R][0] > F_BREAK_THRESHOLD_R_45SLANT) {
			front_wall_break_45slant = 1;
		} else {
			front_wall_break_45slant = 0;
		}

/* スラロームで用いる90度センサーの壁切れ */
		if (g_sensor[SENSOR_LEFT][0] < WALLCUT_THRESHOLD_L90) {
			NoWallCountL90++;
			NoWallDisplacementL90 += (fusion_speedL + fusion_speedR) / 2 * INTERRUPT_TIME;
		} else {
			NoWallCountL90 = 0;
			NoWallDisplacementL90 = 0;
		}

		if (g_sensor[SENSOR_RIGHT][0] < WALLCUT_THRESHOLD_R90) {
			NoWallCountR90++;
			NoWallDisplacementR90 += (fusion_speedL + fusion_speedR) / 2 * INTERRUPT_TIME;
		} else {
			NoWallCountR90 = 0;
			NoWallDisplacementR90 = 0;
		}

/* 大回りターンで用いる45度センサーの壁切れ */
		//g_sensor_diff_wallcut[SENSOR_FRONT_LEFT] > WALLCUT_THRESHOLD_DIFF_L45 &&
		if (g_sensor[SENSOR_FRONT_LEFT][0] > WALLCUT_THRESHOLD_L45) {
			NoWallCountL45++;
			NoWallCountL45_flag = 0;
			NoWallDisplacementL45 += kalman_speed * INTERRUPT_TIME;
		} else {
			if (NoWallCountL45_flag == 0) {
				NoWallCountL45 = 0;
				NoWallCountL45_flag = 1;
				NoWallDisplacementL45 = 0;
			} else {
				NoWallCountL45++;
				NoWallDisplacementL45 += kalman_speed* INTERRUPT_TIME;
			}
		}

		//g_sensor_diff_wallcut[SENSOR_FRONT_RIGHT] > WALLCUT_THRESHOLD_DIFF_R45 &&
		if (g_sensor[SENSOR_FRONT_RIGHT][0] > WALLCUT_THRESHOLD_R45) {
			NoWallCountR45++;
			NoWallCountR45_flag = 0;
			NoWallDisplacementR45 += kalman_speed * INTERRUPT_TIME;
		} else {
			if (NoWallCountR45_flag == 0) {
				NoWallCountR45 = 0;
				NoWallCountR45_flag = 1;
				NoWallDisplacementR45 = 0;
			} else {
				NoWallCountR45++;
				NoWallDisplacementR45 += kalman_speed* INTERRUPT_TIME;
			}
		}

		// 大回りターンで用いる45度センサーの壁切れ(斜め)
		if (g_sensor_diff_wallcut_slant[SENSOR_FRONT_LEFT]
				> WALLCUT_THRESHOLD_DIFF_L45_SLANT) {
			NoWallCountL45slant++;
			pl_l_blue_LED(0);
			NoWallCountL45slant_flag = 0;
			NoWallDisplacementL45slant2 += kalman_speed*INTERRUPT_TIME;//壁切れ用
			if (slantWallControlL_flag == 1) {
				NoWallDisplacementL45slant += straight.velocity*INTERRUPT_TIME;//斜め制御用
			}
		} else {
			if (NoWallCountL45slant_flag == 0) {
				NoWallCountL45slant = 0;
				NoWallCountL45slant_flag = 1;
				pl_l_blue_LED(1);
				NoWallDisplacementL45slant = 0;
				NoWallDisplacementL45slant2 = 0;

			} else {
				NoWallCountL45slant++;
				NoWallDisplacementL45slant2 += kalman_speed*INTERRUPT_TIME;
				if (slantWallControlL_flag == 1) {
					NoWallDisplacementL45slant += straight.velocity*INTERRUPT_TIME;
				}
			}
			slantWallControlL_flag = 1;
		}

		if (g_sensor_diff_wallcut_slant[SENSOR_FRONT_RIGHT]
				> WALLCUT_THRESHOLD_DIFF_R45_SLANT) {
			NoWallCountR45slant++;
			pl_r_blue_LED(0);
			NoWallCountR45slant_flag = 0;
			NoWallDisplacementR45slant2 += kalman_speed*INTERRUPT_TIME;
			if (slantWallControlR_flag == 1) {
				NoWallDisplacementR45slant += straight.velocity*INTERRUPT_TIME;
			}
		} else {
			if (NoWallCountR45slant_flag == 0) {
				NoWallCountR45slant = 0;
				NoWallCountR45slant_flag = 1;
				pl_r_blue_LED(1);
			    NoWallDisplacementR45slant = 0;
			    NoWallDisplacementR45slant2 = 0;
			} else {
				NoWallCountR45slant++;
				NoWallDisplacementR45slant2 += kalman_speed*INTERRUPT_TIME;
				if (slantWallControlR_flag == 1) {
					NoWallDisplacementR45slant += straight.velocity*INTERRUPT_TIME;
				}
			}
			slantWallControlR_flag = 1;
		}

	} else {
		NoWallCountL90 = 0;
		NoWallCountR90 = 0;
		NoWallCountL45 = 0;
		NoWallCountR45 = 0;
		NoWallCountL45slant = 0;
		NoWallCountR45slant = 0;

		NoWallDisplacementL90 = 0;
		NoWallDisplacementR90 = 0;
		NoWallDisplacementL45 = CUTPLACE_THRESHOLD_END_L45;
		NoWallDisplacementR45 = CUTPLACE_THRESHOLD_END_R45;
		NoWallDisplacementL45slant = 0;
		NoWallDisplacementR45slant = 0;
		NoWallDisplacementL45slant2 = 0;			//単純な壁切れ用
		NoWallDisplacementR45slant2 = 0;			//単純な壁切れ用

		NoWallCountL45_flag = 0;
		NoWallCountR45_flag = 0;
		NoWallCountL45slant_flag = 0;
		NoWallCountR45slant_flag = 0;
		slantWallControlL_flag = 0;
		slantWallControlR_flag = 0;

		front_wall_break_90 = 0;
		front_wall_break_45 = 0;
		front_wall_break_45slant = 0;
		front_wall_break_45slant = 0;

		g_sensor_max_fl = 0;
		g_sensor_max_fr = 0;
		g_sensor_max_fl_slant = 0;
		g_sensor_max_fr_slant = 0;

	}

}
