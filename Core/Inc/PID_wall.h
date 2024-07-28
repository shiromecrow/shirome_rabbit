/*
 * PID_wall.h
 *
 *  Created on: 2023/01/17
 *      Author: sf199
 */

#ifndef INC_PID_WALL_H_
#define INC_PID_WALL_H_


#include "stm32g4xx_hal.h"
#include "define.h"
//#define CONTROLLEFTWALL 1
//#define CONTROLRIGHTWALL 2

// 壁制御に使用する値
#define SENSOR_GAIN_P 1.2//0.16横壁制御のゲイン
#define SENSOR_GAIN_D 0.006//0.16横壁制御のゲイン
//#define SENSOR_GAIN_SHORT 0.5*800/1200//0.16横壁制御のゲイン最短用
#define SENSOR_GAIN_SHORT_P 0.3//0.16横壁制御のゲイン最短用
#define SENSOR_GAIN_SHORT_D -0.003//0.16横壁制御のゲイン最短用
#define	CENTER_L	2090
#define	CENTER_R 	1600
#define	CONTROLWALL_THRESHOLD_L	822
#define	CONTROLWALL_THRESHOLD_R	606
#define	CONTROLWALLCUT_THRESHOLD_L 	207
#define	CONTROLWALLCUT_THRESHOLD_R	148

#define CONTROLWALLCUT_THRESHOLD_SHORT_L 180//左壁制御の壁切れの閾値
#define CONTROLWALLCUT_THRESHOLD_SHORT_R 150//右壁制御の壁切れの閾値
#define	SENSOR_L_MIN	922
#define	SENSOR_L_MAX	2900
#define	SENSOR_R_MIN	706
#define	SENSOR_R_MAX	2600


//串制御
#define	CENTER_L_PILLAR	1041
#define	CENTER_R_PILLAR 705
#define SKEWER_LIMIT 30//左90
#define SKEWER_LIMIT_SHORT 80//左90
#define	PILLAR_THRESHOLD_L	250
#define	PILLAR_THRESHOLD_R 200

//前壁制御3348,SEN3=1366,SEN4=1710
#define SENSOR_FRONT_GAIN 1.6
#define CENTER_FRONT_S 3070//前壁制御
#define CENTER_FRONT_L 1575//前壁制御
#define CENTER_FRONT_R 1879//前壁制御
#define F_PRESENCE2 1100

//壁切れ
#define	WALLCUT_THRESHOLD_L90	417
#define	WALLCUT_THRESHOLD_R90	274
//#define WALLCUT_THRESHOLD_L45 100
//#define WALLCUT_THRESHOLD_R45 80
#define	WALLCUT_THRESHOLD_L45	60
#define	WALLCUT_THRESHOLD_R45	40
#define	WALLCUT_THRESHOLD_DIFF_L45	-52
#define	WALLCUT_THRESHOLD_DIFF_R45	-52
#define WALLCUT_THRESHOLD_DIFF_L45_SLANT -130
#define WALLCUT_THRESHOLD_DIFF_R45_SLANT -130//-1200

//壁切れの位置と区画の境目の距離(壁切れ位置-区画の境目)
#define CUTPLACE_TO_CENTER_L90 13
#define CUTPLACE_TO_CENTER_R90 13
#define CUTPLACE_TO_CENTER_L45 0//10
#define CUTPLACE_TO_CENTER_R45 2//10
//#define CUTPLACE_TO_CENTER_L90_2 -60
//#define CUTPLACE_TO_CENTER_R90_2 -60//大回りも90°を使用
#define CUTPLACE_TO_CENTER_L45_SLANT 9//17
#define CUTPLACE_TO_CENTER_R45_SLANT 10//20


//壁切れの距離補正(終了位置)壁切れてから臨界の
#define CUTPLACE_THRESHOLD_END_L90 55
#define CUTPLACE_THRESHOLD_END_R90 55
#define CUTPLACE_THRESHOLD_END_L90_2 25
#define CUTPLACE_THRESHOLD_END_R90_2 25
#define CUTPLACE_THRESHOLD_END_L45 22
#define CUTPLACE_THRESHOLD_END_R45 22
#define CUTPLACE_THRESHOLD_END_L45_SLANT 31//105
#define CUTPLACE_THRESHOLD_END_R45_SLANT 31//105

//壁切れ補正用(壁切れ時の横距離があった場合の縦補正) 未使用
#define CENTER_FL 493//左90
#define CENTER_FR 801//右90
#define CENTER_FL_SLANT 3500//左90
#define CENTER_FR_SLANT 3500//右90
#define GAIN_WALLCUT 0//30/6
#define GAIN_WALLCUT_SLANT 0//210/2//中心より大きい方
#define GAIN_WALLCUT_SLANT2 0//4.8/2//中心より大きい方

// 斜めの壁制御(柱を避ける)
#define SENSOR_GAIN_SLANT 0.3
#define CONTROLWALL_THRESHOLD_FRONT_L 130//斜め用壁制御
#define CONTROLWALL_THRESHOLD_FRONT_R 140//斜め用壁制御


//斜めの制御の変数
#define SENSOR_GAIN_SLANT90_P 1.7//0.099*800/170//0.081
#define SENSOR_GAIN_SLANT90_D -0.16//0.018*800/170//0.018
#define CONTROLWALL_THRESHOLD_SLANT_L 180//左壁制御の閾値110
#define CONTROLWALL_THRESHOLD_SLANT_R 400//右壁制御の閾値110
#define CONTROLWALLCUT_THRESHOLD_SLANT90_L 80//左壁制御の壁切れの閾値
#define CONTROLWALLCUT_THRESHOLD_SLANT90_R 80//右壁制御の壁切れの閾値

#define SENSOR_GAIN_SLANT45_P 1.6//0.081*800/170//0.081
#define SENSOR_GAIN_SLANT45_D -0.12//0.018*800/170//0.018
#define CONTROLWALL_THRESHOLD_SLANT45_R 220//New
#define CONTROLWALL_THRESHOLD_SLANT45_L 220//New
#define CONTROLWALLCUT_THRESHOLD_SLANT45_L 80//左壁制御の壁切れの閾値
#define CONTROLWALLCUT_THRESHOLD_SLANT45_R 80//右壁制御の壁切れの閾値
//#define CONTROLWALLCUT_THRESHOLD_SLANT_L 100//左壁制御の壁切れの閾値
//#define CONTROLWALLCUT_THRESHOLD_SLANT_R 100//右壁制御の壁切れの閾値

//可制御の領域
#define AREAMIN_R0 1
#define AREAMAX_R0 15
#define AREAMIN_R1 35
#define AREAMAX_R1 60
#define AREAMIN_R2 100//
#define AREAMAX_R2 120

#define AREAMIN_L0 1
#define AREAMAX_L0 15
#define AREAMIN_L1 35
#define AREAMAX_L1 60
#define AREAMIN_L2 100
#define AREAMAX_L2 120

#define AREAMIN45_R0 20
#define AREAMAX45_R0 80
#define AREAMIN45_L0 20
#define AREAMAX45_L0 80

//近似直線の係数(matlabから算出)

#define	COEFFICIENT_R0_0	576.7807915
#define	COEFFICIENT_R0_1	31.0094928
#define	COEFFICIENT_R0_2	0.15877754
#define	COEFFICIENT_R0_3	4.20E-02
#define	COEFFICIENT_R1_0	10780.06411
#define	COEFFICIENT_R1_1	-515.8901557
#define	COEFFICIENT_R1_2	8.781489544
#define	COEFFICIENT_R1_3	-0.051580091
#define	COEFFICIENT_R2_0	4166.67337
#define	COEFFICIENT_R2_1	-91.30728409
#define	COEFFICIENT_R2_2	0.617576638
#define	COEFFICIENT_R2_3	-9.56E-04
#define	COEFFICIENT_L0_0	685.9925378
#define	COEFFICIENT_L0_1	32.55375474
#define	COEFFICIENT_L0_2	0.153143407
#define	COEFFICIENT_L0_3	0.037171369
#define	COEFFICIENT_L1_0	13589.06532
#define	COEFFICIENT_L1_1	-652.6942915
#define	COEFFICIENT_L1_2	11.32106311
#define	COEFFICIENT_L1_3	-0.068318617
#define	COEFFICIENT_L2_0	10074.15369
#define	COEFFICIENT_L2_1	-240.0357781
#define	COEFFICIENT_L2_2	1.86058254
#define	COEFFICIENT_L2_3	-4.38E-03












#define	COEFFICIENT45_R0_0	107.7811678
#define	COEFFICIENT45_R0_1	2.291060948
#define	COEFFICIENT45_R0_2	-0.022861097
#define	COEFFICIENT45_R0_3	5.22E-04
#define	COEFFICIENT45_L0_0	74.13490483
#define	COEFFICIENT45_L0_1	2.28894082
#define	COEFFICIENT45_L0_2	-0.033024466
#define	COEFFICIENT45_L0_3	0.000562175
















#define F_BREAK_THRESHOLD_L_90 1000
#define F_BREAK_THRESHOLD_L_45 300
#define F_BREAK_THRESHOLD_L_45SLANT 500
#define F_BREAK_THRESHOLD_R_90 1000
#define F_BREAK_THRESHOLD_R_45 300
#define F_BREAK_THRESHOLD_R_45SLANT 500

//typedef struct {
//	uint8_t g_WallControl_mode;//0で壁制御なし、1で通常の壁制御、2で斜めの制御
//	uint8_t g_WallControlStatus;
//
//
//}CONTROLWALL_MODE;
typedef struct{

	float error;
	float old_error;
//	float sigma_error;
	float delta_error;

}PIDW;

extern PIDW wall_normal;
extern PIDW wall_slant90;
extern PIDW wall_slant45;

extern uint8_t g_WallControl_mode;
extern uint8_t g_FrontWallControl_mode;
extern uint8_t g_wallCut_mode;

extern uint8_t g_WallControlStatus;
extern uint8_t g_WallControlStatus45;

extern float g_log_CenterSlantR45, g_log_CenterSlantL45;//log用
extern float g_log_CenterSlantR90, g_log_CenterSlantL90;//log用


extern volatile uint32_t NoWallCountR90, NoWallCountL90, NoWallCountR45, NoWallCountL45;
extern volatile uint32_t NoWallCountR45slant, NoWallCountL45slant;

extern volatile float NoWallDisplacementR90, NoWallDisplacementL90, NoWallDisplacementR45, NoWallDisplacementL45;
extern volatile float NoWallDisplacementR45slant, NoWallDisplacementL45slant;
extern volatile float NoWallDisplacementR45slant2, NoWallDisplacementL45slant2;

extern volatile  uint8_t front_wall_break_90,front_wall_break_45,front_wall_break_45slant;

void init_WallControl();

float calWallConrol();

void calFrontWallConrol_one(float *);
void calFrontWallConrol(float *,float *);

void interrupt_WallCut(void);


#endif /* INC_PID_WALL_H_ */
