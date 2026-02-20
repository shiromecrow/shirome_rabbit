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

#include <stdio.h>
#include <math.h>

#include "define.h"

#include "PL_timer.h"
#include "PL_encoder.h"
#include "PL_sensor.h"
#include "CL_sensor.h"
#include "CL_EnoderGyro.h"
#include "Control_motor.h"
#include "FF_motor.h"
#include "PID_wall.h"
#include "cal_acceleration.h"


//#include "motor_control.h"
//#include "PID_wall.h"

float record_value[MAX_RECORD_NUM][MAX_RECORD_TIME];

short record_mode;
short head_record_mode;

int sample_time;/* サンプリング時間 [0.5ms] */
int sample_count;/* サンプリング観測用のカウント値 */

int record_time;
int record_end_point;
char record_rupe_flag;

float record_buf;

char recordstop_count = 0;

//int SEN_record[5][15];
//int SEN_recordD[5][15];


// ------------------------------------------------------------
// 各モード定義（record_mode = 1〜36）
// request == 0 のときは従来通り d に値を書き込む
// ------------------------------------------------------------

// モード01: エンコーダ速度・距離
void mode1(float* d, const char* header_out[], int request) {

    if (request == 0) {
        d[0] = E_speedR;
        d[1] = E_speedL;
        d[2] = E_distanceR;
        d[3] = E_distanceL;
        return;
    }
    header_out[0] = "ActStraVelR";
    header_out[1] = "ActStraVelL";
    header_out[2] = "ActStraDisR";
    header_out[3] = "ActStraDisL";
}



// モード02: 回転・直進・推定速度比較
void mode2(float* d, const char* header_out[], int request) {
    if (request == 0) {
        d[0] = turning.velocity;
        d[1] = angle_speed;
        d[2] = straight.velocity;
        d[3] = kalman_speed;
        return;
    }
    header_out[0] = "TarTurnVel";
    header_out[1] = "ActTurnVel";
    header_out[2] = "TarStraVel";
    header_out[3] = "ActKalVel";
}

// モード03: 直進距離・速度と推定値比較
void mode3(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = straight.velocity;
    d[1] = straight.displacement;
    d[2] = kalman_speed;
    d[3] = kalman_distance;
        return;
    }
    header_out[0] = "TarStraVel";
    header_out[1] = "TarStraDis";
    header_out[2] = "ActKalVel";
    header_out[3] = "ActKalDis";
}

// モード04: 距離比較（直進 vs エンコーダ vs 推定）
void mode4(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = straight.displacement;
    d[1] = (E_distanceR + E_distanceL) / 2.0f;
    d[2] = gf_distance;
    d[3] = (fusion_distanceR + fusion_distanceL) / 2.0f;
        return;
    }
    header_out[0] = "TarStraDis";
    header_out[1] = "ActStraDis";
    header_out[2] = "ActAccDis";
    header_out[3] = "ActFusDis";
}

// モード05: 速度比較（直進 vs エンコーダ vs 推定）
void mode5(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = straight.velocity;
    d[1] = (E_speedL + E_speedR) / 2.0f;
    d[2] = gf_speed;
    d[3] = kalman_speed;
        return;
    }
    header_out[0] = "TarStraVel";
    header_out[1] = "ActStraVel";
    header_out[2] = "ActAccVel";
    header_out[3] = "ActKalVel";
}

// モード06: LPF速度比較
void mode6(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = straight.velocity;
    d[1] = E_lpf_speedL;
    d[2] = E_lpf_speedR;
    d[3] = gf_speed;
        return;
    }
    header_out[0] = "TarStraVel";
    header_out[1] = "ActLpfVelL";
    header_out[2] = "ActLpfVelR";
    header_out[3] = "ActAccVel";
}

// モード07: 左右壁センサの生値と差分
void mode7(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = g_sensor[SENSOR_LEFT][0];
    d[1] = g_sensor_diff[SENSOR_LEFT];
    d[2] = g_sensor[SENSOR_RIGHT][0];
    d[3] = g_sensor_diff[SENSOR_RIGHT];
        return;
    }
    header_out[0] = "SenleftVal";
    header_out[1] = "SenleftDiff";
    header_out[2] = "SenRightVal";
    header_out[3] = "SenRightDiff";
}

// モード08: 前壁センサの生値と差分
void mode8(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = g_sensor[SENSOR_FRONT_LEFT][0];
    d[1] = g_sensor_diff_wallcut[SENSOR_FRONT_LEFT];
    d[2] = g_sensor[SENSOR_FRONT_RIGHT][0];
    d[3] = g_sensor_diff_wallcut[SENSOR_FRONT_RIGHT];
        return;
    }
    header_out[0] = "SenleftFrontVal";
    header_out[1] = "SenleftFrontDiff";
    header_out[2] = "SenRightFrontVal";
    header_out[3] = "SenRightFrontDiff";
}
// モード09: 左右壁センサ値 + 壁なし変位（45°斜め）
void mode9(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = g_sensor[SENSOR_LEFT][0];
    d[1] = g_sensor[SENSOR_RIGHT][0];
    d[2] = NoWallDisplacementL45slant;
    d[3] = NoWallDisplacementR45slant;
        return;
    }
    header_out[0] = "SenleftVal";
    header_out[1] = "SenRightVal";
    header_out[2] = "NoWallDisL45slant";
    header_out[3] = "NoWallDisR45slant";
}

// モード10: 前左右センサ値 + 壁なし変位（45°斜め）
void mode10(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = g_sensor[SENSOR_FRONT_LEFT][0];
    d[1] = g_sensor[SENSOR_FRONT_RIGHT][0];
    d[2] = NoWallDisplacementL45slant;
    d[3] = NoWallDisplacementR45slant;
        return;
    }
    header_out[0] = "SenleftFrontVal";
    header_out[1] = "SenRightFrontVal";
    header_out[2] = "NoWallDisL45slant";
    header_out[3] = "NoWallDisR45slant";
}


// モード11: エンコーダ速度とカウント値比較（L/R）
void mode11(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = E_speedL;
    d[1] = encoder_L;
    d[2] = E_speedR;
    d[3] = encoder_R;
        return;
    }
    header_out[0] = "ActStraVelL";
    header_out[1] = "EncCountL";
    header_out[2] = "ActStraVelR";
    header_out[3] = "EncCountR";
}

// モード12: 回転系速度とモータ個別速度比較
void mode12(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = turning.velocity;
    d[1] = angle_speed;
    d[2] = g_V_L;
    d[3] = g_V_R;
        return;
    }
    header_out[0] = "TarTurnVel";
    header_out[1] = "ActTurnVel";
    header_out[2] = "ActMotVolL";
    header_out[3] = "ActMotVolR";
}

// モード13: 直進速度・エンコーダ速度・gf速度比較
void mode13(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = straight.velocity;
    d[1] = E_speedR;
    d[2] = E_speedL;
    d[3] = gf_speed;
        return;
    }
    header_out[0] = "TarStraVel";
    header_out[1] = "ActStraVelR";
    header_out[2] = "ActStraVelL";
    header_out[3] = "ActAccVel";
}

// モード14: 直進速度・Kalman・モータ速度比較
void mode14(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = straight.velocity;
    d[1] = kalman_speed;
    d[2] = g_V_L;
    d[3] = g_V_R;
        return;
    }
    header_out[0] = "TarStraVel";
    header_out[1] = "ActKalVel";
    header_out[2] = "ActMotVolL";
    header_out[3] = "ActMotVolR";
}

// モード15: 前左センサ + 壁切れ差分 + 壁なし変位＆回数
void mode15(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = g_sensor[SENSOR_FRONT_LEFT][0];
    d[1] = g_sensor_diff_wallcut[SENSOR_FRONT_LEFT];
    d[2] = NoWallDisplacementL45;
    d[3] = NoWallCountL45;
        return;
    }
    header_out[0] = "SenleftFrontVal";
    header_out[1] = "SenleftFrontDiff";
    header_out[2] = "NoWallDisL45";
    header_out[3] = "NoWallCntL45";
}

// モード16: 前右センサ + 壁切れ差分 + 壁なし変位2種
void mode16(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = g_sensor[SENSOR_FRONT_RIGHT][0];
    d[1] = g_sensor_diff_wallcut_slant[SENSOR_FRONT_RIGHT];
    d[2] = NoWallDisplacementR45slant;
    d[3] = NoWallDisplacementR45slant2;
        return;
    }
    header_out[0] = "SenRightFrontVal";
    header_out[1] = "SenRightFrontDiffSlant";
    header_out[2] = "NoWallDisR45slant";
    header_out[3] = "NoWallDisR45slant2";
}

// モード17: 前左センサ + 壁切れ差分 + 壁なし変位2種
void mode17(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = g_sensor[SENSOR_FRONT_LEFT][0];
    d[1] = g_sensor_diff_wallcut_slant[SENSOR_FRONT_LEFT];
    d[2] = NoWallDisplacementL45slant;
    d[3] = NoWallDisplacementL45slant2;
        return;
    }
    header_out[0] = "SenleftFrontVal";
    header_out[1] = "SenleftFrontDiffSlant";
    header_out[2] = "NoWallDisL45slant";
    header_out[3] = "NoWallDisL45slant2";
}

// モード18: 斜め左右センサ距離 + 壁なし変位
void mode18(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = g_sensor_distance_slant[SENSOR_LEFT][0];
    d[1] = g_sensor_distance_slant[SENSOR_RIGHT][0];
    d[2] = NoWallDisplacementL45slant;
    d[3] = NoWallDisplacementR45slant;
        return;
    }
    header_out[0] = "SenLeftSlantDis";
    header_out[1] = "SenRightSlantDis";
    header_out[2] = "NoWallDisL45slant";
    header_out[3] = "NoWallDisR45slant";
}

// モード19: 斜め前左右センサ距離 + 壁なし変位
void mode19(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = g_sensor_distance_slant[SENSOR_FRONT_LEFT][0];
    d[1] = g_sensor_distance_slant[SENSOR_FRONT_RIGHT][0];
    d[2] = NoWallDisplacementL45slant;
    d[3] = NoWallDisplacementR45slant;
        return;
    }
    header_out[0] = "SenLeftFrontSlantDis";
    header_out[1] = "SenRightFrontSlantDis";
    header_out[2] = "NoWallDisL45slant";
    header_out[3] = "NoWallDisR45slant";
}

// モード20: 斜め左右センサ距離 + 90度中心補正ログ
void mode20(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = g_sensor_distance_slant[SENSOR_LEFT][0];
    d[1] = g_sensor_distance_slant[SENSOR_RIGHT][0];
    d[2] = g_log_CenterSlantL90;
    d[3] = g_log_CenterSlantR90;
        return;
    }
    header_out[0] = "SenLeftSlantDis";
    header_out[1] = "SenRightSlantDis";
    header_out[2] = "CenterSlantL90";
    header_out[3] = "CenterSlantR90";
}

// モード21: 斜め前左右センサ距離 + 45度中心補正ログ
void mode21(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = g_sensor_distance_slant[SENSOR_FRONT_LEFT][0];
    d[1] = g_sensor_distance_slant[SENSOR_FRONT_RIGHT][0];
    d[2] = g_log_CenterSlantL45;
    d[3] = g_log_CenterSlantR45;
        return;
    }
    header_out[0] = "SenLeftFrontSlantDis";
    header_out[1] = "SenRightFrontSlantDis";
    header_out[2] = "CenterSlantL45";
    header_out[3] = "CenterSlantR45";
}


// モード22: Lターン時のセンサと融合距離
void mode22(float* d, const char* header_out[], int request) {
    if (request == 0) {
    float f = (fusion_distanceL + fusion_distanceR) / 2.0f / sqrtf(2.0f);
    d[0] = g_sensor[SENSOR_LEFT][0];
    d[1] = f;
    d[2] = g_sensor[SENSOR_FRONT_LEFT][0];
    d[3] = f;
        return;
    }
    header_out[0] = "SenLeftVal";
    header_out[1] = "ActFusDis45";
    header_out[2] = "SenLeftFrontVal";
    header_out[3] = "ActFusDis45";
}

// モード23: Rターン時のセンサと融合距離
void mode23(float* d, const char* header_out[], int request) {
    if (request == 0) {
    float f = (fusion_distanceL + fusion_distanceR) / 2.0f / sqrtf(2.0f);
    d[0] = g_sensor[SENSOR_RIGHT][0];
    d[1] = f;
    d[2] = g_sensor[SENSOR_FRONT_RIGHT][0];
    d[3] = f;
        return;
    }
    header_out[0] = "SenRightVal";
    header_out[1] = "ActFusDis45";
    header_out[2] = "SenRightFrontVal";
    header_out[3] = "ActFusDis45";
}

// モード24: 複数速度推定の比較
void mode24(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = (E_speedL + E_speedR) / 2.0f;
    d[1] = (fusion_speedL + fusion_speedR) / 2.0f;
    d[2] = gf_speed;
    d[3] = kalman_speed;
        return;
    }
    header_out[0] = "ActStraVel";
    header_out[1] = "ActFusVel";
    header_out[2] = "ActAccVel";
    header_out[3] = "ActKalVel";
}

// モード25: FF加速度と実加速度の比較
void mode25(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = turning.velocity;
    d[1] = angle_speed;
    d[2] = turning.acceleration / 50.0f;
    d[3] = (turning.velocity - record_buf) / INTERRUPT_TIME / 50.0f;
    record_buf = turning.velocity;
        return;
    }
    header_out[0] = "TarTurnVel";
    header_out[1] = "ActTurnVel";
    header_out[2] = "FFTurnAcc";
    header_out[3] = "ActTurnAcc";
}

// モード26: 旋回調整用ログ（回転・角速度・直進・融合速度）
void mode26(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = turning.velocity;
    d[1] = angle_speed;
    d[2] = straight.velocity;
    d[3] = (fusion_speedL + fusion_speedR) / 2.0f;
        return;
    }
    header_out[0] = "TarTurnVel";
    header_out[1] = "ActTurnVel";
    header_out[2] = "TarStraVel";
    header_out[3] = "ActFusVel";
}

// モード27: 前壁センサの記録（生値のみ）
void mode27(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = g_sensor[SENSOR_FRONT_L][0];
    d[1] = g_sensor[SENSOR_FRONT_R][0];
    d[2] = 0.0f;
    d[3] = 0.0f;
        return;
    }
    header_out[0] = "SenLeftFrontVal";
    header_out[1] = "SenRightFrontVal";
    header_out[2] = "Zero";
    header_out[3] = "Zero";
}

// モード28: 前壁センサ + 壁切れ判定変位（L/R 45度）
void mode28(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = g_sensor[SENSOR_FRONT_LEFT][0];
    d[1] = g_sensor[SENSOR_FRONT_RIGHT][0];
    d[2] = NoWallDisplacementL45;
    d[3] = NoWallDisplacementR45;
        return;
    }
    header_out[0] = "SenLeftFrontVal";
    header_out[1] = "SenRightFrontVal";
    header_out[2] = "NoWallDisL45";
    header_out[3] = "NoWallDisR45";    
}

// モード29: 壁切れ差分 + 斜め変位（L/R 45度 Slant2）
void mode29(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = g_sensor_diff_wallcut_slant[SENSOR_FRONT_LEFT];
    d[1] = g_sensor_diff_wallcut_slant[SENSOR_FRONT_RIGHT];
    d[2] = NoWallDisplacementL45slant2;
    d[3] = NoWallDisplacementR45slant2;
        return;
    }
    header_out[0] = "SenLeftFrontDiffSlant";
    header_out[1] = "SenRightFrontDiffSlant";
    header_out[2] = "NoWallDisL45slant2";
    header_out[3] = "NoWallDisR45slant2";
}

// モード30: 速度・角速度・ヨー角の変化（回転中の状態確認）
void mode30(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = (fusion_speedR + fusion_speedL) / 2.0f;
    d[1] = angle_speed;
    d[2] = yaw_angle;
    d[3] = 0.0f;
        return;
    }
    header_out[0] = "ActFusVel";
    header_out[1] = "ActTurnVel";
    header_out[2] = "YawAngle";
    header_out[3] = "Zero";
}

// モード31: 直進・旋回速度と旋回変位
void mode31(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = straight.velocity;
    d[1] = turning.velocity;
    d[2] = turning.displacement;
    d[3] = 0.0f;
        return;
    }
    header_out[0] = "TarStraVel";
    header_out[1] = "TarTurnVel";
    header_out[2] = "TarTurnDis";
    header_out[3] = "Zero";
}

// モード32: モータ左右速度と電圧情報
void mode32(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = g_V_L;
    d[1] = g_V_R;
    d[2] = g_Vol1;
    d[3] = g_Vol2;
        return;
    }
    header_out[0] = "ActMotVolL";
    header_out[1] = "ActMotVolR";
    header_out[2] = "ActVol1";
    header_out[3] = "ActVol2";
}

// モード33: 電圧と融合・直進速度の比較
void mode33(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = g_Vol1;
    d[1] = (fusion_speedR + fusion_speedL) / 2.0f;
    d[2] = straight.velocity;
    d[3] = 0.0f;
        return;
    }
    header_out[0] = "ActVol1";
    header_out[1] = "ActFusVel";
    header_out[2] = "TarStraVel";
    header_out[3] = "Zero";
}

// モード34: 各種速度と角速度の状態確認
void mode34(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = straight.velocity;
    d[1] = (fusion_speedR + fusion_speedL) / 2.0f;
    d[2] = turning.velocity;
    d[3] = angle_speed;
        return;
    }
    header_out[0] = "TarStraVel";
    header_out[1] = "ActFusVel";
    header_out[2] = "TarTurnVel";
    header_out[3] = "ActTurnVel";
}

// モード35: 回転速度と加速度・平均エンコーダ速度比較
void mode35(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = turning.velocity;
    d[1] = angle_speed;
    d[2] = gf_accel;
    d[3] = (E_speedL + E_speedR) / 2.0f;
        return;
    }
    header_out[0] = "TarTurnVel";
    header_out[1] = "ActTurnVel";
    header_out[2] = "ActAccAcc";
    header_out[3] = "ActStraVel";
}

// モード36: モータ推定出力（拡張FF項の確認）
void mode36(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = turning.velocity;
    d[1] = angle_speed;
    d[2] = g_V_R * 100.0f;
    d[3] = g_V_L * 100.0f;
        return;
    }
    header_out[0] = "TarTurnVel";
    header_out[1] = "ActTurnVel";
    header_out[2] = "ActMotVolR100";
    header_out[3] = "ActMotVolL100";
}

// モード37: simscape用電圧とエンコーダ、ジャイロ
void mode37(float* d, const char* header_out[], int request) {
    if (request == 0) {
    d[0] = kalman_speed;
    d[1] = angle_speed;
    d[2] = g_V_R;
    d[3] = g_V_L;
        return;
    }
    header_out[0] = "ActKalVel";
    header_out[1] = "ActTurnVel";
    header_out[2] = "ActMotVolR";
    header_out[3] = "ActMotVolL";
}


RecordMode record_modes[] = {
    { .record_func = mode1 },
    { .record_func = mode2 },
    { .record_func = mode3 },
    { .record_func = mode4 },
    { .record_func = mode5 },
    { .record_func = mode6 },
    { .record_func = mode7 },
    { .record_func = mode8 },
    { .record_func = mode9 },
    { .record_func = mode10 },
    { .record_func = mode11 },
    { .record_func = mode12 },
    { .record_func = mode13 },
    { .record_func = mode14 },
    { .record_func = mode15 },
    { .record_func = mode16 },
    { .record_func = mode17 },
    { .record_func = mode18 },
    { .record_func = mode19 },
    { .record_func = mode20 },
    { .record_func = mode21 },
    { .record_func = mode22 },
    { .record_func = mode23 },
    { .record_func = mode24 },
    { .record_func = mode25 },
    { .record_func = mode26 },
    { .record_func = mode27 },
    { .record_func = mode28 },
    { .record_func = mode29 },
    { .record_func = mode30 },
    { .record_func = mode31 },
    { .record_func = mode32 },
    { .record_func = mode33 },
    { .record_func = mode34 },
    { .record_func = mode35 },
    { .record_func = mode36 },
    { .record_func = mode37 },
};
int num_record_modes = sizeof(record_modes) / sizeof(RecordMode);


void record_reset(void) {
	record_mode = 0;
	record_time = 0;
	record_rupe_flag = 0;
	sample_time = 2;
	sample_count = 0;
	recordstop_count = 0;
}

void record_data(float *input_record_data, int numlen) {

	sample_count = sample_count % sample_time;

	if( sample_count == 0 ){
		for (int record_count = 0; record_count < numlen; record_count++) {
			record_value[record_count][record_time] =
					input_record_data[record_count];
		}
		if (record_rupe_flag == 1) {
			record_end_point = record_time;
		}
		record_time++;
		if (record_time >= MAX_RECORD_TIME) {
			record_time = 0;
			record_rupe_flag = 1;
		}
		recordstop_count++;

	}
	sample_count++;

}

void record_print(void) {
	int a, time_index;
    const char* header[MAX_RECORD_NUM];
    if(head_record_mode > 0 && head_record_mode <= num_record_modes){ 
    record_modes[head_record_mode - 1].record_func(NULL, header, 1);
    }

	// ヘッダー行
    printf("Time[s]");
    for (int i = 0; i < MAX_RECORD_NUM; i++) {
        printf(",%s", header[i]);
    }
    printf("\n");


	if (record_rupe_flag == 0) {
		for (a = 0; a <= record_time - 1; a++) {

			printf("%f", (float)(a*sample_time)*INTERRUPT_TIME);
			for (int record_count = 0; record_count < MAX_RECORD_NUM;
					record_count++) {
				printf(",%f", record_value[record_count][a]);
			}
			printf("\n");
		}
	} else {
		for (a = 0; a <= MAX_RECORD_TIME - 1; a++) {
			time_index = record_end_point + 1 + a;
			if (time_index >= MAX_RECORD_TIME) {
				time_index -= MAX_RECORD_TIME;
			}
			printf("%f", (float)(a*sample_time)*INTERRUPT_TIME);
			for (int record_count = 0; record_count < MAX_RECORD_NUM;
					record_count++) {
				printf(",%f", record_value[record_count][time_index]);
			}
			printf("\n");
		}
	}

}

void interrupt_record(void) {
	float r_data[MAX_RECORD_NUM];

	if (record_mode == 0) {
		return;
	}else{
        if( record_mode != RECORD_STOPMODE ){
		head_record_mode = record_mode;
        }
	}

	if (record_mode == RECORD_STOPMODE && recordstop_count == 0 ) {
		r_data[0] = RECORD_STOPNUM;
		r_data[1] = RECORD_STOPNUM;
		r_data[2] = RECORD_STOPNUM;
		r_data[3] = RECORD_STOPNUM;
		record_data(r_data, 4);	
		return;		
	}

	if(record_mode <= num_record_modes){
    	record_modes[record_mode - 1].record_func(r_data, NULL, 0);
    	record_data(r_data, MAX_RECORD_NUM);
	}

	if (record_mode != RECORD_STOPMODE){
		recordstop_count=0;
	}


}

