/*
 * maze_strategy.c
 *
 *  Created on: 2023/06/29
 *      Author: sf199
 */

#include"maze_strategy.h"

#include <stdio.h>
#include <math.h>

#include"define.h"

#include "PL_motor.h"
#include "PL_LED.h"
#include "PL_timer.h"
#include "PL_flash.h"

#include "CL_EnoderGyro.h"
#include "CL_sensor.h"
#include "Control_motor.h"
#include "PID_EncoderGyro.h"
#include "PID_wall.h"

#include "maze_Turning.h"
#include"maze_wall.h"
#include "fail_safe.h"
#include "record.h"



unsigned short pass_count;

char maze_mode;
int kitikukan;

int pass[PASS_NUM]; //1f 2r 3l


uint8_t flg_backhit=OFF;


/**
 * @brief センサ値から前方・右・左の壁の有無を判定する
 * 
 * @param[out] front_wall 前方に壁がある場合 true、ない場合 false
 * @param[out] right_wall 右側に壁がある場合 true、ない場合 false
 * @param[out] left_wall 左側に壁がある場合 true、ない場合 false
 * 
 * @details
 * 左右の前方センサ（SENSOR_FRONT_L, SENSOR_FRONT_R）の平均値が
 * F_PRESENCE 閾値の2倍以上であれば前方に壁があると判定する。
 * 右・左の壁はそれぞれ R_PRESENCE, L_PRESENCE を閾値として判定。
 * 
 * 使用するセンサ値は g_sensor_mean[] に格納されている平均化済みの値。
 */
void get_wallData_sensor(_Bool* front_wall,_Bool* right_wall,_Bool* left_wall){

	*front_wall = ((g_sensor_mean[SENSOR_FRONT_L] + g_sensor_mean[SENSOR_FRONT_R]) >= 2 * F_PRESENCE);
	*right_wall = (g_sensor_mean[SENSOR_RIGHT] >= R_PRESENCE);
	*left_wall  = (g_sensor_mean[SENSOR_LEFT]  >= L_PRESENCE);

}

/**
 * @brief 指定された方向に応じて座標 (x, y) を更新する
 * 
 * @param[in,out] x X座標へのポインタ。東西方向の移動に応じて加減算される
 * @param[in,out] y Y座標へのポインタ。南北方向の移動に応じて加減算される
 * @param[in] direction 移動方向を示す数値（1:北, 2:東, 3:南, 4:西）
 * 
 * @details
 * マイクロマウスの進行方向に応じて座標を1マス分更新する。
 * - 北（1）：Y座標を +1
 * - 東（2）：X座標を +1
 * - 南（3）：Y座標を -1
 * - 西（4）：X座標を -1
 * 
 * 方向は整数値で表現され、switch文で分岐処理される。
 */
void update_coordinate(int *x,int *y,int direction){

	switch (direction) {
	case 1://北
		*y += 1;
		break;
	case 2://東
		*x += 1;
		break;
	case 3://南
		*y -= 1;
		break;
	case 4://西
		*x -= 1;
		break;
	}

}


/**
 * @brief 探索中に停止せず連続して移動するための方向判断・制御を行う
 * 
 * @param[in,out] direction 現在の進行方向（1:北, 2:東, 3:南, 4:西）。旋回時に更新される
 * @param[in] front_count 足立法による前方の歩数マップ値（小さいほどゴールに近い）
 * @param[in] right_count 足立法による右方向の歩数マップ値
 * @param[in] back_count 足立法による後方向の歩数マップ値
 * @param[in] left_count 足立法による左方向の歩数マップ値
 * @param[in] input_StraightVelocity 直進時の目標速度
 * @param[in] input_TurningVelocity 旋回時の目標速度
 * @param[in] input_StraightAcceleration 直進時の加速度
 * @param[in] input_TurningAcceleration 旋回時の加速度
 * @param[in] howspeed 旋回速度パラメータ構造体（slalom_R, slalom_Lなどを含む）
 * @param[in] front_wall 前方に壁があるかどうか（true:あり）
 * @param[in] right_wall 右側に壁があるかどうか
 * @param[in] left_wall 左側に壁があるかどうか
 * 
 * @details
 * 停止せずに連続して移動する探索モード。
 * 各方向の歩数マップの値が最も小さい方向がゴールに近い方向として選択される。
 * 
 * 優先順位は「前→右→左→後」の順。
 * - 前方が最も近い場合：直進（WallControl有効）
 * - 右が最も近い場合：右旋回（slalomR）
 * - 左が最も近い場合：左旋回（slalomL）
 * - 後方が最も近い場合：180度旋回（壁補正・ジャイロリセット含む）
 */
void run_movement_continuity(int *direction,unsigned short front_count,unsigned short right_count,
		unsigned short back_count,unsigned short left_count,float input_StraightVelocity,
		float input_TurningVelocity, float input_StraightAcceleration,
		float input_TurningAcceleration, parameter_speed howspeed,_Bool front_wall,_Bool right_wall,_Bool left_wall){
	MOTOR_MODE mode;
	// 移動の優先順位 ： 前→右→左→後
	if (front_count <= right_count && front_count <= left_count && front_count <= back_count){
		// 直進
		mode.WallControlMode=1;
		mode.calMazeMode=0;
		mode.WallCutMode=0;
		straight_table2(MAZE_SECTION-MAZE_OFFSET, input_StraightVelocity,input_StraightVelocity,input_StraightVelocity,input_StraightAcceleration, mode);
	}
	if(right_count < front_count && right_count <= left_count && right_count <= back_count){
		// 右旋回
		slalomR(howspeed.slalom_R, OFF,EXPLORATION,OFF,input_StraightVelocity,ON);
		*direction += 1;
	}
	if(left_count < front_count && left_count < right_count && left_count <= back_count){
		// 左旋回
		slalomL(howspeed.slalom_L, OFF,EXPLORATION,OFF,input_StraightVelocity,ON);
		*direction -= 1;
	}
	if(back_count < front_count && back_count < right_count
			&& back_count < left_count){
		//180度旋回(前壁がある場合は尻当てを行うことで位置修正)
		mode.WallControlMode=1;
		mode.calMazeMode=0;
		mode.WallCutMode=0;
		straight_table2(MAZE_SECTION / 2 - MAZE_OFFSET + (BACK_TO_CENTER - BACK_TO_CENTER_FRONT), input_StraightVelocity, 0, input_StraightVelocity, input_StraightAcceleration, mode);

		create_DijkstraMap3();
		backTurn_controlWall(input_TurningVelocity, input_TurningAcceleration, front_wall, left_wall, right_wall);
		//backTurn_hitWall(input_TurningVelocity, input_TurningAcceleration, front_wall, left_wall, right_wall);
		//wait_ms_NoReset(200);
		mode.WallControlMode=0;
		if(front_wall && flg_backhit == ON){
		straight_table2(-BACK_TO_CENTER-10, 0,0,-150,1000, mode);
		wait_ms_NoReset(400);
		clear_Ierror();
		reset_gyro_integral();
		reset_speed();
		mode.WallControlMode=1;
		straight_table2(BACK_TO_CENTER_FRONT +MAZE_SECTION/2,0,input_StraightVelocity,input_StraightVelocity,input_StraightAcceleration, mode);
		}else{
			mode.WallControlMode=1;
			straight_table2(MAZE_SECTION/2-(BACK_TO_CENTER-BACK_TO_CENTER_FRONT),0,input_StraightVelocity,input_StraightVelocity,input_StraightAcceleration, mode);
		}
		*direction = *direction + 2;
	}

}



/**
 * @brief 探索中に一時停止を挟みながら移動するための方向判断・制御を行う
 * 
 * @param[in,out] direction 現在の進行方向（1:北, 2:東, 3:南, 4:西）。旋回時に更新される
 * @param[in] front_count 足立法による前方の歩数マップ値（小さいほどゴールに近い）
 * @param[in] right_count 足立法による右方向の歩数マップ値
 * @param[in] back_count 足立法による後方向の歩数マップ値
 * @param[in] left_count 足立法による左方向の歩数マップ値
 * @param[in] input_StraightVelocity 直進時の目標速度
 * @param[in] input_TurningVelocity 旋回時の目標速度
 * @param[in] input_StraightAcceleration 直進時の加速度
 * @param[in] input_TurningAcceleration 旋回時の加速度
 * @param[in] howspeed 旋回速度パラメータ構造体（slalom_R, slalom_Lなどを含む）
 * @param[in] front_wall 前方に壁があるかどうか（true:あり）
 * @param[in] right_wall 右側に壁があるかどうか
 * @param[in] left_wall 左側に壁があるかどうか
 * @param[in] x 現在のX座標
 * @param[in] y 現在のY座標
 * @param[in] MazeRecord_mode フラッシュ記録モード（1で記録）
 * @param[in] Dijkstra_mode ダイクストラ探索モード（ダイクストラ法での最短経路を探索する）
 * 
 * @details
 * 一時停止を挟みながら移動する探索モード。
 * 各方向の `_count` 値は足立法による歩数マップの値であり、
 * その方向に進んだ場合のゴールまでの推定歩数を示す。
 * 最も小さい方向が「最も有望な進行方向」として選択される。
 * 
 * 停止後に以下の処理を実行：
 * - 地図記録（MazeRecord_mode）
 * - 最短経路探索（Dijkstra_mode）
 * - 周囲の歩数マップ再評価
 * 
 * 優先順位は「前→右→左→後」の順。
 * - 前方が最も近い場合：直進
 * - 右が最も近い場合：右旋回（mollifier_turning_table）
 * - 左が最も近い場合：左旋回（mollifier_turning_table）
 * - 後方が最も近い場合：180度旋回（壁補正・ジャイロリセット含む）
 * 
 * 迷路破損時には探索を停止し、LED表示とモータ停止を実行する。
 */
void run_movement_suspension(int *direction, unsigned short front_count,
		unsigned short right_count, unsigned short back_count,
		unsigned short left_count, float input_StraightVelocity,
		float input_TurningVelocity, float input_StraightAcceleration,
		float input_TurningAcceleration, parameter_speed howspeed,
		_Bool front_wall, _Bool right_wall, _Bool left_wall,int x,int y,uint8_t MazeRecord_mode,uint8_t Dijkstra_mode) {
	MOTOR_MODE mode;
	// 移動の優先順位 ： 前→右→左→後
	mode.WallControlMode = 1;
	mode.calMazeMode = 0;
	mode.WallCutMode = 0;
	straight_table2(MAZE_SECTION / 2 - MAZE_OFFSET + (BACK_TO_CENTER - BACK_TO_CENTER_FRONT), input_StraightVelocity, 0, input_StraightVelocity, input_StraightAcceleration, mode);

	if(MazeRecord_mode==1){
		if(error_mode==0){
		flash_in();
		}
		maze_mode = 1;
	}
	// clear_Ierror();
	// reset_gyro_integral();
	//reset_speed();
	//enc.sigma_error = 0;

	if(Dijkstra_mode==1){
		create_DijkstraMap3();
		route_Dijkstra(); //ダイクストラ法の結果から最短ルートをスタックに入れる
		create_StepCountMap_unknown();
		search_AroundWalkCount(&front_count, &right_count, &back_count, &left_count, x, y, *direction);
		if (front_wall) {front_count = MAX_WALKCOUNT;}
		if (right_wall) {right_count = MAX_WALKCOUNT;}
		if (left_wall) {left_count = MAX_WALKCOUNT;}
		if (front_count == MAX_WALKCOUNT && right_count == MAX_WALKCOUNT && left_count == MAX_WALKCOUNT && back_count == MAX_WALKCOUNT) {
			// 迷路破損のため停止(一時停止後に周辺の地図情報を初期化して再探索に変更予定)
			error_mode = 21;
			g_WallControl_mode = 0;
			pl_yellow_LED_count(error_mode);
			pl_DriveMotor_stop();
			pl_DriveMotor_standby(OFF);
			//break;
		}

	}

if(error_mode==0){
	mode.WallControlMode = 0;
	mode.WallCutMode = 0;
	mode.calMazeMode = 0;
	if (front_count <= right_count && front_count <= left_count && front_count <= back_count) {
		// 直進
		straight_table2(MAZE_SECTION / 2 - (BACK_TO_CENTER - BACK_TO_CENTER_FRONT), 0, input_StraightVelocity, input_StraightVelocity, input_StraightAcceleration, mode);
	}
	if (right_count < front_count && right_count <= left_count && right_count <= back_count) {
		// 右旋回
		//turning_table2(-90, 0, 0, -input_TurningVelocity, input_TurningAcceleration);
		wait_ms_NoReset(50);
		mollifier_turning_table(-90,input_TurningVelocity);
		wait_ms_NoReset(50);
		straight_table2(MAZE_SECTION / 2 - (BACK_TO_CENTER - BACK_TO_CENTER_FRONT), 0, input_StraightVelocity, input_StraightVelocity, input_StraightAcceleration, mode);
		*direction += 1;
	}
	if (left_count < front_count && left_count < right_count && left_count <= back_count) {
		// 左旋回
		//turning_table2(90, 0, 0, input_TurningVelocity, input_TurningAcceleration);
		wait_ms_NoReset(50);
		mollifier_turning_table(90,input_TurningVelocity);
		wait_ms_NoReset(50);
		straight_table2(MAZE_SECTION / 2 - (BACK_TO_CENTER - BACK_TO_CENTER_FRONT), 0, input_StraightVelocity, input_StraightVelocity, input_StraightAcceleration, mode);
		*direction -= 1;
	}
	if(back_count < front_count && back_count < right_count
			&& back_count < left_count){
		//180度旋回(前壁がある場合は尻当てを行うことで位置修正)
		backTurn_controlWall(input_TurningVelocity, input_TurningAcceleration, front_wall, left_wall, right_wall);
		//backTurn_hitWall(input_TurningVelocity, input_TurningAcceleration, front_wall, left_wall, right_wall);
		//wait_ms_NoReset(100);
		mode.WallControlMode=0;
		if(front_wall && flg_backhit == ON){
		straight_table2(-BACK_TO_CENTER-10, 0,0,-150,1000, mode);
		wait_ms_NoReset(400);
		clear_Ierror();
		reset_gyro_integral();
		reset_speed();
		mode.WallControlMode=1;
		straight_table2(BACK_TO_CENTER +MAZE_SECTION/2,0,input_StraightVelocity,input_StraightVelocity,input_StraightAcceleration, mode);
		}else{
			//clear_Ierror();
			mode.WallControlMode=0;
			straight_table2(MAZE_SECTION/2-(BACK_TO_CENTER-BACK_TO_CENTER_FRONT),0,input_StraightVelocity,input_StraightVelocity,input_StraightAcceleration, mode);
		}
		*direction = *direction + 2;
	}
}

}



/**
 * @brief マイクロマウスによる迷路探索のメイン処理を実行する
 * 
 * @param[in] input_StraightVelocity 直進時の目標速度
 * @param[in] input_TurningVelocity 旋回時の目標速度
 * @param[in] input_StraightAcceleration 直進時の加速度
 * @param[in] input_TurningAcceleration 旋回時の加速度
 * @param[in] howspeed 旋回速度パラメータ構造体（slalom_R, slalom_Lなどを含む）
 * @param[in] know_mode 既知区間加速モード（0:既知区間等速, 1:既知区間加速）
 * @param[in] Dijkstra_mode ダイクストラ探索モード（帰りにダイクストラ法での最短経路を探索する）
 * 
 * @details
 * この関数は迷路探索の全体制御を行う。
 * 初期化 → センサ取得 → 壁更新 → 歩数マップ生成 → 方向判断 → 移動制御
 * をループしながら、ゴール到達または異常検出まで探索を継続する。
 * 
 * - 足立法による歩数マップ（StepCountMap）を用いて、各方向の歩数を評価
 * - 最も歩数が少ない方向を選択し、`run_movement_continuity` または `run_movement_suspension` を呼び出す
 * - `kitikukan` フラグにより既知区間の圧縮移動を実行
 * - ゴール到達時には `GOAL_ALL` フラグを確認し、静止動作で最終調整
 * - `error_mode` による異常検出（自己位置破損、迷路破損、時間制限など）を含む
 * - 探索終了後、壁情報をフラッシュに記録（正常終了時は `flash_in()`、異常時は `flash_out()`）
 * 
 * 使用される主な補助関数：
 * - `get_wallData_sensor()`：壁センサ取得
 * - `update_wall()`：壁情報更新
 * - `create_StepCountMap_queue()`：足立法マップ生成
 * - `search_AroundWalkCount()`：周囲の歩数取得
 * - `decision_kitiku()`：既知区間判定
 * - `compress_kitiku()`：既知区間圧縮移動
 * - `run_movement_continuity()`：連続移動制御
 * - `run_movement_suspension()`：静止移動制御
 * 
 * @note
 * `x`, `y`, `direction` は本来構造体化すべきだが、現状は個別変数で管理。
 * `MAX_WALKCOUNT` は到達不能領域を示す特殊値。
 * `GOAL_ALL` フラグはゴール到達判定に使用。
 */
void AdatiWayReturn(float input_StraightVelocity, float input_TurningVelocity, float input_StraightAcceleration,
		float input_TurningAcceleration, parameter_speed howspeed,int know_mode,uint8_t Dijkstra_mode) {



	//初期化
	maze_mode = 1; //迷路探索開始フラグ
	unsigned short front_count, right_count, back_count, left_count;
	int x=0;//構造体にしたい
	int y=0;
	int direction=1;
	_Bool front_wall,right_wall,left_wall;
	char timer_end_mode=0;
	int kitiku_distance;
	MOTOR_MODE mode;
	mode.WallControlMode=1;
	mode.WallControlStatus=0;
	mode.WallCutMode=0;
	mode.calMazeMode=0;
	highspeed_mode = 0;

	//モータenable
	pl_DriveMotor_standby(ON);
	//wait_ms_NoReset(500);
	reset_gyro();
	reset_speed();
	reset_distance();
	clear_Ierror();
	reset_Kalman();
	
	//初期位置のセンサー確認
	get_wallData_sensor(&front_wall,&right_wall,&left_wall);
	//初期位置での壁更新
	update_wall(x,y,direction,front_wall,right_wall,left_wall);
	//初期位置での迷路展開
	create_StepCountMap_queue();
	straight_table2(MAZE_SECTION/2+BACK_TO_CENTER_FRONT,0,input_StraightVelocity,input_StraightVelocity,input_StraightAcceleration, mode);

	while (1) {

		update_coordinate(&x,&y,direction);

		get_wallData_sensor(&front_wall,&right_wall,&left_wall);

		mode.WallControlMode=1;
		mode.calMazeMode=1;
		mode.WallCutMode=0;
		straight_table2(MAZE_OFFSET, input_StraightVelocity,input_StraightVelocity,input_StraightVelocity,input_StraightAcceleration, mode);
		//走行中計算
		update_wall(x,y,direction,front_wall,right_wall,left_wall);
		create_StepCountMap_queue();
		search_AroundWalkCount(&front_count,&right_count,&back_count,&left_count,x,y,direction);
		if (front_wall) {front_count = MAX_WALKCOUNT;}
		if (right_wall) {right_count = MAX_WALKCOUNT;}
		if (left_wall) {left_count = MAX_WALKCOUNT;}
		decision_kitiku(x,y,direction,front_count,right_count,back_count,left_count);

		mode.WallCutMode=1;
		End_straight(MAZE_OFFSET, mode,right_wall,left_wall);


		//異常終了
		if (x == 0 && y == 0) {
			error_mode=16;
			pl_yellow_LED_count(error_mode);
			pl_DriveMotor_stop();
			pl_DriveMotor_standby(OFF);
			break;
		}
		if (front_count==MAX_WALKCOUNT && right_count==MAX_WALKCOUNT && left_count==MAX_WALKCOUNT && back_count==MAX_WALKCOUNT){
			// 迷路破損のため停止(一時停止後に周辺の地図情報を初期化して再探索に変更予定)
			error_mode=17;
			pl_yellow_LED_count(error_mode);
			pl_DriveMotor_stop();
			pl_DriveMotor_standby(OFF);
			break;
		}
		//　時間制限
		if (g_timCount_sec>MAZE_TIMER*60){
			timer_end_mode=1;
			pl_DriveMotor_stop();
			pl_DriveMotor_standby(OFF);
			break;
		}
		//正常終了
		if(GOAL_ALL){
			run_movement_suspension(&direction,front_count,right_count,back_count,left_count,
					input_StraightVelocity, input_TurningVelocity, input_StraightAcceleration, input_TurningAcceleration, howspeed,
					front_wall, right_wall, left_wall, x, y, 1, 1);
			if (direction >= 5) {direction = direction-4;}
			if (direction <= 0) {direction = direction+4;}
			break;
		}

		if(know_mode==0){kitikukan = 0;}
		if (kitikukan == OFF) {

			run_movement_continuity(&direction,front_count,right_count,back_count,left_count,
					input_StraightVelocity, input_TurningVelocity, input_StraightAcceleration, input_TurningAcceleration, howspeed,
					front_wall, right_wall, left_wall);

		} else {
			mode.WallControlMode=1;
			mode.calMazeMode=1;
			mode.WallCutMode=0;
			straight_table2(MAZE_SECTION-MAZE_OFFSET, input_StraightVelocity,input_StraightVelocity,input_StraightVelocity,input_StraightAcceleration, mode);
			compress_kitiku(&x,&y,&direction,&kitiku_distance);
			End_straight(MAZE_SECTION-MAZE_OFFSET,mode,1,1);
			mode.calMazeMode=0;
			straight_table2((MAZE_SECTION/2 * kitiku_distance),input_StraightVelocity,input_StraightVelocity,600,input_StraightAcceleration, mode);
		}

		if (direction >= 5) {direction = direction-4;}
		if (direction <= 0) {direction = direction+4;}

		if(error_mode>=1){
			break;
		}

	}





	while (1) {
		update_coordinate(&x,&y,direction);

		get_wallData_sensor(&front_wall,&right_wall,&left_wall);

		if(GOAL_ALL){
			noGoalPillarMode=1;
		}else{
			noGoalPillarMode=0;
		}

		mode.WallControlMode=1;
		mode.calMazeMode=1;
		mode.WallCutMode=0;
		straight_table2(MAZE_OFFSET, input_StraightVelocity,input_StraightVelocity,input_StraightVelocity,input_StraightAcceleration, mode);
		update_wall(x,y,direction,front_wall,right_wall,left_wall);
		if(Dijkstra_mode==1){
			route_Dijkstra();//ダイクストラ法の結果から最短ルートをスタックに入れる
			create_StepCountMap_unknown();
		}else{
			create_StepCountMapBack_queue();
		}
		search_AroundWalkCount(&front_count,&right_count,&back_count,&left_count,x,y,direction);
		if (front_wall) {front_count = MAX_WALKCOUNT;}
		if (right_wall) {right_count = MAX_WALKCOUNT;}
		if (left_wall) {left_count = MAX_WALKCOUNT;}
		decision_kitiku(x,y,direction,front_count,right_count,back_count,left_count);
		mode.WallCutMode=1;
		End_straight(MAZE_OFFSET,mode,right_wall,left_wall);

		//異常終了
        if(back_count < front_count && back_count < right_count
			&& back_count < left_count){
            Dijkstra_maker_flag=1;
        }
		if (front_count==MAX_WALKCOUNT && right_count==MAX_WALKCOUNT && left_count==MAX_WALKCOUNT && back_count==MAX_WALKCOUNT){
			// 迷路破損のため、ダイクストラ法更新
			Dijkstra_maker_flag=1;
		}
		if (x<0 || y<0 || x>MAZE_SQUARE_NUM-1 || y>MAZE_SQUARE_NUM-1){
			// 自己位置の破損
			error_mode=18;
			g_WallControl_mode=0;
			pl_yellow_LED_count(error_mode);
			pl_DriveMotor_stop();
			pl_DriveMotor_standby(OFF);
			break;
		}
		if (g_timCount_sec>MAZE_TIMER*60){
			// 秒数エンド
			timer_end_mode=1;
			pl_DriveMotor_stop();
			pl_DriveMotor_standby(OFF);
			break;
		}
		//正常終了
		if(x == 0 && y == 0) {
			mode.WallControlMode=0;
			mode.calMazeMode=0;
			mode.WallCutMode=0;
			straight_table2(MAZE_SECTION/2-MAZE_OFFSET+(BACK_TO_CENTER - BACK_TO_CENTER_FRONT), input_StraightVelocity,0,input_StraightVelocity,input_StraightAcceleration, mode);
			turning_table2(180,0,0,input_TurningVelocity,input_TurningAcceleration);
			break;
		}


		if(Dijkstra_maker_flag==1){
			run_movement_suspension(&direction,front_count,right_count,back_count,left_count,
					input_StraightVelocity, input_TurningVelocity, input_StraightAcceleration, input_TurningAcceleration, howspeed,
					front_wall, right_wall, left_wall, x, y, 0, 1);
		}else{

			if(know_mode==0){kitikukan = 0;}
			if (kitikukan == OFF) {
				run_movement_continuity(&direction,front_count,right_count,back_count,left_count,
						input_StraightVelocity, input_TurningVelocity, input_StraightAcceleration, input_TurningAcceleration, howspeed,
						front_wall, right_wall, left_wall);
			} else {
				mode.WallControlMode=1;
				mode.calMazeMode=1;
				mode.WallCutMode=0;
				straight_table2(MAZE_SECTION-MAZE_OFFSET, input_StraightVelocity,input_StraightVelocity,input_StraightVelocity,input_StraightAcceleration, mode);
				compress_kitiku(&x,&y,&direction,&kitiku_distance);
				End_straight(MAZE_SECTION-MAZE_OFFSET,mode,1,1);
				mode.calMazeMode=0;
				straight_table2((MAZE_SECTION/2 * kitiku_distance),input_StraightVelocity,input_StraightVelocity,600,input_StraightAcceleration, mode);
			}

		}

		if (direction >= 5) {direction = direction-4;}
		if (direction <= 0) {direction = direction+4;}

		if(error_mode>=1){break;}

		}



	pl_DriveMotor_standby(OFF); //MTU2.TSTR.BIT.CST0 = 0;
	maze_mode = 0;
	wait_ms_NoReset(100);
	//maze_display(&wall);
	create_StepCountMap_queue();
	if(walk_count[0][0] == MAX_WALKCOUNT){
		error_mode = 19;
		pl_yellow_LED_count(error_mode);
	}
	if (error_mode == 0) {
		flash_in();
	} else if(timer_end_mode==0) {
		int t = 0;
		while (t <= MAZE_SQUARE_NUM-2) {
			error_wall.row[t] = wall.row[t];
			error_wall.column[t] = wall.column[t];
			t++;
		}
		t = 0;
		while (t <= MAZE_SQUARE_NUM-2) {
			error_wall.row_look[t] = wall.row_look[t];
			error_wall.column_look[t] = wall.column_look[t];
			t++;
		}

		t = 0;
		flash_out();
	}else{
		flash_in();
	}

}



/**
 * @brief 足立法による歩数マップを用いて最短走行用のパスを生成する
 * 
 * @details
 * 探索済みの迷路情報と足立法の歩数マップ（StepCountMap）をもとに、
 * ゴールまでの最短経路を `pass[]` 配列に記録する。
 * 
 * - 初期位置 (x=0, y=0, direction=1) からスタート
 * - 各方向の歩数を比較し、最も少ない方向へ進む
 * - 壁がある方向は MAX_WALKCOUNT に置き換え、選択対象から除外
 * - 移動内容は以下の形式で `pass[]` に記録：
 *   - `+2`：直進（2区画分）
 *   - `-2`：右旋回
 *   - `-3`：左旋回
 *   - `+1`：ゴール調整（直進1区画）
 * 
 * ゴール到達時には方向補正を行い、探索を終了する。
 * 
 * @note
 * `pass[]` は走行命令列として使用され、最短走行時に再生される。
 * `pass_count` は命令数のインデックス。
 */
void pass_maker(void){
	unsigned short front_count, right_count, back_count, left_count;

	_Bool front_wall;
	_Bool right_wall;
	_Bool left_wall;

	int x = 0;
	int y = 0;
	int direction = 1;
	pass_count = 0;
	create_StepCountMap_queue();
	maze_display(&wall);
	pass[0] = 1;
	while (1) {
		update_coordinate(&x,&y,direction);

		if(GOAL_ALL){

			if (pass[pass_count] >= 0) {
					} else {
						pass_count++;
					}
					pass[pass_count] = pass[pass_count] + 1;

					direction = direction + 2;
					if (direction == 5) {
						direction = 1;
					}
					if (direction == 6) {
						direction = 2;
					}
					if (direction == 0) {
						direction = 4;
					}
					if (direction == -1) {
						direction = 3;
					}
					break;

		}

		get_wall(x,y,direction,&front_wall,&right_wall,&left_wall);
		search_AroundWalkCount(&front_count,&right_count,&back_count,&left_count,x,y,direction);
		if (front_wall) {front_count = MAX_WALKCOUNT;}
		if (right_wall) {right_count = MAX_WALKCOUNT;}
		if (left_wall) {left_count = MAX_WALKCOUNT;}

		if (front_count==MAX_WALKCOUNT && right_count==MAX_WALKCOUNT && left_count==MAX_WALKCOUNT && back_count==MAX_WALKCOUNT){
			// 迷路破損のため停止(一時停止後に周辺の地図情報を初期化して再探索に変更予定)
			break;
		}
		if (front_count <= right_count && front_count <= left_count && front_count <= back_count){
			// 直進
			if (pass[pass_count] >= 0) {} else {pass_count++;}
			pass[pass_count] = pass[pass_count] + 2;
		}
		if(right_count < front_count && right_count <= left_count && right_count <= back_count){
			// 右旋回
			pass_count++;
			pass[pass_count] = -2;
			direction++;
		}
		if(left_count < front_count && left_count < right_count && left_count <= back_count){
			// 左旋回
			pass_count++;
			pass[pass_count] = -3;
			direction--;
		}

		if (direction == 5) {
			direction = 1;
		}
		if (direction == 6) {
			direction = 2;
		}
		if (direction == 0) {
			direction = 4;
		}
		if (direction == -1) {
			direction = 3;
		}

	}
}




/**
 * @brief ダイクストラ法による歩数マップを用いて最短走行用のパスを生成する
 * 
 * @details
 * ゴールまでの最短経路を、ダイクストラ法で生成した歩数マップ（DijkstraMap）をもとに
 * `pass[]` 配列へ記録する。
 * 
 * - 初期位置 (x=0, y=0, direction=1) からスタート
 * - 各方向の歩数を比較し、最も少ない方向へ進む
 * - 壁がある方向は MAX_WALKCOUNT_DIJKSTRA に置き換え、選択対象から除外
 * - 移動内容は以下の形式で `pass[]` に記録：
 *   - `+2`：直進（2区画分）
 *   - `-2`：右旋回
 *   - `-3`：左旋回
 * 
 * ゴール到達後は、姿勢に応じて貫通するようなゴールを実施するための補正を行う。
 * - `goal_mode = 0`：直進ゴール
 * - `goal_mode = 1`：左旋回ゴール
 * - `goal_mode = 2`：右旋回ゴール
 * 
 * ゴール補正後、`+1` を追加してパスを完了させる。
 * 
 * @note
 * `pass[]` は最短走行命令列として使用される。
 * `pass_count` は命令数のインデックス。
 */
void pass_maker_Dijkstra(void){

	unsigned short front_count, right_count, back_count, left_count;

	_Bool front_wall;
	_Bool right_wall;
	_Bool left_wall;

	int x = 0;
	int y = 0;
	int direction = 1;
	pass_count = 0;
	create_DijkstraMap3();
	//maze_display_Dijkstra();
	pass[0] = 1;
	while (1) {

		update_coordinate(&x,&y,direction);

		if(GOAL_ALL){
					break;

		}

		get_wall(x,y,direction,&front_wall,&right_wall,&left_wall);
		search_AroundDijkstraCount(&front_count,&right_count,&back_count,&left_count,x,y,direction);
		if (front_wall) {front_count = MAX_WALKCOUNT_DIJKSTRA;}
		if (right_wall) {right_count = MAX_WALKCOUNT_DIJKSTRA;}
		if (left_wall) {left_count = MAX_WALKCOUNT_DIJKSTRA;}

		if (front_count==MAX_WALKCOUNT_DIJKSTRA && right_count==MAX_WALKCOUNT_DIJKSTRA && left_count==MAX_WALKCOUNT_DIJKSTRA && back_count==MAX_WALKCOUNT_DIJKSTRA){
			// 迷路破損のため停止(一時停止後に周辺の地図情報を初期化して再探索に変更予定)

			break;
		}
		if (front_count <= right_count && front_count <= left_count && front_count <= back_count){
			// 直進
			if (pass[pass_count] >= 0) {} else {pass_count++;}
			pass[pass_count] = pass[pass_count] + 2;
		}
		if(right_count < front_count && right_count <= left_count && right_count <= back_count){
			// 右旋回
			pass_count++;
			pass[pass_count] = -2;
			direction++;
		}
		if(left_count < front_count && left_count < right_count && left_count <= back_count){
			// 左旋回
			pass_count++;
			pass[pass_count] = -3;
			direction--;
		}

		if (direction == 5) {
			direction = 1;
		}
		if (direction == 6) {
			direction = 2;
		}
		if (direction == 0) {
			direction = 4;
		}
		if (direction == -1) {
			direction = 3;
		}

	}

	char goal_mode;
	char break_end_mode=0;
	if (pass[pass_count] >= 0){goal_mode=0;}//直進ゴールモード
	if (pass[pass_count] == -2){goal_mode=1;}//斜めゴールモード1
	if (pass[pass_count] == -3){goal_mode=2;}//斜めゴールモード2


	while (1) {

		get_wall(x,y,direction,&front_wall,&right_wall,&left_wall);


		switch (goal_mode) {
			case 0://直進ゴールモード
				if (front_wall) {
					break_end_mode=1;
				}else{
						pass[pass_count] = pass[pass_count] + 2;
				}
					break;
			case 1://斜めゴールモード1
				if (left_wall) {
					break_end_mode=1;
				}else{
					pass_count++;
					pass[pass_count] = -3;
					direction--;
					goal_mode=2;
				}
					break;
			case 2://斜めゴールモード2
				if (right_wall) {
					break_end_mode=1;
				}else{
					pass_count++;
					pass[pass_count] = -2;
					direction++;
					goal_mode=1;
				}
					break;

		}

		if(break_end_mode==1){break;}

		if (direction == 5) {
			direction = 1;
		}
		if (direction == 6) {
			direction = 2;
		}
		if (direction == 0) {
			direction = 4;
		}
		if (direction == -1) {
			direction = 3;
		}
		update_coordinate(&x,&y,direction);

	}



	if (pass[pass_count] >= 0) {
	} else {
		pass_count++;
	}
	pass[pass_count] = pass[pass_count] + 1;



}


void run_shortest(float inspeed, float inacc, float indec, char pass_mode, char fun_mode,
		char slant_mode, parameter_speed howspeed,float fun_V,char mollifier_mode,char max_mode) {
	unsigned char slant_count;
	int slant_direction;
	float first_v;//,last_v
	float end_velocity;

	g_dijkstra_parameter.max_velocity=inspeed;
    g_dijkstra_parameter.acceleration=inacc;
    g_dijkstra_parameter.Turn_parameter=howspeed;
	g_dijkstra_parameter.Turn_parameter_corrtime = convert_parameter_speed_to_corrtime(&g_dijkstra_parameter.Turn_parameter);

	slant_direction = -2;

	MOTOR_MODE mode;
	mode.WallControlMode=1;
	mode.WallControlStatus=0;
	mode.WallCutMode=0;
	mode.calMazeMode=0;

	//highspeed_mode = 1;
	for(int i = 0; i < PASS_NUM; i++){pass[i] = 0;}
	for(int i = 0; i <= MAZE_SQUARE_NUM-2; i++){
		record.row[i] = wall.row[i];
		record.column[i] = wall.column[i];
		record.row_look[i] = wall.row_look[i];
		record.column_look[i] = wall.column_look[i];
		wall.row_look[i] = ~wall.row_look[i];
		wall.column_look[i] = ~wall.column_look[i];
		wall.row[i] = wall.row[i] | wall.row_look[i];
		wall.column[i] = wall.column[i] | wall.column_look[i];
	}
	//pass_maker();
   pass_maker_Dijkstra();
   //maze_display_Dijkstra();

	pass_count = 1;
if(pass_mode==1){
	while (1) {		//パス圧縮

		if (pass[pass_count] == 0) {
			break;
		}

		if (pass[pass_count] == -2 && pass[pass_count - 1] >= 1	//右90度大回りの条件
		&& pass[pass_count + 1] >= 1) {
			pass[pass_count - 1] = pass[pass_count - 1] - 1;	//前90直進の削除
			pass[pass_count + 1] = pass[pass_count + 1] - 1;	//後90直進の削除
			pass[pass_count] = -4;		//右90度大回り

		}
		if (pass[pass_count] == -3 && pass[pass_count - 1] >= 1	//左90度大回りの条件
		&& pass[pass_count + 1] >= 1) {
			pass[pass_count - 1] = pass[pass_count - 1] - 1;	//前90直進の削除
			pass[pass_count + 1] = pass[pass_count + 1] - 1;	//後90直進の削除
			pass[pass_count] = -5;		//左90度大回り

		}
		if (pass[pass_count - 1] >= 1 && pass[pass_count] == -2
				&& pass[pass_count + 1] == -2 && pass[pass_count + 2] >= 1) {//右180度大回りの条件
			pass[pass_count - 1] = pass[pass_count - 1] - 1;
			pass[pass_count] = -6;
			pass[pass_count + 1] = -1;
			pass[pass_count + 2] = pass[pass_count + 2] - 1;

		}
		if (pass[pass_count - 1] >= 1 && pass[pass_count] == -3
				&& pass[pass_count + 1] == -3 && pass[pass_count + 2] >= 1) {//左180度大回りの条件
			pass[pass_count - 1] = pass[pass_count - 1] - 1;
			pass[pass_count] = -7;
			pass[pass_count + 1] = -1;
			pass[pass_count + 2] = pass[pass_count + 2] - 1;
		}
		if (pass[pass_count] == -2 && pass[pass_count - 1] == -3	//左90度大回りの条件

				) {
		}
//		if(){}
		if (pass[pass_count - 1] == 0) {
			pass[pass_count - 1] = -1;		//passが0になってしまったときの対策
		}

		pass_count++;
	}

	pass_count = 1;
	if (slant_mode == 1) {
		while (1) {		//斜め入出の圧縮
			if (pass[pass_count] == 0) {
				break;
			}

			if (pass[pass_count - 1] >= 1) {
				if (pass[pass_count] == -2 || pass[pass_count] == -3) {
//***************************************************************************************入りのモーションstart
					if (pass[pass_count] == -2 && pass[pass_count + 1] == -3) {
						pass[pass_count - 1] = pass[pass_count - 1] - 1;
						if (pass[pass_count - 1] == 0) {
							pass[pass_count - 1] = -1;	//passが0になってしまったときの対策
						}
						pass[pass_count] = -8;		//右45
					}
					if (pass[pass_count] == -3 && pass[pass_count + 1] == -2) {
						pass[pass_count - 1] = pass[pass_count - 1] - 1;
						if (pass[pass_count - 1] == 0) {
							pass[pass_count - 1] = -1;	//passが0になってしまったときの対策
						}
						pass[pass_count] = -9;		//左45
					}
					if (pass[pass_count] == -2 && pass[pass_count + 1] == -2) {
						pass[pass_count - 1] = pass[pass_count - 1] - 1;
						if (pass[pass_count - 1] == 0) {
							pass[pass_count - 1] = -1;	//passが0になってしまったときの対策
						}
						pass[pass_count] = -10;		//右135
						pass[pass_count + 1] = -1;
					}
					if (pass[pass_count] == -3 && pass[pass_count + 1] == -3) {
						pass[pass_count - 1] = pass[pass_count - 1] - 1;
						if (pass[pass_count - 1] == 0) {
							pass[pass_count - 1] = -1;	//passが0になってしまったときの対策
						}
						pass[pass_count] = -11;		//左135
						pass[pass_count + 1] = -1;
					}
//***************************************************************************************入りのモーションend

//***************************************************************************************途中のモーションstart
					while (pass[pass_count] <= -1) {
						pass_count++;
					}
//***************************************************************************************途中のモーションend

//***************************************************************************************出のモーションstart
					if (pass[pass_count - 1] == -2) {
						if (pass[pass_count - 2] == -2) {
							pass[pass_count] = pass[pass_count] - 1;
							if (pass[pass_count] == 0) {
								pass[pass_count] = -1;	//passが0になってしまったときの対策
							}
							pass[pass_count - 1] = -14;		//右135
							pass[pass_count - 2] = -1;
						} else {
							pass[pass_count] = pass[pass_count] - 1;
							if (pass[pass_count] == 0) {
								pass[pass_count] = -1;	//passが0になってしまったときの対策
							}
							pass[pass_count - 1] = -12;		//右45
						}

					}
					if (pass[pass_count - 1] == -3) {
						if (pass[pass_count - 2] == -3) {
							pass[pass_count] = pass[pass_count] - 1;
							if (pass[pass_count] == 0) {
								pass[pass_count] = -1;	//passが0になってしまったときの対策
							}
							pass[pass_count - 1] = -15;		//左135
							pass[pass_count - 2] = -1;
						} else {
							pass[pass_count] = pass[pass_count] - 1;
							if (pass[pass_count] == 0) {
								pass[pass_count] = -1;	//passが0になってしまったときの対策
							}
							pass[pass_count - 1] = -13;		//左45
						}

					}
//***************************************************************************************出のモーションend
				}
			}
			//		if(){}

			pass_count++;
		}

		pass_count = 1;
		while (1) {		//斜の圧縮
			if (pass[pass_count] == 0) {
				break;
			}

			if (pass[pass_count] == -8 || pass[pass_count] == -9
					|| pass[pass_count] == -10 || pass[pass_count] == -11) {
				if (pass[pass_count] == -8 || pass[pass_count] == -10) {
					slant_direction = -3;
				}
				if (pass[pass_count] == -9 || pass[pass_count] == -11) {
					slant_direction = -2;
				}
				pass_count++;
				if (pass[pass_count] == -1) {		//135ターンようのー１を進めるため
					pass_count++;
				}
				if (pass[pass_count] == -1) {		//135ターンようのー１を進めるため
					pass_count++;
				}
				if (pass[pass_count] >= -3) {
					slant_count = pass_count;
					pass[slant_count] = SLANT_PASS_COUNT+1;
					pass_count++;
				}

				//***************************************************************************************途中のモーションstart
				while (pass[pass_count] >= -3) {
					if (pass[pass_count] == -1) {		//135ターンようのー１を進めるため
						pass_count++;
					}
					if (pass[pass_count] == -12 || pass[pass_count] == -13
							|| pass[pass_count] == -14
							|| pass[pass_count] == -15) {
						break;
					}
					if (pass[pass_count] == slant_direction) {
						pass[slant_count] = pass[slant_count] - 1;
						slant_count = pass_count;
						if (slant_direction == -2) {
							pass[pass_count] = -16;
						}
						if (slant_direction == -3) {
							pass[pass_count] = -17;
						}

					} else {
						if (pass[slant_count] >= SLANT_PASS_COUNT) {
							pass[pass_count] = -1;
						} else {
							slant_count = pass_count;
							pass[slant_count] = SLANT_PASS_COUNT;
						}
						pass[slant_count] = pass[slant_count] + 1;
						if (slant_direction == -2) {
							slant_direction = -3;
						} else {
							slant_direction = -2;
						}

					}

					pass_count++;
				}
				//***************************************************************************************途中のモーションend

			}

			//		if(){}

			pass_count++;
		}
		pass_count=0;
		while (1) {		//パス圧縮

			if (pass[pass_count] == SLANT_PASS_COUNT) {
				pass[pass_count] =-1;

			}
			if (pass[pass_count] == 0) {
				break;
			}
			pass_count++;
		}

	}
}
	int j = 0;
	while (pass[j] != 0) {
		printf("pass_count %d pass %d\n", j, pass[j]);
		j++;
	}
	int pass_count2;
	pass_count2=0;
	while(pass[pass_count2] == -1){
		pass_count2++;
	}
	end_velocity=get_center_velocity(howspeed,pass[pass_count2]);
	printf("%d,%f\n",pass_count2, end_velocity);

	wait_ms_NoReset(500);
	pl_DriveMotor_standby(ON);
	pl_R_DriveMotor_mode(MOTOR_BREAK);
	pl_L_DriveMotor_mode(MOTOR_BREAK);
	wait_ms_NoReset(500);
	clear_Ierror();
	reset_gyro();
	reset_speed();
	////wall_control_mode = 1;
	if (fun_mode == 1) {
		start_fun(fun_V);
		//reset_gyro();
		reset_gyro_integral();
		reset_speed();
		clear_Ierror();
//		wait_ms_NoReset(1000);
//				reset_gyro();
//				enc.sigma_error = 0;
//					Gyro.sigma_error = 0;
//				pl_FunMotor_duty(160);

	}
	maze_mode = 1;
	highspeed_mode = 1;
	record_mode=2;
//	encoder_PID_error=2500;
//	gyro_PID_error=1800;
	pass_count = 0;
	float first_move_displacement = 0;
	uint8_t wallcut_mode=ON;

	mode.WallControlMode=1;
	mode.WallControlStatus=0;
	mode.calMazeMode=0;
	mode.WallCutMode=0;
	pass_count2=0;
	while(pass[pass_count2] == -1){
		pass_count2++;
	}
	end_velocity=get_center_velocity(howspeed,pass[pass_count2]);
	straight_acceleration_lpf=end_velocity*end_velocity/ BACK_TO_CENTER_FRONT;/* 加速度追従性向上のため2倍の加速度代入 */
	if ( pass[pass_count2] == -4 ){
		first_move_displacement = FIRST_MOVE_R90;
	}else if( pass[pass_count2] == -8 ){
		first_move_displacement = FIRST_MOVE_R45;
	}else if( pass[pass_count2] == -10 ){
		first_move_displacement = FIRST_MOVE_R135;
	}
	straight_table2(BACK_TO_CENTER_FRONT+first_move_displacement,0,end_velocity,end_velocity,end_velocity*end_velocity/ BACK_TO_CENTER_FRONT/2, mode);

	while (pass_count <= PASS_NUM) {
		pass_count2=pass_count+1;
		while(pass[pass_count2] == -1){
			pass_count2++;
		}
		end_velocity=get_center_velocity(howspeed,pass[pass_count2]);
		if (pass[pass_count2] == 0){/* 次のパスが終了のとき止まる準備(壁切れOFF) */
			wallcut_mode=OFF;
		}else{
			wallcut_mode=ON;
		}

		if (pass[pass_count] == -1) {
			pass_count++;
		}
		else if (pass[pass_count] == -2) {

			slalomR(howspeed.slalom_R, OFF,SHORTEST,mollifier_mode,end_velocity,wallcut_mode);

			pass_count++;
		}
		else if (pass[pass_count] == -3) {

			slalomL(howspeed.slalom_L, OFF,SHORTEST,mollifier_mode,end_velocity,wallcut_mode);

			pass_count++;
		}
		else if (pass[pass_count] == -4) {
			turn90R(howspeed.turn90_R, OFF,mollifier_mode,end_velocity,wallcut_mode);
			pass_count++;
		}
		else if (pass[pass_count] == -5) {
			turn90L(howspeed.turn90_L, OFF,mollifier_mode,end_velocity,wallcut_mode);
			pass_count++;
		}
		else if (pass[pass_count] == -6) {
			turn180R(howspeed.turn180_R, OFF,mollifier_mode,end_velocity,wallcut_mode);
			pass_count++;
		}
		else if (pass[pass_count] == -7) {
			turn180L(howspeed.turn180_L, OFF,mollifier_mode,end_velocity,wallcut_mode);
			pass_count++;
		}
		else if (pass[pass_count] == -8) { //入り45R
			turn45inR(howspeed.turn45in_R, OFF,mollifier_mode,end_velocity,wallcut_mode);
			pass_count++;
		}
		else if (pass[pass_count] == -9) { //入り45L
			turn45inL(howspeed.turn45in_L, OFF,mollifier_mode,end_velocity,wallcut_mode);
			pass_count++;
		}
		else if (pass[pass_count] == -10) { //入り135R
			turn135inR(howspeed.turn135in_R, OFF,mollifier_mode,end_velocity,wallcut_mode);
			pass_count++;
		}
		else if (pass[pass_count] == -11) { //入り135L
			turn135inL(howspeed.turn135in_L, OFF,mollifier_mode,end_velocity,wallcut_mode);
			pass_count++;
		}
		else if (pass[pass_count] == -12) { //出り45R
			turn45outR(howspeed.turn45out_R, OFF,mollifier_mode,end_velocity,wallcut_mode);
			pass_count++;
		}
		else if (pass[pass_count] == -13) { //出り45L
			turn45outL(howspeed.turn45out_L, OFF,mollifier_mode,end_velocity,wallcut_mode);
			pass_count++;
		}
		else if (pass[pass_count] == -14) { //出り135R
			turn135outR(howspeed.turn135out_R, OFF,mollifier_mode,end_velocity,wallcut_mode);
			pass_count++;
		}
		else if (pass[pass_count] == -15) { //出り135L
			turn135outL(howspeed.turn135out_L, OFF,mollifier_mode,end_velocity,wallcut_mode);
			pass_count++;
		}
		else if (pass[pass_count] == -16) { //V90R
			V90R(howspeed.V90_R, OFF,mollifier_mode,end_velocity,wallcut_mode);
			pass_count++;
		}
		else if (pass[pass_count] == -17) { //V90L
			V90L(howspeed.V90_L, OFF,mollifier_mode,end_velocity,wallcut_mode);
			pass_count++;
		}
		else if (pass[pass_count] >= 1) {
			first_v = howspeed.TurnCentervelocity;
			//last_v = howspeed.TurnCentervelocity;
			if (pass_count >= 1) {

				if (pass[pass_count - 1] == -2 || pass[pass_count - 1] == -3) {
					first_v = howspeed.SlalomCentervelocity;
				}
			}
			if (pass[pass_count + 1] == -2 || pass[pass_count + 1] == -3) {
				//last_v = howspeed.SlalomCentervelocity;
			}
			if (pass[pass_count] >= SLANT_PASS_COUNT) {
				mode.WallControlMode=3;
				mode.WallControlStatus=0;
				if(max_mode==0){
				straight_table2((45 * sqrt(2) * (pass[pass_count] - SLANT_PASS_COUNT)),first_v, end_velocity,inspeed, inacc, mode);
				}else if(max_mode==1){
				straight_table_dis((45 * sqrt(2) * (pass[pass_count] - SLANT_PASS_COUNT)),first_v, end_velocity,inspeed, inacc,indec, mode);
				}else if(max_mode==2){
				straight_table_max((45 * sqrt(2) * (pass[pass_count] - SLANT_PASS_COUNT)),first_v, end_velocity,inspeed, inacc,indec, mode);
				}
			} else {
				mode.WallControlMode=1;
				mode.WallControlStatus=0;
				if(max_mode==0){
				straight_table2((45 * pass[pass_count]),first_v, end_velocity,inspeed, inacc, mode);
				}else if(max_mode==1){
				straight_table_dis((45 * pass[pass_count]),first_v, end_velocity,inspeed, inacc,indec, mode);
				}else if(max_mode==2){
				straight_table_max((45 * pass[pass_count]),first_v, end_velocity,inspeed, inacc,indec, mode);
				}
			}

			pass_count++;
		}

		if (pass[pass_count] == 0) {
			break;
		}


//		if (mode_safty == 1) {
//
//			break;
//		}
	}

		mode.WallControlMode=1;
		mode.WallControlStatus=0;
		straight_table2(FRONT_TO_CENTER_FRONT,end_velocity,0,end_velocity,end_velocity*end_velocity/ FRONT_TO_CENTER_FRONT/2, mode);
		wait_ms_NoReset(700);
		stop_fun();
//		turning_table(180, 0, 0, 400, 5000);

	maze_mode = 0;
	highspeed_mode = 0;
	record_mode=0;
	pl_DriveMotor_standby(OFF);
	int t = 0;

	while (t <= MAZE_SQUARE_NUM-2) {
		wall.row[t] = record.row[t];
		wall.column[t] = record.column[t];
		t++;
	}
	t = 0;
	while (t <= MAZE_SQUARE_NUM-2) {
		wall.row_look[t] = ~wall.row_look[t];
		wall.column_look[t] = ~wall.column_look[t];
		t++;
	}

}











void decision_kitiku(int x,int y,int direction,unsigned short front_count,unsigned short right_count,unsigned short back_count,unsigned short left_count){
	_Bool front_wall=1;
	_Bool right_wall=1;
	_Bool left_wall=1;
	int x_front=x;
	int y_front=y;
	update_coordinate(&x_front,&y_front,direction);
	get_wall_look(x_front,y_front,direction,&front_wall,&right_wall,&left_wall);
	_Bool look_f=(front_wall && right_wall && left_wall);

	//ここに壁条件がない
	if (look_f && front_count <= right_count
			&& front_count <= left_count && front_count <= back_count) {
		if ((direction==1 && y>=MAZE_SQUARE_NUM-2) ||
			(direction==2 && x>=MAZE_SQUARE_NUM-2) ||
			(direction==3 && y<=1) ||
			(direction==4 && x<=1) ){
			kitikukan = 0;
		}else{
			kitikukan = 1;
		}

	} else {
		kitikukan = 0;
	}


}

void compress_kitiku(int *x,int *y,int *direction,int *kitiku_distance) {
	*kitiku_distance = 0;
	int kitiku = 1;
	_Bool front_wall;
	_Bool right_wall;
	_Bool left_wall;
	_Bool look_f,look_r,look_l;
	int x_now,y_now,direction_now;
	int x_front,y_front,x_right,y_right,x_left,y_left;
	int direction_right,direction_left;
	unsigned short front_count, right_count, back_count, left_count;
	x_now=*x;y_now=*y;direction_now=*direction;
	while (1) {
		update_coordinate(&x_now,&y_now,direction_now);
		x_front=x_now;y_front=y_now;x_right=x_now;y_right=y_now;x_left=x_now;y_left=y_now;

		update_coordinate(&x_front,&y_front,direction_now);
		get_wall_look(x_front,y_front,direction_now,&front_wall,&right_wall,&left_wall);
		look_f=(front_wall && right_wall && left_wall);


		if(direction_now==4){direction_right=1;}else{direction_right=direction_now+1;}
		update_coordinate(&x_right,&y_right,direction_right);
		get_wall_look(x_right,y_right,direction_right,&front_wall,&right_wall,&left_wall);
		look_r=(front_wall && right_wall && left_wall);


		if(direction_now==1){direction_left=4;}else{direction_left=direction_now-1;}
		update_coordinate(&x_left,&y_left,direction_left);
		get_wall_look(x_left,y_left,direction_left,&front_wall,&right_wall,&left_wall);
		look_l=(front_wall && right_wall && left_wall);

		get_wall(x_now,y_now,direction_now,&front_wall,&right_wall,&left_wall);

		search_AroundWalkCount(&front_count,&right_count,&back_count,&left_count,x_now,y_now,direction_now);
		if (front_wall) {front_count = MAX_WALKCOUNT;}
		if (right_wall) {right_count = MAX_WALKCOUNT;}
		if (left_wall) {left_count = MAX_WALKCOUNT;}
		// 移動の優先順位 ： 前→右→左→後
		if (walk_count[x_now][y_now] <= 1) {
			//goal間近で停止
			break;
		}
		if (direction_now==1 && y_now>=MAZE_SQUARE_NUM-2) {break;}
		if (direction_now==2 && x_now>=MAZE_SQUARE_NUM-2) {break;}
		if (direction_now==3 && y_now<=1) {break;}
		if (direction_now==4 && x_now<=1) {break;}
		if (front_count==MAX_WALKCOUNT && right_count==MAX_WALKCOUNT && left_count==MAX_WALKCOUNT && back_count==MAX_WALKCOUNT){
		// 迷路破損のため停止(一時停止後に周辺の地図情報を初期化して再探索に変更予定)
			error_mode=20;
			flash_in();
		break;
		}
		if (front_count <= right_count && front_count <= left_count && front_count <= back_count){
		// 直進
			if(look_f){
				*kitiku_distance += 2;
			}else{
				kitiku = 0;
				break;
			}
		}
		if(right_count < front_count && right_count <= left_count && right_count <= back_count){
		// 右旋回
			if(look_r){
				kitiku = 0;
				break;
			}else{
				kitiku = 0;
				break;
			}
			direction_now++;
		}
		if(left_count < front_count && left_count < right_count && left_count <= back_count){
		// 左旋回
			if(look_l){
				kitiku = 0;
				break;
			}else{
				kitiku = 0;
				break;
			}
			direction_now--;
		}
		if(back_count < front_count && back_count < right_count
								&& back_count < left_count){
		//180度旋回(前壁がある場合は尻当てを行うことで位置修正)
		//180度旋回(前壁がある場合は尻当てを行うことで位置修正)
			kitiku = 0;
			break;
			direction_now+=2;
		}


		if (direction_now == 5) {
			direction_now = 1;
		}
		if (direction_now == 6) {
			direction_now = 2;
		}
		if (direction_now == 0) {
			direction_now = 4;
		}
		if (direction_now == -1) {
			direction_now = 3;
		}
		if (kitiku == 0) {

			break;
		}

	}

	int direction2=direction_now+2;
	if (direction2 == 5) {
				direction2 = 1;
			}
			if (direction2 == 6) {
				direction2 = 2;
			}
			if (direction2 == 0) {
				direction2 = 4;
			}
			if (direction2 == -1) {
				direction2 = 3;
			}

	update_coordinate(&x_now,&y_now,direction2);

	*x=x_now;
	*y=y_now;
	*direction=direction_now;

}

