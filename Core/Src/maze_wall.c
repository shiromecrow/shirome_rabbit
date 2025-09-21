/*
 * maze_wall.c
 *
 *  Created on: Jan 9, 2024
 *      Author: sf199
 */


/*
 * maze_wall.c
 *
 *  Created on: 2023/06/29
 *      Author: sf199
 */


#include "maze_wall.h"
#include "fail_safe.h"
#include "maze_strategy.h"
#include "define.h"
#include <stdio.h>
#include <inttypes.h>
#include <math.h>



WALL wall;
WALL record;
WALL error_wall;

char Dijkstra_maker_flag;

uint16_t walk_count[MAZE_SQUARE_NUM][MAZE_SQUARE_NUM]; //歩数いれる箱
//uint16_t Network[MAZE_SQUARE_NUM*MAZE_SQUARE_NUM][MAZE_SQUARE_NUM*MAZE_SQUARE_NUM]; //歩数いれる箱
//uint16_t Network[3000][4];
DIJKSTRA Dijkstra;
STACK_T g_Goal_x;
STACK_T g_Goal_y;
Dijkstra_parameter g_dijkstra_parameter;

//int16_t discount_v[V_NUM_MAX]={129,74,61,53,45};//5
// int16_t discount_d[D_NUM_MAX]={91,58,49,43,32};//5
int16_t discount_v[V_NUM_MAX]={129,88,75,67,61,56,52,49,47,44,42,41,39,38,37,37,37,37,37,37};
int16_t discount_d[D_NUM_MAX]={91,67,59,53,49,45,42,40,38,36,35,34,32,31,30,29,29,28,27,27};

void maze_out_matlab(void){

	int tt=0;
	while (tt <= MAZE_SQUARE_NUM-2) {
		printf("g_maze_row(%d) = %" PRIu32 ";\n",tt+1,wall.row[tt]);
		printf("g_maze_column(%d) = %" PRIu32 ";\n",tt+1,wall.column[tt]);
		tt++;
	}
	tt = 0;

}



void maze_clear(void) { //初期化

	Dijkstra_maker_flag=0;
	int tt = 0;
	while (tt <= MAZE_SQUARE_NUM-2) {
		wall.row[tt] = 0;
		wall.column[tt] = 0;
		wall.row_look[tt] = 0;
		wall.column_look[tt] = 0;
		tt++;
	}
	tt = 0;

	g_dijkstra_parameter.max_velocity=5000;
    g_dijkstra_parameter.acceleration=35000;
    g_dijkstra_parameter.Turn_parameter=speed1600_shortest_mollifier;
	g_dijkstra_parameter.Turn_parameter_corrtime = convert_parameter_speed_to_corrtime(&g_dijkstra_parameter.Turn_parameter);
//	wall.row[0]=0;wall.row[1]=2;wall.row[2]=32762;wall.row[3]=50;wall.row[4]=16320;wall.row[5]=423;wall.row[6]=105;wall.row[7]=32490;
//	wall.row[8]=469;wall.row[9]=533;wall.row[10]=1258;wall.row[11]=3182;wall.row[12]=7837;wall.row[13]=13818;wall.row[14]=57342;
//	wall.column[0] = 20499;wall.column[1] = 8301;wall.column[2] = 61;wall.column[3] = 50;wall.column[4] = 6261;wall.column[5] = 10130;wall.column[6] = 4117;wall.column[7] = 3149;
//	wall.column[8] = 16085;wall.column[9] = 365;wall.column[10] = 725;wall.column[11] = 9837;wall.column[12] = 2773;wall.column[13] = 23149;wall.column[14] = 16381;




	for(int i=0;i<=MAZE_SQUARE_NUM-1;i++){
		for(int j=0;j<=MAZE_SQUARE_NUM-2;j++){
			Dijkstra.column_count[i][j]=MAX_WALKCOUNT_DIJKSTRA;
			Dijkstra.row_count[i][j]=MAX_WALKCOUNT_DIJKSTRA;
		}
	}
	// Dijkstra.row_count[GOAL_X][GOAL_Y]=0;
	// Dijkstra.row_count[GOAL_X+1][GOAL_Y]=0;
	// Dijkstra.column_count[GOAL_Y][GOAL_X]=0;
	// Dijkstra.column_count[GOAL_Y+1][GOAL_X]=0;
	for(int i=0;i<=GOAL_SIZE-1;i++){
		Dijkstra.row_count[GOAL_X+i][GOAL_Y]=0;
		Dijkstra.column_count[GOAL_Y+i][GOAL_X]=0;
	}


//ここから歩数マップの初期状態を作る．
create_StepCountMap_queue();

//kokomade

}



void update_wall(int x,int y,int direction,_Bool front_wall,_Bool right_wall,_Bool left_wall){
// x:x座標, y:y座標, direction:向き(北1東2南3西4),
//front_wall:前壁の有無(Ture=1 false=0), right_wall:右壁の有無(Ture=1 false=0), left_wall:左壁の有無(Ture=1 false=0)

	switch (direction) {
	case 1:
		if (y <= MAZE_SQUARE_NUM-2) {
			wall.row_look[y] = wall.row_look[y] | (1 << x);
			if(front_wall){wall.row[y] = wall.row[y] | (1 << x);}
		}

		if (x >= 1) {
			wall.column_look[x - 1] = wall.column_look[x - 1] | (1 << y);
			if(left_wall){wall.column[x - 1] = wall.column[x - 1] | (1 << y);}
		}

		if (x <= MAZE_SQUARE_NUM-2) {
			wall.column_look[x] = wall.column_look[x] | (1 << y);
			if(right_wall){wall.column[x] = wall.column[x] | (1 << y);}
		}

		break;
	case 2:
		if (x <= MAZE_SQUARE_NUM-2) {
			wall.column_look[x] = wall.column_look[x] | (1 << y);
			if(front_wall){wall.column[x] = wall.column[x] | (1 << y);}
		}

		if (y <= MAZE_SQUARE_NUM-2) {
			wall.row_look[y] = wall.row_look[y] | (1 << x);
			if(left_wall){wall.row[y] = wall.row[y] | (1 << x);}
		}

		if (y >= 1) {
			wall.row_look[y - 1] = wall.row_look[y - 1] | (1 << x);
			if(right_wall){wall.row[y - 1] = wall.row[y - 1] | (1 << x);}
		}

		break;
	case 3:
		if (y >= 1) {
			wall.row_look[y - 1] = wall.row_look[y - 1] | (1 << x);
			if(front_wall){wall.row[y - 1] = wall.row[y - 1] | (1 << x);}
		}

		if (x <= MAZE_SQUARE_NUM-2) {
			wall.column_look[x] = wall.column_look[x] | (1 << y);
			if(left_wall){wall.column[x] = wall.column[x] | (1 << y);}
		}

		if (x >= 1) {
			wall.column_look[x - 1] = wall.column_look[x - 1] | (1 << y);
			if(right_wall){wall.column[x - 1] = wall.column[x - 1] | (1 << y);}
		}

		break;
	case 4:
		if (x >= 1) {
			wall.column_look[x - 1] = wall.column_look[x - 1] | (1 << y);
			if(front_wall){wall.column[x - 1] = wall.column[x - 1] | (1 << y);}
		}

		if (y >= 1) {
			wall.row_look[y - 1] = wall.row_look[y - 1] | (1 << x);
			if(left_wall){wall.row[y - 1] = wall.row[y - 1] | (1 << x);}
		}

		if (y <= MAZE_SQUARE_NUM-2) {
			wall.row_look[y] = wall.row_look[y] | (1 << x);
			if(right_wall){wall.row[y] = wall.row[y] | (1 << x);}
		}

		break;

	}


}


void get_wall(int x,int y,int direction,_Bool* front_wall,_Bool* right_wall,_Bool* left_wall){
	*front_wall=1;
	*right_wall=1;
	*left_wall=1;
	switch (direction) {
	case 1:
		if (y <= MAZE_SQUARE_NUM-2) {
			*front_wall=((wall.row[y] & (1 << x)) == (1 << x));
		}
		if (x >= 1) {
			*left_wall=((wall.column[x - 1] & (1 << y)) == (1 << y));
		}
		if (x <= MAZE_SQUARE_NUM-2) {
			*right_wall=((wall.column[x] & (1 << y)) == (1 << y));
		}
		break;
	case 2:
		if (x <= MAZE_SQUARE_NUM-2) {
			*front_wall=((wall.column[x] & (1 << y)) == (1 << y));
		}
		if (y <= MAZE_SQUARE_NUM-2) {
			*left_wall=((wall.row[y] & (1 << x)) == (1 << x));
		}
		if (y >= 1) {
			*right_wall=((wall.row[y - 1] & (1 << x)) == (1 << x));
		}
		break;
	case 3:
		if (y >= 1) {
			*front_wall=((wall.row[y - 1] & (1 << x)) == (1 << x));
		}
		if (x <= MAZE_SQUARE_NUM-2) {
			*left_wall=((wall.column[x] & (1 << y)) == (1 << y));
		}
		if (x >= 1) {
			*right_wall=((wall.column[x - 1] & (1 << y)) == (1 << y));
		}
		break;
	case 4:
		if (x >= 1) {
			*front_wall=((wall.column[x - 1] & (1 << y)) == (1 << y));
		}
		if (y >= 1) {
			*left_wall=((wall.row[y - 1] & (1 << x)) == (1 << x));
		}
		if (y <= MAZE_SQUARE_NUM-2) {
			*right_wall=((wall.row[y] & (1 << x)) == (1 << x));
		}
		break;
	}

}


void get_wall_look(int x,int y,int direction,_Bool* front_wall,_Bool* right_wall,_Bool* left_wall){
	*front_wall=1;
	*right_wall=1;
	*left_wall=1;
	switch (direction) {
	case 1:
		if (y <= MAZE_SQUARE_NUM-2) {
			*front_wall=((wall.row_look[y] & (1 << x)) == (1 << x));
		}
		if (x >= 1) {
			*left_wall=((wall.column_look[x - 1] & (1 << y)) == (1 << y));
		}
		if (x <= MAZE_SQUARE_NUM-2) {
			*right_wall=((wall.column_look[x] & (1 << y)) == (1 << y));
		}
		break;
	case 2:
		if (x <= MAZE_SQUARE_NUM-2) {
			*front_wall=((wall.column_look[x] & (1 << y)) == (1 << y));
		}
		if (y <= MAZE_SQUARE_NUM-2) {
			*left_wall=((wall.row_look[y] & (1 << x)) == (1 << x));
		}
		if (y >= 1) {
			*right_wall=((wall.row_look[y - 1] & (1 << x)) == (1 << x));
		}
		break;
	case 3:
		if (y >= 1) {
			*front_wall=((wall.row_look[y - 1] & (1 << x)) == (1 << x));
		}
		if (x <= MAZE_SQUARE_NUM-2) {
			*left_wall=((wall.column_look[x] & (1 << y)) == (1 << y));
		}
		if (x >= 1) {
			*right_wall=((wall.column_look[x - 1] & (1 << y)) == (1 << y));
		}
		break;
	case 4:
		if (x >= 1) {
			*front_wall=((wall.column_look[x - 1] & (1 << y)) == (1 << y));
		}
		if (y >= 1) {
			*left_wall=((wall.row_look[y - 1] & (1 << x)) == (1 << x));
		}
		if (y <= MAZE_SQUARE_NUM-2) {
			*right_wall=((wall.row_look[y] & (1 << x)) == (1 << x));
		}
		break;
	}

}


int16_t add_or_zero(int16_t a, int16_t b) {
	int16_t c;
	c = a + b;
    if (c > 0) {
        return c;
    } else {
        return 0;
    }
}

/**
 * @brief 1ステップ分の移動コスト差分を計算する
 * 
 * @param step_index  移動ステップ番号（1以上）
 * @param is_diagonal 0:直進移動, 1:斜め移動
 * @param Turn_velocity 旋回速度
 * @param max_velocity 直進最高速
 * @param acc 加速度
 * @return 差分コスト（整数値）
 */
uint16_t compute_step_cost_diff(int step_index,uint8_t is_diagonal,float Turn_velocity,float max_velocity,float acc,int16_t correction_time){
    float secTOcost=1000;
    if (step_index < 0) return 0;
	// if(g_Dijkstra_ver3==OFF){
	// 	if(is_diagonal==0){return discount_v[step_index];}else{return discount_d[step_index];}
	// }
	
    float unit_distance = is_diagonal ? 63.6396f : 90.0f;//is_diagonalなら距離変更

    if (step_index == 0) {
        // 初期コスト：加速しない（旋回のみ）
        float start_cost = unit_distance / Turn_velocity * secTOcost;
		uint16_t time_cost=(uint16_t)roundf(add_or_zero(start_cost, correction_time));
        return time_cost;
    }


    float dist_prev = unit_distance * step_index;
    float dist_curr = unit_distance * (step_index + 1);


    float threshold = (max_velocity * max_velocity - Turn_velocity * Turn_velocity) / acc;//十分に加速できる距離があるか

    // 前ステップの移動時間
    float time_prev;
    if (dist_prev < threshold) {
        float peak_v_prev = sqrtf(Turn_velocity*Turn_velocity + acc * dist_prev);
        time_prev = 2.0f * (peak_v_prev - Turn_velocity) / acc * secTOcost;
    } else {
        float const_dist_prev = dist_prev - threshold;
        time_prev = (2.0f * (max_velocity - Turn_velocity) / acc + const_dist_prev / max_velocity) * secTOcost;
    }

    // 現ステップの移動時間
    float time_curr;
    if (dist_curr < threshold) {
        float peak_v_curr = sqrtf(Turn_velocity*Turn_velocity + acc * dist_curr);
        time_curr = 2.0f * (peak_v_curr - Turn_velocity) / acc * secTOcost;
    } else {
        float const_dist_curr = dist_curr - threshold;
        time_curr = (2.0f * (max_velocity - Turn_velocity) / acc + const_dist_curr / max_velocity) * secTOcost;
    }

    return (uint16_t)roundf(time_curr - time_prev);
}


/**
 * @brief 方向履歴に応じたターン速度を返す関数
 *
 * 指定された方向バッファ（dir_buf2, dir_buf1, dir）を元に、
 * 移動タイプ（直進・斜め）とターンパターン（45in/out, 90, 135in/out, 180, V90）を判定し、
 * それぞれに対応するターン中心速度を返す。
 * 
 * 使用例：経路探索中の動的コスト計算、走行制御フェーズの速度選択などに活用可能。
 * 
 * @param dir_buf2    2つ前の方向（0〜7: 方位定数）
 * @param dir_buf1    1つ前の方向（0〜7: 方位定数）
 * @param dir         現在の方向（0〜7: 方位定数、8はエラー）
 * @param dir_next    次の方向（0〜7: 方位定数、8はエラー）
 * @param param       Dijkstra_parameter 構造体ポインタ（ターンプロファイル含む）
 * @param turn_velocity 出力：ターン時の中心速度 [mm/s]
 * @param correction_time 出力：ターン前の補正時間 [ms]
 * @return ターン速度（float型, mm/s）
 */

void calculate_turn_profile(uint8_t dir_next,volatile uint8_t dir,uint8_t dir_buf1,uint8_t dir_buf2,float *turn_velocity,int16_t *correction_time) {
	*turn_velocity = g_dijkstra_parameter.Turn_parameter.TurnCentervelocity;
	*correction_time = 0;
	// if(g_Dijkstra_ver3==OFF){
	// 	return;
    // }
    // 異常値チェック
    if (dir >= 8) {
		return;
    }

    if ((dir_next & 1) == 0) {
        // 偶数方向 → 直進
        if (dir_buf1 == dir) {
            // 45inターン処理
			*turn_velocity = g_dijkstra_parameter.Turn_parameter.turn45in_R.g_speed;
			*correction_time = 0;
            return;
        }
		if((dir_buf1 & 1) == 0){
			// 90ターン処理(90 + 45.0f * sqrtf(2.0f))*1000
			*turn_velocity = g_dijkstra_parameter.Turn_parameter.turn90_R.g_speed;
			*correction_time = g_dijkstra_parameter.Turn_parameter_corrtime.turn90_corrtime;
            return;
		}
        if (dir_buf1 == dir_buf2) {
            // 135inターン処理(90 + 2 * 45.0f * sqrtf(2.0f))
			*turn_velocity = g_dijkstra_parameter.Turn_parameter.turn135in_R.g_speed;
			*correction_time = g_dijkstra_parameter.Turn_parameter_corrtime.turn135in_corrtime;
            return;
		}
        
		// 180ターン処理(2 * 90 + 2 * 45.0f * sqrtf(2.0f))
		*turn_velocity = g_dijkstra_parameter.Turn_parameter.turn180_R.g_speed;
		*correction_time = g_dijkstra_parameter.Turn_parameter_corrtime.turn180_corrtime;
        return;
            
		
    } else {
        // 奇数方向 → 斜め
        if ((dir & 1) == 0) {
            // 直進へ → 45outターン
			*turn_velocity = g_dijkstra_parameter.Turn_parameter.turn45out_R.g_speed;
			*correction_time = 0;
            return;
        }
        if (dir_buf1 == dir) {
            // V90ターン処理(45.0f * sqrtf(2.0f))
			*turn_velocity = g_dijkstra_parameter.Turn_parameter.V90_R.g_speed;
			*correction_time = g_dijkstra_parameter.Turn_parameter_corrtime.V90_corrtime;
            return;
        }

        // 135outターン処理(2 * 45.0f * sqrtf(2.0f))
		*turn_velocity = g_dijkstra_parameter.Turn_parameter.turn135out_R.g_speed;
		*correction_time = g_dijkstra_parameter.Turn_parameter_corrtime.turn135out_corrtime;
        return;
            
    }

}

parameter_speed_corrtime convert_parameter_speed_to_corrtime(const parameter_speed *src) {
    parameter_speed_corrtime corrtime;
	float secTOcost=1000;//defineのほうがいいかも

    corrtime.turn90_corrtime = (int16_t)roundf((90 + 45.0f * sqrtf(2.0f))*(1/g_dijkstra_parameter.Turn_parameter.turn90_R.g_speed-1/g_dijkstra_parameter.Turn_parameter.turn45in_R.g_speed) * secTOcost);
    corrtime.turn180_corrtime  =(int16_t)roundf((2 * 90 + 2 * 45.0f * sqrtf(2.0f))*(1/g_dijkstra_parameter.Turn_parameter.turn180_R.g_speed-1/g_dijkstra_parameter.Turn_parameter.turn45in_R.g_speed) * secTOcost);
    corrtime.turn135in_corrtime  =(int16_t)roundf((90 + 2 * 45.0f * sqrtf(2.0f))*(1/g_dijkstra_parameter.Turn_parameter.turn135in_R.g_speed-1/g_dijkstra_parameter.Turn_parameter.turn45in_R.g_speed) * secTOcost);
    corrtime.turn135out_corrtime   = (int16_t)roundf(2 * 45.0f * sqrtf(2.0f)*(1/g_dijkstra_parameter.Turn_parameter.turn135out_R.g_speed-1/g_dijkstra_parameter.Turn_parameter.turn45out_R.g_speed) * secTOcost);
    corrtime.V90_corrtime           = (int16_t)roundf(45.0f * sqrtf(2.0f)*(1/g_dijkstra_parameter.Turn_parameter.V90_R.g_speed-1/g_dijkstra_parameter.Turn_parameter.turn45out_R.g_speed) * secTOcost);

    return corrtime;
}





void search_AroundWalkCount(unsigned short *front_count,unsigned short *right_count,unsigned short *back_count,unsigned short *left_count,int x,int y,int direction){
//int direction,int x_coordinate,int y_coordinate
	unsigned short north_count,east_count,south_count,west_count;
//	unsigned short front_count, right_count, back_count, left_count;

	if (y >= MAZE_SQUARE_NUM-1) {north_count = MAX_WALKCOUNT;}
	else {north_count = walk_count[x][y + 1];}

	if (x >= MAZE_SQUARE_NUM-1) {east_count = MAX_WALKCOUNT;}
	else {east_count = walk_count[x + 1][y];}

	if (y <= 0) {south_count = MAX_WALKCOUNT;}
	else {south_count = walk_count[x][y - 1];}

	if (x <= 0) {west_count = MAX_WALKCOUNT;}
	else {west_count = walk_count[x - 1][y];}


	switch (direction) {		//
	case 1:
		*front_count = north_count;
		*right_count = east_count;
		*back_count = south_count;
		*left_count = west_count;
		break;
	case 2:
		*front_count = east_count;
		*right_count = south_count;
		*back_count = west_count;
		*left_count = north_count;
		break;
	case 3:
		*front_count = south_count;
		*right_count = west_count;
		*back_count = north_count;
		*left_count = east_count;
		break;
	case 4:
		*front_count = west_count;
		*right_count = north_count;
		*back_count = east_count;
		*left_count = south_count;
		break;

	}


}





void search_AroundDijkstraCount(unsigned short *front_count,unsigned short *right_count,unsigned short *back_count,unsigned short *left_count,int x,int y,int direction){
//int direction,int x_coordinate,int y_coordinate
	unsigned short north_count,east_count,south_count,west_count;
//	unsigned short front_count, right_count, back_count, left_count;

	if (y >= MAZE_SQUARE_NUM-1) {north_count = MAX_WALKCOUNT_DIJKSTRA;}
	else {north_count = Dijkstra.row_count[x][y];}

	if (x >= MAZE_SQUARE_NUM-1) {east_count = MAX_WALKCOUNT_DIJKSTRA;}
	else {east_count = Dijkstra.column_count[y][x];}

	if (y <= 0) {south_count = MAX_WALKCOUNT_DIJKSTRA;}
	else {south_count = Dijkstra.row_count[x][y-1];}

	if (x <= 0) {west_count = MAX_WALKCOUNT_DIJKSTRA;}
	else {west_count = Dijkstra.column_count[y][x-1];}


	switch (direction) {		//
	case 1:
		*front_count = north_count;
		*right_count = east_count;
		*back_count = south_count;
		*left_count = west_count;
		break;
	case 2:
		*front_count = east_count;
		*right_count = south_count;
		*back_count = west_count;
		*left_count = north_count;
		break;
	case 3:
		*front_count = south_count;
		*right_count = west_count;
		*back_count = north_count;
		*left_count = east_count;
		break;
	case 4:
		*front_count = west_count;
		*right_count = north_count;
		*back_count = east_count;
		*left_count = south_count;
		break;

	}


}





void create_DijkstraMap(void){
	STACK_T stack_x;
	STACK_T stack_y;
	STACK_T stack_matrix;//行列
	STACK_T stack_direction;//向き(0北　1北東　2東　3南東　4南　5南西　6西　7北西　8エラー)
	STACK_T stack_cost;//引かれるコスト
	int16_t VerticalCost=VERTICALCOST;
	int16_t DiagonalCost=DIAGONALCOST;
	int16_t dis_cost_in;
	initStack_walk(&stack_x);
	initStack_walk(&stack_y);
	initStack_walk(&stack_matrix);
	initStack_walk(&stack_direction);
	initStack_walk(&stack_cost);
	for(int i=0;i<=MAZE_SQUARE_NUM-1;i++){
		for(int j=0;j<=MAZE_SQUARE_NUM-2;j++){
			Dijkstra.column_count[i][j]=MAX_WALKCOUNT_DIJKSTRA;
			Dijkstra.row_count[i][j]=MAX_WALKCOUNT_DIJKSTRA;
		}
	}
	// Dijkstra.row_count[GOAL_X][GOAL_Y]=0;
	// Dijkstra.row_count[GOAL_X+1][GOAL_Y]=0;
	// Dijkstra.column_count[GOAL_Y][GOAL_X]=0;
	// Dijkstra.column_count[GOAL_Y+1][GOAL_X]=0;
	for(int i=0;i<=GOAL_SIZE-1;i++){
		Dijkstra.row_count[GOAL_X+i][GOAL_Y]=0;
		Dijkstra.column_count[GOAL_Y+i][GOAL_X]=0;
		pushStack_walk(&stack_x,GOAL_X+i);pushStack_walk(&stack_y,GOAL_Y);
		pushStack_walk(&stack_matrix,ROW);pushStack_walk(&stack_direction,8);pushStack_walk(&stack_cost,0);
		pushStack_walk(&stack_x,GOAL_X);pushStack_walk(&stack_y,GOAL_Y+i);
		pushStack_walk(&stack_matrix,COLUMN);pushStack_walk(&stack_direction,8);pushStack_walk(&stack_cost,0);
	}





	unsigned short Xcoordinate,Ycoordinate,Row_or_Column,Direction,dis_cost;
	while (1) {

		Xcoordinate = popStack_walk(&stack_x);
		Ycoordinate = popStack_walk(&stack_y);
		Row_or_Column = popStack_walk(&stack_matrix);
		Direction = popStack_walk(&stack_direction);
		dis_cost = popStack_walk(&stack_cost);
		//printf("x %d,y %d,C(0)R(1) %d\n",Xcoordinate,Ycoordinate,Row_or_Column);
		//printf("cost_num %d\n",dis_cost);
		//printf("x head %d tail %d\n y head %d tail %d\n",stack_x.head,stack_x.tail,stack_y.head,stack_y.tail);
		if (Xcoordinate == MAX_WALKCOUNT_DIJKSTRA || Ycoordinate == MAX_WALKCOUNT_DIJKSTRA || error_mode >= 1) {
			//printf("stack_end\n");
			break;
		}
		if(Row_or_Column==ROW){
			if(Ycoordinate <= MAZE_SQUARE_NUM-3){
				if(Direction==SLANT_NORTH){
					dis_cost_in=dis_cost+DISCOUNTCOST_V;
					if(dis_cost_in>=V_NUM_MAX){dis_cost_in=V_NUM_MAX-1;}
					VerticalCost=discount_v[dis_cost_in];
				}else{VerticalCost=discount_v[0];dis_cost_in=0;}
				if((wall.row[Ycoordinate+1] & (1 << Xcoordinate))==0 && Dijkstra.row_count[Xcoordinate][Ycoordinate+1]>Dijkstra.row_count[Xcoordinate][Ycoordinate]+VerticalCost){
					Dijkstra.row_count[Xcoordinate][Ycoordinate+1]=Dijkstra.row_count[Xcoordinate][Ycoordinate]+VerticalCost;
					pushStack_walk(&stack_x,Xcoordinate);
					pushStack_walk(&stack_y,Ycoordinate + 1);
					pushStack_walk(&stack_matrix,ROW);
					pushStack_walk(&stack_direction,SLANT_NORTH);
					pushStack_walk(&stack_cost,dis_cost_in);
				}
			}
			if (Ycoordinate >= 1) {
				if(Direction==SLANT_SOUTH){
					dis_cost_in=dis_cost+DISCOUNTCOST_V;
					if(dis_cost_in>=V_NUM_MAX){dis_cost_in=V_NUM_MAX-1;}
					VerticalCost=discount_v[dis_cost_in];
				}else{VerticalCost=discount_v[0];dis_cost_in=0;}
				if((wall.row[Ycoordinate-1] & (1 << Xcoordinate))==0 && Dijkstra.row_count[Xcoordinate][Ycoordinate-1]>Dijkstra.row_count[Xcoordinate][Ycoordinate]+VerticalCost){
					Dijkstra.row_count[Xcoordinate][Ycoordinate-1]=Dijkstra.row_count[Xcoordinate][Ycoordinate]+VerticalCost;
					pushStack_walk(&stack_x,Xcoordinate);
					pushStack_walk(&stack_y,Ycoordinate - 1);
					pushStack_walk(&stack_matrix,ROW);
					pushStack_walk(&stack_direction,SLANT_SOUTH);
					pushStack_walk(&stack_cost,dis_cost_in);
				}
			}
			if (Xcoordinate <= MAZE_SQUARE_NUM-2) {
				if(Direction==SLANT_SOUTH_EAST){
					dis_cost_in=dis_cost+DISCOUNTCOST_D;
					if(dis_cost_in>=D_NUM_MAX){dis_cost_in=D_NUM_MAX-1;}
					DiagonalCost=discount_d[dis_cost_in];
				}else{DiagonalCost=discount_d[0];dis_cost_in=0;}
				if((wall.column[Xcoordinate] & (1 << Ycoordinate))==0 && Dijkstra.column_count[Ycoordinate][Xcoordinate]>Dijkstra.row_count[Xcoordinate][Ycoordinate]+DiagonalCost){
					Dijkstra.column_count[Ycoordinate][Xcoordinate]=Dijkstra.row_count[Xcoordinate][Ycoordinate]+DiagonalCost;
					pushStack_walk(&stack_x,Xcoordinate);
					pushStack_walk(&stack_y,Ycoordinate);
					pushStack_walk(&stack_matrix,COLUMN);
					pushStack_walk(&stack_direction,SLANT_SOUTH_EAST);
					pushStack_walk(&stack_cost,dis_cost_in);
				}
				if(Direction==SLANT_NORTH_EAST){
					dis_cost_in=dis_cost+DISCOUNTCOST_D;
					if(dis_cost_in>=D_NUM_MAX){dis_cost_in=D_NUM_MAX-1;}
					DiagonalCost=discount_d[dis_cost_in];
				}else{DiagonalCost=discount_d[0];dis_cost_in=0;}
				if((wall.column[Xcoordinate] & (1 << (Ycoordinate+1)))==0 && Dijkstra.column_count[Ycoordinate+1][Xcoordinate]>Dijkstra.row_count[Xcoordinate][Ycoordinate]+DiagonalCost){
					Dijkstra.column_count[Ycoordinate+1][Xcoordinate]=Dijkstra.row_count[Xcoordinate][Ycoordinate]+DiagonalCost;
					pushStack_walk(&stack_x,Xcoordinate);
					pushStack_walk(&stack_y,Ycoordinate+1);
					pushStack_walk(&stack_matrix,COLUMN);
					pushStack_walk(&stack_direction,SLANT_NORTH_EAST);
					pushStack_walk(&stack_cost,dis_cost_in);
				}
			}
			if (Xcoordinate >= 1) {
				if(Direction==SLANT_SOUTH_WEST){
					dis_cost_in=dis_cost+DISCOUNTCOST_D;
					if(dis_cost_in>=D_NUM_MAX){dis_cost_in=D_NUM_MAX-1;}
					DiagonalCost=discount_d[dis_cost_in];
				}else{DiagonalCost=discount_d[0];dis_cost_in=0;}
				if((wall.column[Xcoordinate-1] & (1 << Ycoordinate))==0 && Dijkstra.column_count[Ycoordinate][Xcoordinate-1]>Dijkstra.row_count[Xcoordinate][Ycoordinate]+DiagonalCost){
					Dijkstra.column_count[Ycoordinate][Xcoordinate-1]=Dijkstra.row_count[Xcoordinate][Ycoordinate]+DiagonalCost;
					pushStack_walk(&stack_x,Xcoordinate-1);
					pushStack_walk(&stack_y,Ycoordinate);
					pushStack_walk(&stack_matrix,COLUMN);
					pushStack_walk(&stack_direction,SLANT_SOUTH_WEST);
					pushStack_walk(&stack_cost,dis_cost_in);
				}
				if(Direction==SLANT_NORTH_WEST){
					dis_cost_in=dis_cost+DISCOUNTCOST_D;
					if(dis_cost_in>=D_NUM_MAX){dis_cost_in=D_NUM_MAX-1;}
					DiagonalCost=discount_d[dis_cost_in];
				}else{DiagonalCost=discount_d[0];dis_cost_in=0;}
				if((wall.column[Xcoordinate-1] & (1 << (Ycoordinate+1)))==0 && Dijkstra.column_count[Ycoordinate+1][Xcoordinate-1]>Dijkstra.row_count[Xcoordinate][Ycoordinate]+DiagonalCost){
					Dijkstra.column_count[Ycoordinate+1][Xcoordinate-1]=Dijkstra.row_count[Xcoordinate][Ycoordinate]+DiagonalCost;
					pushStack_walk(&stack_x,Xcoordinate-1);
					pushStack_walk(&stack_y,Ycoordinate+1);
					pushStack_walk(&stack_matrix,COLUMN);
					pushStack_walk(&stack_direction,SLANT_NORTH_WEST);
					pushStack_walk(&stack_cost,dis_cost_in);
				}
			}

		}
		if(Row_or_Column==COLUMN){
					if(Xcoordinate <= MAZE_SQUARE_NUM-3){
						if(Direction==SLANT_EAST){
							dis_cost_in=dis_cost+DISCOUNTCOST_V;
							if(dis_cost_in>=V_NUM_MAX){dis_cost_in=V_NUM_MAX-1;}
							VerticalCost=discount_v[dis_cost_in];
						}else{VerticalCost=discount_v[0];dis_cost_in=0;}
						if((wall.column[Xcoordinate+1] & (1 << Ycoordinate))==0 && Dijkstra.column_count[Ycoordinate][Xcoordinate+1]>Dijkstra.column_count[Ycoordinate][Xcoordinate]+VerticalCost){
							Dijkstra.column_count[Ycoordinate][Xcoordinate+1]=Dijkstra.column_count[Ycoordinate][Xcoordinate]+VerticalCost;
							pushStack_walk(&stack_x,Xcoordinate + 1);
							pushStack_walk(&stack_y,Ycoordinate);
							pushStack_walk(&stack_matrix,COLUMN);
							pushStack_walk(&stack_direction,SLANT_EAST);
							pushStack_walk(&stack_cost,dis_cost_in);
						}
					}
					if (Xcoordinate >= 1) {
						if(Direction==SLANT_WEST){
							dis_cost_in=dis_cost+DISCOUNTCOST_V;
							if(dis_cost_in>=V_NUM_MAX){dis_cost_in=V_NUM_MAX-1;}
							VerticalCost=discount_v[dis_cost_in];
						}else{VerticalCost=discount_v[0];dis_cost_in=0;}
						if((wall.column[Xcoordinate-1] & (1 << Ycoordinate))==0 && Dijkstra.column_count[Ycoordinate][Xcoordinate-1]>Dijkstra.column_count[Ycoordinate][Xcoordinate]+VerticalCost){
							Dijkstra.column_count[Ycoordinate][Xcoordinate-1]=Dijkstra.column_count[Ycoordinate][Xcoordinate]+VerticalCost;
							pushStack_walk(&stack_x,Xcoordinate - 1);
							pushStack_walk(&stack_y,Ycoordinate);
							pushStack_walk(&stack_matrix,COLUMN);
							pushStack_walk(&stack_direction,SLANT_WEST);
							pushStack_walk(&stack_cost,dis_cost_in);
						}
					}
					if (Ycoordinate <= MAZE_SQUARE_NUM-2) {
						if(Direction==SLANT_NORTH_WEST){
							dis_cost_in=dis_cost+DISCOUNTCOST_D;
							if(dis_cost_in>=D_NUM_MAX){dis_cost_in=D_NUM_MAX-1;}
							DiagonalCost=discount_d[dis_cost_in];
						}else{DiagonalCost=discount_d[0];dis_cost_in=0;}
						if((wall.row[Ycoordinate] & (1 << Xcoordinate))==0 && Dijkstra.row_count[Xcoordinate][Ycoordinate]>Dijkstra.column_count[Ycoordinate][Xcoordinate]+DiagonalCost){
							Dijkstra.row_count[Xcoordinate][Ycoordinate]=Dijkstra.column_count[Ycoordinate][Xcoordinate]+DiagonalCost;
							pushStack_walk(&stack_x,Xcoordinate);
							pushStack_walk(&stack_y,Ycoordinate);
							pushStack_walk(&stack_matrix,ROW);
							pushStack_walk(&stack_direction,SLANT_NORTH_WEST);
							pushStack_walk(&stack_cost,dis_cost_in);
						}
						if(Direction==SLANT_NORTH_EAST){
							dis_cost_in=dis_cost+DISCOUNTCOST_D;
							if(dis_cost_in>=D_NUM_MAX){dis_cost_in=D_NUM_MAX-1;}
							DiagonalCost=discount_d[dis_cost_in];
						}else{DiagonalCost=discount_d[0];dis_cost_in=0;}
						if((wall.row[Ycoordinate] & (1 << (Xcoordinate+1)))==0 && Dijkstra.row_count[Xcoordinate+1][Ycoordinate]>Dijkstra.column_count[Ycoordinate][Xcoordinate]+DiagonalCost){
							Dijkstra.row_count[Xcoordinate+1][Ycoordinate]=Dijkstra.column_count[Ycoordinate][Xcoordinate]+DiagonalCost;
							pushStack_walk(&stack_x,Xcoordinate + 1);
							pushStack_walk(&stack_y,Ycoordinate);
							pushStack_walk(&stack_matrix,ROW);
							pushStack_walk(&stack_direction,SLANT_NORTH_EAST);
							pushStack_walk(&stack_cost,dis_cost_in);
						}
					}
					if (Ycoordinate >= 1) {
						if(Direction==SLANT_SOUTH_WEST){
							dis_cost_in=dis_cost+DISCOUNTCOST_D;
							if(dis_cost_in>=D_NUM_MAX){dis_cost_in=D_NUM_MAX-1;}
							DiagonalCost=discount_d[dis_cost_in];
						}else{DiagonalCost=discount_d[0];dis_cost_in=0;}
						if((wall.row[Ycoordinate-1] & (1 << Xcoordinate))==0 && Dijkstra.row_count[Xcoordinate][Ycoordinate-1]>Dijkstra.column_count[Ycoordinate][Xcoordinate]+DiagonalCost){
							Dijkstra.row_count[Xcoordinate][Ycoordinate-1]=Dijkstra.column_count[Ycoordinate][Xcoordinate]+DiagonalCost;
							pushStack_walk(&stack_x,Xcoordinate);
							pushStack_walk(&stack_y,Ycoordinate - 1);
							pushStack_walk(&stack_matrix,ROW);
							pushStack_walk(&stack_direction,SLANT_SOUTH_WEST);
							pushStack_walk(&stack_cost,dis_cost_in);
						}
						if(Direction==SLANT_SOUTH_EAST){
							dis_cost_in=dis_cost+DISCOUNTCOST_D;
							if(dis_cost_in>=D_NUM_MAX){dis_cost_in=D_NUM_MAX-1;}
							DiagonalCost=discount_d[dis_cost_in];
						}else{DiagonalCost=discount_d[0];dis_cost_in=0;}
						if((wall.row[Ycoordinate-1] & (1 << (Xcoordinate+1)))==0 && Dijkstra.row_count[Xcoordinate+1][Ycoordinate-1]>Dijkstra.column_count[Ycoordinate][Xcoordinate]+DiagonalCost){
							Dijkstra.row_count[Xcoordinate+1][Ycoordinate-1]=Dijkstra.column_count[Ycoordinate][Xcoordinate]+DiagonalCost;
							pushStack_walk(&stack_x,Xcoordinate+1);
							pushStack_walk(&stack_y,Ycoordinate-1);
							pushStack_walk(&stack_matrix,ROW);
							pushStack_walk(&stack_direction,SLANT_SOUTH_EAST);
							pushStack_walk(&stack_cost,dis_cost_in);
						}
					}

				}

		}



}


void create_DijkstraMap3(void){
	MinHeap heap;
	int16_t VerticalCost=VERTICALCOST;
	int16_t DiagonalCost=DIAGONALCOST;
	int16_t dis_cost_in;
	
	initHeap(&heap);

	for(int i=0;i<=MAZE_SQUARE_NUM-1;i++){
		for(int j=0;j<=MAZE_SQUARE_NUM-2;j++){
			Dijkstra.column_count[i][j]=MAX_WALKCOUNT_DIJKSTRA;
			Dijkstra.row_count[i][j]=MAX_WALKCOUNT_DIJKSTRA;
		}
	}
// 初期ノード群（GOALまわり）を pushHeap で入れる
DijkstraNode in_node;

	for(int i=0;i<=GOAL_SIZE-1;i++){
		Dijkstra.row_count[GOAL_X+i][GOAL_Y]=0;
		Dijkstra.column_count[GOAL_Y+i][GOAL_X]=0;
		in_node = (DijkstraNode){	
			.x = GOAL_X + i, .y = GOAL_Y, .matrix = ROW,
			.direction = 8, .direction_buf1 = 8, .direction_buf2 = 8,
			.dis_cost = 0, .total_cost = 0
			};
		pushHeap(&heap, in_node);
		in_node = (DijkstraNode){	
			.x = GOAL_X, .y = GOAL_Y + i, .matrix = COLUMN,
			.direction = 8, .direction_buf1 = 8, .direction_buf2 = 8,
			.dis_cost = 0, .total_cost = 0
			};
		pushHeap(&heap, in_node);
	}




	DijkstraNode out_node;
	unsigned short Direction_next;
	float Turn_speed=g_dijkstra_parameter.Turn_parameter.TurnCentervelocity;
	int16_t correction_time=0;
	while (!isHeapEmpty(&heap)) {
		out_node = popHeap(&heap);
		Direction_next = 8;
		correction_time=0;		
		//printf("x %d,y %d,C(0)R(1) %d\n",out_node.x,out_node.y,out_node.matrix);
		//printf("cost_num %d\n",out_node.dis_cost);
		//printf("x head %d tail %d\n y head %d tail %d\n",stack_x.head,stack_x.tail,stack_y.head,stack_y.tail);
		if (error_mode >= 1) {
			//printf("stack_end\n");
			break;
		}
		if(out_node.matrix==ROW){// 横壁の位置
			if(out_node.y <= MAZE_SQUARE_NUM-3){
				// 北向きの計算(上)
				Direction_next = SLANT_NORTH;
				if(out_node.direction==Direction_next){
					dis_cost_in=out_node.dis_cost+DISCOUNTCOST_V;
					if(dis_cost_in>=V_NUM_MAX){dis_cost_in=V_NUM_MAX-1;}
				}else{
                    dis_cost_in=0;
					calculate_turn_profile(Direction_next,out_node.direction,out_node.direction_buf1,out_node.direction_buf2,&Turn_speed,&correction_time);
                }
                VerticalCost=compute_step_cost_diff(dis_cost_in,0,Turn_speed,g_dijkstra_parameter.max_velocity,g_dijkstra_parameter.acceleration,correction_time);
				if((wall.row[out_node.y+1] & (1 << out_node.x))==0 && Dijkstra.row_count[out_node.x][out_node.y+1]>Dijkstra.row_count[out_node.x][out_node.y]+VerticalCost){
					Dijkstra.row_count[out_node.x][out_node.y+1]=Dijkstra.row_count[out_node.x][out_node.y]+VerticalCost;
                    Dijkstra.row_direction[out_node.x][out_node.y+1]=out_node.direction;
					in_node = (DijkstraNode){	
					.x = out_node.x, .y = out_node.y + 1, .matrix = ROW,
					.direction = Direction_next, .direction_buf1 = out_node.direction, .direction_buf2 = out_node.direction_buf1,
					.dis_cost = dis_cost_in, .total_cost = Dijkstra.row_count[out_node.x][out_node.y+1]
					};
					pushHeap(&heap, in_node);
				}
			}
			if (out_node.y >= 1) {
				// 南向きの計算(下)
				Direction_next = SLANT_SOUTH;
				if(out_node.direction==Direction_next){
					dis_cost_in=out_node.dis_cost+DISCOUNTCOST_V;
					if(dis_cost_in>=V_NUM_MAX){dis_cost_in=V_NUM_MAX-1;}
				}else{
                    dis_cost_in=0;
					calculate_turn_profile(Direction_next,out_node.direction,out_node.direction_buf1,out_node.direction_buf2,&Turn_speed,&correction_time);
                }
                VerticalCost=compute_step_cost_diff(dis_cost_in,0,Turn_speed,g_dijkstra_parameter.max_velocity,g_dijkstra_parameter.acceleration,correction_time);
				if((wall.row[out_node.y-1] & (1 << out_node.x))==0 && Dijkstra.row_count[out_node.x][out_node.y-1]>Dijkstra.row_count[out_node.x][out_node.y]+VerticalCost){
					Dijkstra.row_count[out_node.x][out_node.y-1]=Dijkstra.row_count[out_node.x][out_node.y]+VerticalCost;
                    Dijkstra.row_direction[out_node.x][out_node.y-1]=out_node.direction;
					in_node = (DijkstraNode){	
					.x = out_node.x, .y = out_node.y - 1, .matrix = ROW,
					.direction = Direction_next, .direction_buf1 = out_node.direction, .direction_buf2 = out_node.direction_buf1,
					.dis_cost = dis_cost_in, .total_cost = Dijkstra.row_count[out_node.x][out_node.y-1]
					};
					pushHeap(&heap, in_node);
				}
			}
			if (out_node.x <= MAZE_SQUARE_NUM-2) {
				// 南東向きの計算(右下)
				Direction_next = SLANT_SOUTH_EAST;
				if(out_node.direction==Direction_next){
					dis_cost_in=out_node.dis_cost+DISCOUNTCOST_D;
					if(dis_cost_in>=D_NUM_MAX){dis_cost_in=D_NUM_MAX-1;}
				}else{
                    dis_cost_in=0;
					calculate_turn_profile(Direction_next,out_node.direction,out_node.direction_buf1,out_node.direction_buf2,&Turn_speed,&correction_time);
                }
                DiagonalCost=compute_step_cost_diff(dis_cost_in,1,Turn_speed,g_dijkstra_parameter.max_velocity,g_dijkstra_parameter.acceleration,correction_time);
				if((wall.column[out_node.x] & (1 << out_node.y))==0 && Dijkstra.column_count[out_node.y][out_node.x]>Dijkstra.row_count[out_node.x][out_node.y]+DiagonalCost){
					Dijkstra.column_count[out_node.y][out_node.x]=Dijkstra.row_count[out_node.x][out_node.y]+DiagonalCost;
                    Dijkstra.column_direction[out_node.y][out_node.x]=out_node.direction;
					in_node = (DijkstraNode){	
					.x = out_node.x, .y = out_node.y, .matrix = COLUMN,
					.direction = Direction_next, .direction_buf1 = out_node.direction, .direction_buf2 = out_node.direction_buf1,
					.dis_cost = dis_cost_in, .total_cost = Dijkstra.column_count[out_node.y][out_node.x]
					};
					pushHeap(&heap, in_node);
				}
				// 北東向きの計算(右上)
				Direction_next = SLANT_NORTH_EAST;
				if(out_node.direction==Direction_next){
					dis_cost_in=out_node.dis_cost+DISCOUNTCOST_D;
					if(dis_cost_in>=D_NUM_MAX){dis_cost_in=D_NUM_MAX-1;}
				}else{
                    dis_cost_in=0;
					calculate_turn_profile(Direction_next,out_node.direction,out_node.direction_buf1,out_node.direction_buf2,&Turn_speed,&correction_time);
                }
                DiagonalCost=compute_step_cost_diff(dis_cost_in,1,Turn_speed,g_dijkstra_parameter.max_velocity,g_dijkstra_parameter.acceleration,correction_time);
				if((wall.column[out_node.x] & (1 << (out_node.y+1)))==0 && Dijkstra.column_count[out_node.y+1][out_node.x]>Dijkstra.row_count[out_node.x][out_node.y]+DiagonalCost){
					Dijkstra.column_count[out_node.y+1][out_node.x]=Dijkstra.row_count[out_node.x][out_node.y]+DiagonalCost;
                    Dijkstra.column_direction[out_node.y+1][out_node.x]=out_node.direction;
					in_node = (DijkstraNode){	
					.x = out_node.x, .y = out_node.y+1, .matrix = COLUMN,
					.direction = Direction_next, .direction_buf1 = out_node.direction, .direction_buf2 = out_node.direction_buf1,
					.dis_cost = dis_cost_in, .total_cost = Dijkstra.column_count[out_node.y+1][out_node.x]
					};
					pushHeap(&heap, in_node);
				}
			}
			if (out_node.x >= 1) {
				// 南西向きの計算(左下)
				Direction_next = SLANT_SOUTH_WEST;
				if(out_node.direction==Direction_next){
					dis_cost_in=out_node.dis_cost+DISCOUNTCOST_D;
					if(dis_cost_in>=D_NUM_MAX){dis_cost_in=D_NUM_MAX-1;}
				}else{
                    dis_cost_in=0;
					calculate_turn_profile(Direction_next,out_node.direction,out_node.direction_buf1,out_node.direction_buf2,&Turn_speed,&correction_time);
                }
                DiagonalCost=compute_step_cost_diff(dis_cost_in,1,Turn_speed,g_dijkstra_parameter.max_velocity,g_dijkstra_parameter.acceleration,correction_time);
				if((wall.column[out_node.x-1] & (1 << out_node.y))==0 && Dijkstra.column_count[out_node.y][out_node.x-1]>Dijkstra.row_count[out_node.x][out_node.y]+DiagonalCost){
					Dijkstra.column_count[out_node.y][out_node.x-1]=Dijkstra.row_count[out_node.x][out_node.y]+DiagonalCost;
                    Dijkstra.column_direction[out_node.y][out_node.x-1]=out_node.direction;
					in_node = (DijkstraNode){	
					.x = out_node.x-1, .y = out_node.y, .matrix = COLUMN,
					.direction = Direction_next, .direction_buf1 = out_node.direction, .direction_buf2 = out_node.direction_buf1,
					.dis_cost = dis_cost_in, .total_cost = Dijkstra.column_count[out_node.y][out_node.x-1]
					};
					pushHeap(&heap, in_node);
				}
				// 北西向きの計算(左上)
				Direction_next = SLANT_NORTH_WEST;
				if(out_node.direction==Direction_next){
					dis_cost_in=out_node.dis_cost+DISCOUNTCOST_D;
					if(dis_cost_in>=D_NUM_MAX){dis_cost_in=D_NUM_MAX-1;}
				}else{
                    dis_cost_in=0;
					calculate_turn_profile(Direction_next,out_node.direction,out_node.direction_buf1,out_node.direction_buf2,&Turn_speed,&correction_time);
                }
                DiagonalCost=compute_step_cost_diff(dis_cost_in,1,Turn_speed,g_dijkstra_parameter.max_velocity,g_dijkstra_parameter.acceleration,correction_time);
				if((wall.column[out_node.x-1] & (1 << (out_node.y+1)))==0 && Dijkstra.column_count[out_node.y+1][out_node.x-1]>Dijkstra.row_count[out_node.x][out_node.y]+DiagonalCost){
					Dijkstra.column_count[out_node.y+1][out_node.x-1]=Dijkstra.row_count[out_node.x][out_node.y]+DiagonalCost;
                    Dijkstra.column_direction[out_node.y+1][out_node.x-1]=out_node.direction;
					in_node = (DijkstraNode){	
					.x = out_node.x-1, .y = out_node.y+1, .matrix = COLUMN,
					.direction = Direction_next, .direction_buf1 = out_node.direction, .direction_buf2 = out_node.direction_buf1,
					.dis_cost = dis_cost_in, .total_cost = Dijkstra.column_count[out_node.y+1][out_node.x-1]
					};
					pushHeap(&heap, in_node);
				}
			}

		}
		if(out_node.matrix==COLUMN){
					if(out_node.x <= MAZE_SQUARE_NUM-3){
						// 東向きの計算(右)
						Direction_next = SLANT_EAST;
						if(out_node.direction==Direction_next){
							dis_cost_in=out_node.dis_cost+DISCOUNTCOST_V;
							if(dis_cost_in>=V_NUM_MAX){dis_cost_in=V_NUM_MAX-1;}
						}else{
                            dis_cost_in=0;
							calculate_turn_profile(Direction_next,out_node.direction,out_node.direction_buf1,out_node.direction_buf2,&Turn_speed,&correction_time);
                        }
                        VerticalCost=compute_step_cost_diff(dis_cost_in,0,Turn_speed,g_dijkstra_parameter.max_velocity,g_dijkstra_parameter.acceleration,correction_time);
						if((wall.column[out_node.x+1] & (1 << out_node.y))==0 && Dijkstra.column_count[out_node.y][out_node.x+1]>Dijkstra.column_count[out_node.y][out_node.x]+VerticalCost){
							Dijkstra.column_count[out_node.y][out_node.x+1]=Dijkstra.column_count[out_node.y][out_node.x]+VerticalCost;
                            Dijkstra.column_direction[out_node.y][out_node.x+1]=out_node.direction;
							in_node = (DijkstraNode){	
							.x = out_node.x+1, .y = out_node.y, .matrix = COLUMN,
							.direction = Direction_next, .direction_buf1 = out_node.direction, .direction_buf2 = out_node.direction_buf1,
							.dis_cost = dis_cost_in, .total_cost = Dijkstra.column_count[out_node.y][out_node.x+1]
							};
							pushHeap(&heap, in_node);
						}
					}
					if (out_node.x >= 1) {
						// 西向きの計算(左)
						Direction_next = SLANT_WEST;
						if(out_node.direction==Direction_next){
							dis_cost_in=out_node.dis_cost+DISCOUNTCOST_V;
							if(dis_cost_in>=V_NUM_MAX){dis_cost_in=V_NUM_MAX-1;}
						}else{
                            dis_cost_in=0;
							calculate_turn_profile(Direction_next,out_node.direction,out_node.direction_buf1,out_node.direction_buf2,&Turn_speed,&correction_time);
                        }
                        VerticalCost=compute_step_cost_diff(dis_cost_in,0,Turn_speed,g_dijkstra_parameter.max_velocity,g_dijkstra_parameter.acceleration,correction_time);
						if((wall.column[out_node.x-1] & (1 << out_node.y))==0 && Dijkstra.column_count[out_node.y][out_node.x-1]>Dijkstra.column_count[out_node.y][out_node.x]+VerticalCost){
							Dijkstra.column_count[out_node.y][out_node.x-1]=Dijkstra.column_count[out_node.y][out_node.x]+VerticalCost;
                            Dijkstra.column_direction[out_node.y][out_node.x-1]=out_node.direction;
							in_node = (DijkstraNode){	
							.x = out_node.x-1, .y = out_node.y, .matrix = COLUMN,
							.direction = Direction_next, .direction_buf1 = out_node.direction, .direction_buf2 = out_node.direction_buf1,
							.dis_cost = dis_cost_in, .total_cost = Dijkstra.column_count[out_node.y][out_node.x-1]
							};
							pushHeap(&heap, in_node);
						}
					}
					if (out_node.y <= MAZE_SQUARE_NUM-2) {
						// 北西向きの計算(左上)
						Direction_next = SLANT_NORTH_WEST;
						if(out_node.direction==Direction_next){
							dis_cost_in=out_node.dis_cost+DISCOUNTCOST_D;
							if(dis_cost_in>=D_NUM_MAX){dis_cost_in=D_NUM_MAX-1;}
						}else{
                            dis_cost_in=0;
							calculate_turn_profile(Direction_next,out_node.direction,out_node.direction_buf1,out_node.direction_buf2,&Turn_speed,&correction_time);
                        }
                        DiagonalCost=compute_step_cost_diff(dis_cost_in,1,Turn_speed,g_dijkstra_parameter.max_velocity,g_dijkstra_parameter.acceleration,correction_time);
						if((wall.row[out_node.y] & (1 << out_node.x))==0 && Dijkstra.row_count[out_node.x][out_node.y]>Dijkstra.column_count[out_node.y][out_node.x]+DiagonalCost){
							Dijkstra.row_count[out_node.x][out_node.y]=Dijkstra.column_count[out_node.y][out_node.x]+DiagonalCost;
                            Dijkstra.row_direction[out_node.x][out_node.y]=out_node.direction;
							in_node = (DijkstraNode){	
							.x = out_node.x, .y = out_node.y, .matrix = ROW,
							.direction = Direction_next, .direction_buf1 = out_node.direction, .direction_buf2 = out_node.direction_buf1,
							.dis_cost = dis_cost_in, .total_cost = Dijkstra.row_count[out_node.x][out_node.y]
							};
							pushHeap(&heap, in_node);
						}
						// 北東向きの計算(右上)
						Direction_next = SLANT_NORTH_EAST;
						if(out_node.direction==Direction_next){
							dis_cost_in=out_node.dis_cost+DISCOUNTCOST_D;
							if(dis_cost_in>=D_NUM_MAX){dis_cost_in=D_NUM_MAX-1;}
						}else{
                            dis_cost_in=0;
							calculate_turn_profile(Direction_next,out_node.direction,out_node.direction_buf1,out_node.direction_buf2,&Turn_speed,&correction_time);
                        }
                        DiagonalCost=compute_step_cost_diff(dis_cost_in,1,Turn_speed,g_dijkstra_parameter.max_velocity,g_dijkstra_parameter.acceleration,correction_time);
						if((wall.row[out_node.y] & (1 << (out_node.x+1)))==0 && Dijkstra.row_count[out_node.x+1][out_node.y]>Dijkstra.column_count[out_node.y][out_node.x]+DiagonalCost){
							Dijkstra.row_count[out_node.x+1][out_node.y]=Dijkstra.column_count[out_node.y][out_node.x]+DiagonalCost;
                            Dijkstra.row_direction[out_node.x+1][out_node.y]=out_node.direction;
							in_node = (DijkstraNode){	
							.x = out_node.x+1, .y = out_node.y, .matrix = ROW,
							.direction = Direction_next, .direction_buf1 = out_node.direction, .direction_buf2 = out_node.direction_buf1,
							.dis_cost = dis_cost_in, .total_cost = Dijkstra.row_count[out_node.x+1][out_node.y]
							};
							pushHeap(&heap, in_node);
						}
					}
					if (out_node.y >= 1) {
						// 南西向きの計算(左下)
						Direction_next = SLANT_SOUTH_WEST;
						if(out_node.direction==Direction_next){
							dis_cost_in=out_node.dis_cost+DISCOUNTCOST_D;
							if(dis_cost_in>=D_NUM_MAX){dis_cost_in=D_NUM_MAX-1;}
						}else{
                            dis_cost_in=0;
							calculate_turn_profile(Direction_next,out_node.direction,out_node.direction_buf1,out_node.direction_buf2,&Turn_speed,&correction_time);
                        }
                        DiagonalCost=compute_step_cost_diff(dis_cost_in,1,Turn_speed,g_dijkstra_parameter.max_velocity,g_dijkstra_parameter.acceleration,correction_time);
						if((wall.row[out_node.y-1] & (1 << out_node.x))==0 && Dijkstra.row_count[out_node.x][out_node.y-1]>Dijkstra.column_count[out_node.y][out_node.x]+DiagonalCost){
							Dijkstra.row_count[out_node.x][out_node.y-1]=Dijkstra.column_count[out_node.y][out_node.x]+DiagonalCost;
                            Dijkstra.row_direction[out_node.x][out_node.y-1]=out_node.direction;
							in_node = (DijkstraNode){	
							.x = out_node.x, .y = out_node.y-1, .matrix = ROW,
							.direction = Direction_next, .direction_buf1 = out_node.direction, .direction_buf2 = out_node.direction_buf1,
							.dis_cost = dis_cost_in, .total_cost = Dijkstra.row_count[out_node.x][out_node.y-1]
							};
							pushHeap(&heap, in_node);
						}
						// 南東向きの計算(右下)
						Direction_next = SLANT_SOUTH_EAST;
						if(out_node.direction==Direction_next){
							dis_cost_in=out_node.dis_cost+DISCOUNTCOST_D;
							if(dis_cost_in>=D_NUM_MAX){dis_cost_in=D_NUM_MAX-1;}
						}else{
                            dis_cost_in=0;
							calculate_turn_profile(Direction_next,out_node.direction,out_node.direction_buf1,out_node.direction_buf2,&Turn_speed,&correction_time);
                        }
                        DiagonalCost=compute_step_cost_diff(dis_cost_in,1,Turn_speed,g_dijkstra_parameter.max_velocity,g_dijkstra_parameter.acceleration,correction_time);
						if((wall.row[out_node.y-1] & (1 << (out_node.x+1)))==0 && Dijkstra.row_count[out_node.x+1][out_node.y-1]>Dijkstra.column_count[out_node.y][out_node.x]+DiagonalCost){
							Dijkstra.row_count[out_node.x+1][out_node.y-1]=Dijkstra.column_count[out_node.y][out_node.x]+DiagonalCost;
                            Dijkstra.row_direction[out_node.x+1][out_node.y-1]=out_node.direction;
							in_node = (DijkstraNode){	
							.x = out_node.x+1, .y = out_node.y-1, .matrix = ROW,
							.direction = Direction_next, .direction_buf1 = out_node.direction, .direction_buf2 = out_node.direction_buf1,
							.dis_cost = dis_cost_in, .total_cost = Dijkstra.row_count[out_node.x+1][out_node.y-1]
							};
							pushHeap(&heap, in_node);
						}
					}

				}

		}



}





void route_Dijkstra(void){
	STACK_T stack_x;
	STACK_T stack_y;
	STACK_T stack_matrix;//行列
	//STACK_T stack_x_unknow;
	//STACK_T stack_y_unknow;
	//STACK_T stack_matrix_unknow;//行列
	initStack_walk(&stack_x);
	initStack_walk(&stack_y);
	initStack_walk(&stack_matrix);
	initStack_walk(&g_Goal_x);
	initStack_walk(&g_Goal_y);

	pushStack_walk(&stack_x,0);pushStack_walk(&stack_y,0);
	pushStack_walk(&stack_matrix,ROW);

	unsigned short front_count, right_count, back_count, left_count;

	//_Bool front_wall;
	//_Bool right_wall;
	//_Bool left_wall;

	int xd = 0;
	int yd = 0;
	int direction_d = 1;


	while (1) {
//		if (mode_safty == 1) {break;}
		update_coordinate(&xd,&yd,direction_d);

		if(GOAL_ALL_D){
					break;
		}


		search_AroundDijkstraCount(&front_count,&right_count,&back_count,&left_count,xd,yd,direction_d);
//      get_wall(xd,yd,direction_d,&front_wall,&right_wall,&left_wall);
// 		if (front_wall) {front_count = MAX_WALKCOUNT_DIJKSTRA;}
// 		if (right_wall) {right_count = MAX_WALKCOUNT_DIJKSTRA;}
// 		if (left_wall) {left_count = MAX_WALKCOUNT_DIJKSTRA;}

		if (front_count==MAX_WALKCOUNT_DIJKSTRA && right_count==MAX_WALKCOUNT_DIJKSTRA && left_count==MAX_WALKCOUNT_DIJKSTRA && back_count==MAX_WALKCOUNT_DIJKSTRA){
			// 迷路破損のため停止(一時停止後に周辺の地図情報を初期化して再探索に変更予定)
			break;
		}
		if ( error_mode >= 1 ){
			break;
		}
		if (front_count <= right_count && front_count <= left_count && front_count <= back_count){
			// 直進
			switch (direction_d) {		//
			case 1:
				pushStack_walk(&stack_x,xd);
				pushStack_walk(&stack_y,yd);
				pushStack_walk(&stack_matrix,ROW);
				break;
			case 2:
				pushStack_walk(&stack_x,xd);
				pushStack_walk(&stack_y,yd);
				pushStack_walk(&stack_matrix,COLUMN);
				break;
			case 3:
				pushStack_walk(&stack_x,xd);
				pushStack_walk(&stack_y,yd-1);
				pushStack_walk(&stack_matrix,ROW);
				break;
			case 4:
				pushStack_walk(&stack_x,xd-1);
				pushStack_walk(&stack_y,yd);
				pushStack_walk(&stack_matrix,COLUMN);
				break;
			}

		}

		if(right_count < front_count && right_count <= left_count && right_count <= back_count){
			// 右旋回
			switch (direction_d) {		//
			case 1:
				pushStack_walk(&stack_x,xd);
				pushStack_walk(&stack_y,yd);
				pushStack_walk(&stack_matrix,COLUMN);
				break;
			case 2:
				pushStack_walk(&stack_x,xd);
				pushStack_walk(&stack_y,yd-1);
				pushStack_walk(&stack_matrix,ROW);
				break;
			case 3:
				pushStack_walk(&stack_x,xd-1);
				pushStack_walk(&stack_y,yd);
				pushStack_walk(&stack_matrix,COLUMN);
				break;
			case 4:
				pushStack_walk(&stack_x,xd);
				pushStack_walk(&stack_y,yd);
				pushStack_walk(&stack_matrix,ROW);
				break;
			}
			direction_d++;
		}
		if(left_count < front_count && left_count < right_count && left_count <= back_count){
			// 左旋回
			switch (direction_d) {		//
			case 1:
				pushStack_walk(&stack_x,xd-1);
				pushStack_walk(&stack_y,yd);
				pushStack_walk(&stack_matrix,COLUMN);
				break;
			case 2:
				pushStack_walk(&stack_x,xd);
				pushStack_walk(&stack_y,yd);
				pushStack_walk(&stack_matrix,ROW);
				break;
			case 3:
				pushStack_walk(&stack_x,xd);
				pushStack_walk(&stack_y,yd);
				pushStack_walk(&stack_matrix,COLUMN);
				break;
			case 4:
				pushStack_walk(&stack_x,xd);
				pushStack_walk(&stack_y,yd-1);
				pushStack_walk(&stack_matrix,ROW);
				break;
			}
			direction_d--;
		}

		if (direction_d == 5) {
			direction_d = 1;
		}
		if (direction_d == 6) {
			direction_d = 2;
		}
		if (direction_d == 0) {
			direction_d = 4;
		}
		if (direction_d == -1) {
			direction_d = 3;
		}

	}

	unsigned short Xcoordinate,Ycoordinate,Row_or_Column;
	while (1) {

			Xcoordinate = popStack_walk(&stack_x);
			Ycoordinate = popStack_walk(&stack_y);
			Row_or_Column = popStack_walk(&stack_matrix);
			//printf("x %d,y %d,C(0)R(1) %d\n",Xcoordinate,Ycoordinate,Row_or_Column);
			//printf("x head %d tail %d\n y head %d tail %d\n",stack_x.head,stack_x.tail,stack_y.head,stack_y.tail);
			if (Xcoordinate == MAX_WALKCOUNT_DIJKSTRA || Ycoordinate == MAX_WALKCOUNT_DIJKSTRA || error_mode >= 1) {
				//printf("stack_end\n");
				break;
			}
			if(Row_or_Column==ROW && ((wall.row_look[Ycoordinate] & (1 << Xcoordinate)) == 0)){
				//pushStack_walk(&stack_x_unknow,Xcoordinate);
				//pushStack_walk(&stack_y_unknow,Ycoordinate);
				//pushStack_walk(&stack_matrix_unknow,Row_or_Column);
				walk_count[Xcoordinate][Ycoordinate] = 0;
				walk_count[Xcoordinate][Ycoordinate + 1] = 0;
				pushStack_walk(&g_Goal_x,Xcoordinate);pushStack_walk(&g_Goal_y,Ycoordinate);
				pushStack_walk(&g_Goal_x,Xcoordinate);pushStack_walk(&g_Goal_y,Ycoordinate+1);
			}
			if(Row_or_Column==COLUMN && ((wall.column_look[Xcoordinate] & (1 << Ycoordinate)) == 0)){
				//pushStack_walk(&stack_x_unknow,Xcoordinate);
				//pushStack_walk(&stack_y_unknow,Ycoordinate);
				//pushStack_walk(&stack_matrix_unknow,Row_or_Column);
				walk_count[Xcoordinate][Ycoordinate] = 0;
				walk_count[Xcoordinate + 1][Ycoordinate] = 0;
				pushStack_walk(&g_Goal_x,Xcoordinate);pushStack_walk(&g_Goal_y,Ycoordinate);
				pushStack_walk(&g_Goal_x,Xcoordinate+1);pushStack_walk(&g_Goal_y,Ycoordinate);
			}
	}


}




void create_StepCountMap_unknown(void){
	//ここから歩数マップを作る．*************************************
	STACK_T stack_x;
	STACK_T stack_y;
	unsigned short goalX,goalY;
	for(uint8_t xx = 0;xx <= MAZE_SQUARE_NUM-1;xx++){
		for(uint8_t yy = 0;yy <= MAZE_SQUARE_NUM-1;yy++){
			walk_count[xx][yy] = MAX_WALKCOUNT;
		}
	}

	initStack_walk(&stack_x);
	initStack_walk(&stack_y);

	while (1) {

			goalX = popStack_walk(&g_Goal_x);
			goalY = popStack_walk(&g_Goal_y);
			//printf("x %d,y %d,C(0)R(1) %d\n",Xcoordinate,Ycoordinate,Row_or_Column);
			//printf("x head %d tail %d\n y head %d tail %d\n",stack_x.head,stack_x.tail,stack_y.head,stack_y.tail);
			if (goalX == MAX_WALKCOUNT_DIJKSTRA || goalY == MAX_WALKCOUNT_DIJKSTRA || error_mode >= 1) {
				//printf("stack_end\n");
				break;
			}
			walk_count[goalX][goalY] = 0;
			pushStack_walk(&stack_x,goalX);pushStack_walk(&stack_y,goalY);
	}
	if(stack_x.tail == stack_x.head){
		walk_count[0][0] = 0;
		pushStack_walk(&stack_x,0);pushStack_walk(&stack_y,0);
		if (Dijkstra_maker_flag>=1){
			Dijkstra_maker_flag=2;
		}else{
			Dijkstra_maker_flag=1;
		}
	}else{
		Dijkstra_maker_flag=0;
	}
	//printf("(%d,%d),(%d,%d),(%d,%d),(%d,%d)\n",stack_x.data[0],stack_y.data[0],stack_x.data[1],stack_y.data[1],stack_x.data[2],stack_y.data[2],stack_x.data[3],stack_y.data[3]);
	//printf("x head %d tail %d\n y head %d tail %d\n",stack_x.head,stack_x.tail,stack_y.head,stack_y.tail);
	unsigned short Xcoordinate,Ycoordinate;
	uint32_t wall_north=1,wall_south=1,wall_east=1,wall_west=1;
	while (1) {

		Xcoordinate = popStack_walk(&stack_x);
		Ycoordinate = popStack_walk(&stack_y);
		//printf("x %d,y %d\n",Xcoordinate,Ycoordinate);
		//printf("x head %d tail %d\n y head %d tail %d\n",stack_x.head,stack_x.tail,stack_y.head,stack_y.tail);
		if (Xcoordinate == MAX_WALKCOUNT_DIJKSTRA || Ycoordinate == MAX_WALKCOUNT_DIJKSTRA || error_mode >= 1) {
			//printf("stack_end\n");
			break;
		}

		if (Ycoordinate <= MAZE_SQUARE_NUM-2) {
			wall_north = wall.row[Ycoordinate] & (1 << Xcoordinate);
		}
		if (Ycoordinate >= 1) {
			wall_south = wall.row[Ycoordinate - 1] & (1 << Xcoordinate);
		}
		if (Xcoordinate <= MAZE_SQUARE_NUM-2) {
			wall_east = wall.column[Xcoordinate] & (1 << Ycoordinate);
		}
		if (Xcoordinate >= 1) {
			wall_west = wall.column[Xcoordinate - 1] & (1 << Ycoordinate);
		}

		if (walk_count[Xcoordinate][Ycoordinate + 1] == MAX_WALKCOUNT && Ycoordinate != MAZE_SQUARE_NUM-1 && wall_north == 0) {
			walk_count[Xcoordinate][Ycoordinate + 1] = walk_count[Xcoordinate][Ycoordinate] + 1;
			pushStack_walk(&stack_x,Xcoordinate);
			pushStack_walk(&stack_y,Ycoordinate + 1);
		}
		if (walk_count[Xcoordinate][Ycoordinate - 1] == MAX_WALKCOUNT && Ycoordinate != 0 && wall_south == 0) {
			walk_count[Xcoordinate][Ycoordinate - 1] = walk_count[Xcoordinate][Ycoordinate] + 1;
			pushStack_walk(&stack_x,Xcoordinate);
			pushStack_walk(&stack_y,Ycoordinate - 1);
		}
		if (walk_count[Xcoordinate + 1][Ycoordinate] == MAX_WALKCOUNT && Xcoordinate != MAZE_SQUARE_NUM-1 && wall_east == 0) {
			walk_count[Xcoordinate + 1][Ycoordinate] = walk_count[Xcoordinate][Ycoordinate] + 1;
			pushStack_walk(&stack_x,Xcoordinate + 1);
			pushStack_walk(&stack_y,Ycoordinate);
		}
		if (walk_count[Xcoordinate - 1][Ycoordinate] == MAX_WALKCOUNT && Xcoordinate != 0 && wall_west == 0) {
			walk_count[Xcoordinate - 1][Ycoordinate] = walk_count[Xcoordinate][Ycoordinate] + 1;
			pushStack_walk(&stack_x,Xcoordinate - 1);
			pushStack_walk(&stack_y,Ycoordinate);
		}

		}

}




void create_StepCountMap_queue(void){

	//ここから歩数マップを作る．*************************************
	STACK_T stack_x;
	STACK_T stack_y;
	for(uint8_t xx = 0;xx <= MAZE_SQUARE_NUM-1;xx++){
		for(uint8_t yy = 0;yy <= MAZE_SQUARE_NUM-1;yy++){
			walk_count[xx][yy] = MAX_WALKCOUNT;
		}
	}
	initStack_walk(&stack_x);
	initStack_walk(&stack_y);

	for(int i=0;i<=GOAL_SIZE-1;i++){
		for(int j=0;j<=GOAL_SIZE-1;j++){
			walk_count[GOAL_X+i][GOAL_Y+j] = 0;
			pushStack_walk(&stack_x,GOAL_X+i);pushStack_walk(&stack_y,GOAL_Y+j);
		}
	}

	//printf("(%d,%d),(%d,%d),(%d,%d),(%d,%d)\n",stack_x.data[0],stack_y.data[0],stack_x.data[1],stack_y.data[1],stack_x.data[2],stack_y.data[2],stack_x.data[3],stack_y.data[3]);
	//printf("x head %d tail %d\n y head %d tail %d\n",stack_x.head,stack_x.tail,stack_y.head,stack_y.tail);
	unsigned short Xcoordinate,Ycoordinate;
	uint32_t wall_north=1,wall_south=1,wall_east=1,wall_west=1;
	while (1) {

		Xcoordinate = popStack_walk(&stack_x);
		Ycoordinate = popStack_walk(&stack_y);
		//printf("x %d,y %d\n",Xcoordinate,Ycoordinate);
		//printf("x head %d tail %d\n y head %d tail %d\n",stack_x.head,stack_x.tail,stack_y.head,stack_y.tail);
		if (Xcoordinate == MAX_WALKCOUNT_DIJKSTRA || Ycoordinate == MAX_WALKCOUNT_DIJKSTRA || error_mode >= 1) {
			//printf("stack_end\n");
			break;
		}

		if (Ycoordinate <= MAZE_SQUARE_NUM-2) {
			wall_north = wall.row[Ycoordinate] & (1 << Xcoordinate);
		}
		if (Ycoordinate >= 1) {
			wall_south = wall.row[Ycoordinate - 1] & (1 << Xcoordinate);
		}
		if (Xcoordinate <= MAZE_SQUARE_NUM-2) {
			wall_east = wall.column[Xcoordinate] & (1 << Ycoordinate);
		}
		if (Xcoordinate >= 1) {
			wall_west = wall.column[Xcoordinate - 1] & (1 << Ycoordinate);
		}

		if (walk_count[Xcoordinate][Ycoordinate + 1] == MAX_WALKCOUNT && Ycoordinate != MAZE_SQUARE_NUM-1 && wall_north == 0) {
			walk_count[Xcoordinate][Ycoordinate + 1] = walk_count[Xcoordinate][Ycoordinate] + 1;
			pushStack_walk(&stack_x,Xcoordinate);
			pushStack_walk(&stack_y,Ycoordinate + 1);
		}
		if (walk_count[Xcoordinate][Ycoordinate - 1] == MAX_WALKCOUNT && Ycoordinate != 0 && wall_south == 0) {
			walk_count[Xcoordinate][Ycoordinate - 1] = walk_count[Xcoordinate][Ycoordinate] + 1;
			pushStack_walk(&stack_x,Xcoordinate);
			pushStack_walk(&stack_y,Ycoordinate - 1);
		}
		if (walk_count[Xcoordinate + 1][Ycoordinate] == MAX_WALKCOUNT && Xcoordinate != MAZE_SQUARE_NUM-1 && wall_east == 0) {
			walk_count[Xcoordinate + 1][Ycoordinate] = walk_count[Xcoordinate][Ycoordinate] + 1;
			pushStack_walk(&stack_x,Xcoordinate + 1);
			pushStack_walk(&stack_y,Ycoordinate);
		}
		if (walk_count[Xcoordinate - 1][Ycoordinate] == MAX_WALKCOUNT && Xcoordinate != 0 && wall_west == 0) {
			walk_count[Xcoordinate - 1][Ycoordinate] = walk_count[Xcoordinate][Ycoordinate] + 1;
			pushStack_walk(&stack_x,Xcoordinate - 1);
			pushStack_walk(&stack_y,Ycoordinate);
		}


		}

}

void create_StepCountMapBack_queue(void){
	//ここから歩数マップを作る．*************************************
	STACK_T stack_x;
	STACK_T stack_y;
	for(uint8_t xx = 0;xx <= MAZE_SQUARE_NUM-1;xx++){
		for(uint8_t yy = 0;yy <= MAZE_SQUARE_NUM-1;yy++){
			walk_count[xx][yy] = MAX_WALKCOUNT;
		}
	}
	initStack_walk(&stack_x);
	initStack_walk(&stack_y);


	walk_count[0][0] = 0;
	pushStack_walk(&stack_x,0);pushStack_walk(&stack_y,0);
	//printf("(%d,%d),(%d,%d),(%d,%d),(%d,%d)\n",stack_x.data[0],stack_y.data[0],stack_x.data[1],stack_y.data[1],stack_x.data[2],stack_y.data[2],stack_x.data[3],stack_y.data[3]);
	//printf("x head %d tail %d\n y head %d tail %d\n",stack_x.head,stack_x.tail,stack_y.head,stack_y.tail);
	//unsigned short coordinate;
	unsigned short Xcoordinate,Ycoordinate;
	uint32_t wall_north=1,wall_south=1,wall_east=1,wall_west=1;
	while (1) {

		Xcoordinate = popStack_walk(&stack_x);
		Ycoordinate = popStack_walk(&stack_y);
		//printf("x %d,y %d\n",Xcoordinate,Ycoordinate);
		//printf("x head %d tail %d\n y head %d tail %d\n",stack_x.head,stack_x.tail,stack_y.head,stack_y.tail);
		if (Xcoordinate == MAX_WALKCOUNT_DIJKSTRA || Ycoordinate == MAX_WALKCOUNT_DIJKSTRA || error_mode >= 1) {
			//printf("stack_end\n");
			break;
		}


		if (Ycoordinate <= MAZE_SQUARE_NUM-2) {
			wall_north = wall.row[Ycoordinate] & (1 << Xcoordinate);
		}
		if (Ycoordinate >= 1) {
			wall_south = wall.row[Ycoordinate - 1] & (1 << Xcoordinate);
		}
		if (Xcoordinate <= MAZE_SQUARE_NUM-2) {
			wall_east = wall.column[Xcoordinate] & (1 << Ycoordinate);
		}
		if (Xcoordinate >= 1) {
			wall_west = wall.column[Xcoordinate - 1] & (1 << Ycoordinate);
		}

		if (walk_count[Xcoordinate][Ycoordinate + 1] == MAX_WALKCOUNT && Ycoordinate != MAZE_SQUARE_NUM-1 && wall_north == 0) {
			walk_count[Xcoordinate][Ycoordinate + 1] = walk_count[Xcoordinate][Ycoordinate] + 1;
			pushStack_walk(&stack_x,Xcoordinate);
			pushStack_walk(&stack_y,Ycoordinate + 1);
		}
		if (walk_count[Xcoordinate][Ycoordinate - 1] == MAX_WALKCOUNT && Ycoordinate != 0 && wall_south == 0) {
			walk_count[Xcoordinate][Ycoordinate - 1] = walk_count[Xcoordinate][Ycoordinate] + 1;
			pushStack_walk(&stack_x,Xcoordinate);
			pushStack_walk(&stack_y,Ycoordinate - 1);
		}
		if (walk_count[Xcoordinate + 1][Ycoordinate] == MAX_WALKCOUNT && Xcoordinate != MAZE_SQUARE_NUM-1 && wall_east == 0) {
			walk_count[Xcoordinate + 1][Ycoordinate] = walk_count[Xcoordinate][Ycoordinate] + 1;
			pushStack_walk(&stack_x,Xcoordinate + 1);
			pushStack_walk(&stack_y,Ycoordinate);
		}
		if (walk_count[Xcoordinate - 1][Ycoordinate] == MAX_WALKCOUNT && Xcoordinate != 0 && wall_west == 0) {
			walk_count[Xcoordinate - 1][Ycoordinate] = walk_count[Xcoordinate][Ycoordinate] + 1;
			pushStack_walk(&stack_x,Xcoordinate - 1);
			pushStack_walk(&stack_y,Ycoordinate);
		}

		}

}


void initStack_walk(STACK_T *stack){
//	for(int i=0;i<=MAX_QUEUE_NUM-1;i++){
//		stack->data[i] = 0;
//	}
    /* スタックを空に設定 */
	stack->head = 0;
    stack->tail = 0;
}



void pushStack_walk(STACK_T *stack, unsigned short input){

    /* データをデータの最後尾の１つ後ろに格納 */
    stack->data[stack->tail] = input;

    /* データの最後尾を１つ後ろに移動 */
    stack->tail = stack->tail + 1;

    /* 巡回シフト */
    if(stack->tail == MAX_QUEUE_NUM) stack->tail = 0;

    /* スタックが満杯なら何もせず関数終了 */
    if(stack->tail == stack->head ){
    	printf("stack_full\n");
        return;
    }
}


unsigned short popStack_walk(STACK_T *stack){
    unsigned short ret = 0;

    /* スタックが空なら何もせずに関数終了 */
    if(stack->tail == stack->head){
    	//printf("stack_empty\n");
        return MAX_WALKCOUNT_DIJKSTRA;
    }

    /* データの最前列からデータを取得 */
    ret = stack->data[stack->head];

    /* データの最前列を１つ前にずらす */
    stack->head = stack->head + 1;

    /* 巡回シフト */
    if(stack->head == MAX_QUEUE_NUM) stack->head = 0;

    /* 取得したデータを返却 */
    return ret;
}



void initHeap(MinHeap* heap) {
    heap->size = 0;
}

_Bool isHeapEmpty(MinHeap* heap) {
    return heap->size == 0;
}

void pushHeap(MinHeap* heap, DijkstraNode node) {
    int i = heap->size++;
    while (i > 0 && heap->data[(i - 1) / 2].total_cost > node.total_cost) {
        heap->data[i] = heap->data[(i - 1) / 2];
        i = (i - 1) / 2;
    }
    heap->data[i] = node;
}


DijkstraNode popHeap(MinHeap* heap) {
    DijkstraNode top = heap->data[0];
    DijkstraNode last = heap->data[--heap->size];
    int i = 0;
    while (2*i + 1 < heap->size) {
        int child = 2*i + 1;
        if (child + 1 < heap->size && heap->data[child + 1].total_cost < heap->data[child].total_cost)
            child++;
        if (heap->data[child].total_cost >= last.total_cost) break;
        heap->data[i] = heap->data[child];
        i = child;
    }
    heap->data[i] = last;
    return top;
}








void maze_maker(int direction, int front_SEN, int left_SEN, int right_SEN,
		int x, int y) {}
void maze_maker2(int direction, int front_SEN, int left_SEN, int right_SEN,
		int x, int y) {}
void maze_makerhosuu(int direction, int x, int y) {}

void maze_makerRun(int direction, int front_SEN, int left_SEN, int right_SEN,
		int x, int y) {}

void maze_makeronly(int direction, int front_SEN, int left_SEN, int right_SEN,
		int x, int y) {}

void maze_makerback(int direction, int front_SEN, int left_SEN, int right_SEN,
		int x, int y) {}


void maze_display(WALL *wall_tmp) {

	int tt = MAZE_SQUARE_NUM-2;
	int ss = 0;

	for(int i=0;i<=MAZE_SQUARE_NUM-1;i++){
		printf("+-----");
	}
	printf("+\n");
	for (tt = MAZE_SQUARE_NUM-2;tt >= -1;tt--){

		printf("|%5d", walk_count[0][tt + 1]);
		for(ss = 0;ss < MAZE_SQUARE_NUM-1;ss++){
			if ((wall_tmp->column[ss] & (1 << (tt + 1))) == (1 << (tt + 1))){
				printf("|%5d", walk_count[ss + 1][tt + 1]);
			}else{
				printf(" %5d", walk_count[ss + 1][tt + 1]);
			}
		}
		printf("|\n");
		if (tt <= -1) {
			break;
		}
		for(ss = 0;ss <= MAZE_SQUARE_NUM-1;ss++){
			if ((wall_tmp->row[tt] & (1 << ss)) == (1 << ss)){
				printf("+-----");
			}else{
				printf("+     ");
			}
		}

		printf("+\n");

	}

	for(int i=0;i<=MAZE_SQUARE_NUM-1;i++){
		printf("+-----");
	}
	printf("+\n\n");

	for(int i=0;i<=MAZE_SQUARE_NUM-1;i++){
		printf("+-----");
	}
	printf("+\n");
	for (tt = MAZE_SQUARE_NUM-2;tt >= -1;tt--){
		printf("|%5d", walk_count[0][tt + 1]);
		for(ss = 0;ss < MAZE_SQUARE_NUM-1;ss++){
			if ((wall_tmp->column_look[ss] & (1 << (tt + 1))) == (1 << (tt + 1))){
				printf("|%5d", walk_count[ss + 1][tt + 1]);
			}else{
				printf(" %5d", walk_count[ss + 1][tt + 1]);
			}
		}
		printf("|\n");
		if (tt <= -1) {
			break;
		}
		for(ss = 0;ss <= MAZE_SQUARE_NUM-1;ss++){
			if ((wall_tmp->row_look[tt] & (1 << ss)) == (1 << ss)){
				printf("+-----");
			}else{
				printf("+     ");
			}
		}
		printf("+\n");
	}

	for(int i=0;i<=MAZE_SQUARE_NUM-1;i++){
		printf("+-----");
	}
	printf("+\n\n");

}






void maze_display_Dijkstra(void) {

	int tt = MAZE_SQUARE_NUM-2;
	int ss = 0;

	for(int i=0;i<=MAZE_SQUARE_NUM-1;i++){
		printf("+-----");
	}
	printf("+\n");
	for (tt = MAZE_SQUARE_NUM-2;tt >= -1;tt--){

		printf("|   ");//walk_count[tt + 1]
		for(ss = 0;ss < MAZE_SQUARE_NUM-1;ss++){
			if ((wall.column[ss] & (1 << (tt + 1))) == (1 << (tt + 1))){
				printf("  |   ");
			}else{
				printf("%5d ", Dijkstra.column_count[(tt + 1)][ss]);
			}
		}
		printf("   |\n");
		if (tt <= -1) {
			break;
		}
		for(ss = 0;ss <= MAZE_SQUARE_NUM-1;ss++){
			if ((wall.row[tt] & (1 << ss)) == (1 << ss)){
				printf("+-----");
			}else{
				printf("+%5d",Dijkstra.row_count[ss][tt]);
			}
		}

		printf("+\n");

	}

	for(int i=0;i<=MAZE_SQUARE_NUM-1;i++){
		printf("+-----");
	}
	printf("+\n\n");

	for(int i=0;i<=MAZE_SQUARE_NUM-1;i++){
		printf("+-----");
	}
	printf("+\n");
	for (tt = MAZE_SQUARE_NUM-2;tt >= -1;tt--){

		printf("|   ");//walk_count[tt + 1]
		for(ss = 0;ss < MAZE_SQUARE_NUM-1;ss++){

				printf("%5d ", Dijkstra.column_count[(tt + 1)][ss]);

		}
		printf("   |\n");
		if (tt <= -1) {
			break;
		}
		for(ss = 0;ss <= MAZE_SQUARE_NUM-1;ss++){
				printf("+%5d",Dijkstra.row_count[ss][tt]);
		}

		printf("+\n");

	}

	for(int i=0;i<=MAZE_SQUARE_NUM-1;i++){
		printf("+-----");
	}
	printf("+\n\n");




}
