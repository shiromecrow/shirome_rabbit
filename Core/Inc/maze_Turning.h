/*
 * maze_Turning.h
 *
 *  Created on: Jan 9, 2024
 *      Author: sf199
 */

#ifndef INC_MAZE_TURNING_H_
#define INC_MAZE_TURNING_H_

#include "turning_parameter.h"

#define OFFSET_CONTROL_IN_SLALOM 0//大回りのオフセットでの壁制御(入り)
#define OFFSET_CONTROL_OUT_SLALOM 0//大回りのオフセットでの壁制御(出る)
#define OFFSET_CONTROL_IN 0//大回りのオフセットでの壁制御(入り)
#define OFFSET_CONTROL_OUT 1//大回りのオフセットでの壁制御(出る)
#define OFFSET_CONTROL_IN_SLANT 0//3で制御あり
#define OFFSET_CONTROL_OUT_SLANT 0//3で制御あり

void test_mollifier_slalomR(parameter);

void backTurn_hitWall(float,float,_Bool,_Bool,_Bool);
void backTurn_controlWall(float,float,_Bool,_Bool,_Bool);

void slalomR(parameter,char,char,char,float);
void slalomL(parameter,char,char,char,float);
void turn90R(parameter,char,char,float);
void turn90L(parameter,char,char,float);
void turn180R(parameter,char,char,float);
void turn180L(parameter,char,char,float);
void turn45inR(parameter,char,char,float);
void turn45inL(parameter,char,char,float);
void turn135inR(parameter,char,char,float);
void turn135inL(parameter,char,char,float);
void turn45outR(parameter,char,char,float);
void turn45outL(parameter,char,char,float);
void turn135outR(parameter,char,char,float);
void turn135outL(parameter,char,char,float);
void V90R(parameter,char,char,float);
void V90L(parameter,char,char,float);



void testturning(parameter_speed,int,char,char,float,char);

#endif /* INC_MAZE_TURNING_H_ */
