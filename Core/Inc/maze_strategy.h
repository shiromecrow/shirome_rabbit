/*
 * maze_strategy.h
 *
 *  Created on: Jan 9, 2024
 *      Author: sf199
 */

#ifndef INC_MAZE_STRATEGY_H_
#define INC_MAZE_STRATEGY_H_

#include <stdint.h>
#include "turning_parameter.h"

#define TURN_ON 1
#define TURN_OFF 0
#define FUN_ON 1
#define FUN_OFF 0
#define SLANT_ON 1
#define SLANT_OFF 0

#define MAZE_SECTION 90

#define PASS_NUM 501
#define SLANT_PASS_COUNT 500

extern int g_X, g_Y, g_Dir;/* 最短後の位置保存 */

void decision_kitiku(int,int,int,unsigned short,unsigned short,unsigned short,unsigned short);
void compress_kitiku(int *,int *,int *,int *);


void get_wallData_sensor(_Bool*,_Bool*,_Bool*);
void update_coordinate(int *,int *,int);
int update_direction(int, int);
void run_movement_continuity(int *,unsigned short,unsigned short,unsigned short,unsigned short,float,float,float,float, parameter_speed,_Bool ,_Bool ,_Bool);
void run_movement_suspension(int *,unsigned short,unsigned short,unsigned short,unsigned short,float,float,float,float, parameter_speed,_Bool ,_Bool ,_Bool,int,int,uint8_t,uint8_t);


void AdatiWayReturn(float,float,float,float, parameter_speed,int,uint8_t);

void pass_maker();
void pass_maker_Dijkstra();

void run_shortest(float,float,float,char,char,char,parameter_speed,float,char,char);
void AdatiWayReturnOnly(float,float,float,float, parameter_speed,int,uint8_t);
void goal_cheak();

#endif /* INC_MAZE_STRATEGY_H_ */
