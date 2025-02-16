/*
 * define.h
 *
 *  Created on: Dec 23, 2023
 *      Author: sf199
 */

#ifndef INC_DEFINE_H_
#define INC_DEFINE_H_

#define ON 1
#define OFF 0
#define CONNECT 2

#define BATT_MAX 8.4


#define MAXMOTOR (3360-1)

#define SENSOR_RIGHT 4
#define SENSOR_LEFT 1
#define SENSOR_FRONT_LEFT 2
#define SENSOR_FRONT_RIGHT 3
#define SENSOR_FRONT_L 0
#define SENSOR_FRONT_R 5

#define EXPLORATION 0
#define SHORTEST 1

#define MAZE_OFFSET 19

#define FRONT_TO_CENTER_FRONT 12
#define BACK_TO_CENTER2 16.5
#define BACK_TO_CENTER 20
#define BACK_TO_CENTER_FRONT 12.5
#define BACK_TO_CENTER_BACK 28.5
#define TURN_CENTER (BACK_TO_CENTER_BACK-BACK_TO_CENTER_FRONT)
#define BACK_TO_CENTER_SLANT 42.5
#define BACK_TO_CENTER_FRONT_SLANT 34.5

#define FIRST_MOVE_R90 16
#define FIRST_MOVE_R45 16
#define FIRST_MOVE_R135 18

// 32は3
#define GOAL_SIZE 3

// #define GOAL_X 8
// #define GOAL_Y 7
// #define GOAL_ALL ((x == GOAL_X || x == GOAL_X + 1) && (y == GOAL_Y || y == GOAL_Y + 1))
// #define GOAL_ALL_D ((xd == GOAL_X || xd == GOAL_X + 1) && (yd == GOAL_Y || yd == GOAL_Y + 1))
// #define MAZE_TIMER 8

#define GOAL_X 6
#define GOAL_Y 6
#define GOAL_ALL ((x == GOAL_X || x == GOAL_X + 1 || x == GOAL_X + 2) && (y == GOAL_Y || y == GOAL_Y + 1 || y == GOAL_Y + 2))
#define GOAL_ALL_D ((xd == GOAL_X || xd == GOAL_X + 1 || xd == GOAL_X + 2) && (yd == GOAL_Y || yd == GOAL_Y + 1 || yd == GOAL_Y + 2))

#define MAZE_TIMER 7

#define F_PRESENCE 190
#define R_PRESENCE 500//180//900
#define L_PRESENCE 500//180//900

#endif /* INC_DEFINE_H_ */
