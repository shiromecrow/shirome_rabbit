/*
 * maze_wall.h
 *
 *  Created on: Jan 9, 2024
 *      Author: sf199
 */

#ifndef INC_MAZE_WALL_H_
#define INC_MAZE_WALL_H_

#include "main.h"
#include "turning_parameter.h"

#define MAZE_SQUARE_NUM 32

#define MAX_QUEUE_NUM 1500
#define ROW 0
#define COLUMN 1

#define MAX_WALKCOUNT 65535
#define MAX_WALKCOUNT_DIJKSTRA 65535

#define VERTICALCOST 180
#define DIAGONALCOST 127
#define MIN_VERTICALCOST 12
#define MIN_DIAGONALCOST 10
#define DISCOUNTCOST_V 1//絶対1
#define DISCOUNTCOST_D 1//絶対1
// #define V_NUM_MAX 5
// #define D_NUM_MAX 5
#define V_NUM_MAX 20
#define D_NUM_MAX 20

#define SLANT_NORTH 0
#define SLANT_NORTH_EAST 1
#define SLANT_EAST 2
#define SLANT_SOUTH_EAST 3
#define SLANT_SOUTH 4
#define SLANT_SOUTH_WEST 5
#define SLANT_WEST 6
#define SLANT_NORTH_WEST 7


typedef struct{
	uint32_t row[MAZE_SQUARE_NUM-1];
	uint32_t column[MAZE_SQUARE_NUM-1];
	uint32_t row_look[MAZE_SQUARE_NUM-1];
	uint32_t column_look[MAZE_SQUARE_NUM-1];

}WALL;

typedef struct{
	uint16_t row_count[MAZE_SQUARE_NUM][MAZE_SQUARE_NUM-1];
	uint16_t column_count[MAZE_SQUARE_NUM][MAZE_SQUARE_NUM-1];
    uint16_t row_direction[MAZE_SQUARE_NUM][MAZE_SQUARE_NUM-1];
    uint16_t column_direction[MAZE_SQUARE_NUM][MAZE_SQUARE_NUM-1];
}DIJKSTRA;

typedef struct {
	int16_t turn90_corrtime;
	int16_t turn180_corrtime;
    int16_t turn135in_corrtime;
    int16_t turn135out_corrtime;
    int16_t V90_corrtime;
} parameter_speed_corrtime;


typedef struct {

    float max_velocity;           // 最高速度 [mm/s]
    float acceleration;           // 加速度 [mm/s^2]
    parameter_speed Turn_parameter;  // ターン・スラローム各種プロファイル
	parameter_speed_corrtime Turn_parameter_corrtime;  // ターン補正時間(事前計算で45ターンから補正)

} Dijkstra_parameter;



// スタック構造体
typedef struct{
	/* データの最前列 */
	int head;
    /* データの最後尾 */
    int tail;
    /* スタックされているデータ */
    int data[MAX_QUEUE_NUM];
} STACK_T;

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t matrix;
    uint16_t direction;
    uint16_t direction_buf1;
    uint16_t direction_buf2;
    uint16_t dis_cost;
    uint16_t total_cost;
} DijkstraNode;

typedef struct {
    DijkstraNode data[MAX_QUEUE_NUM];
    int size;
} MinHeap;


extern WALL wall;
extern WALL record;
extern WALL error_wall;
extern Dijkstra_parameter g_dijkstra_parameter;


extern char Dijkstra_maker_flag;


extern uint16_t walk_count[MAZE_SQUARE_NUM][MAZE_SQUARE_NUM]; //歩数いれる箱


void maze_out_matlab();
void maze_clear();
void update_wall(int,int,int,_Bool,_Bool,_Bool);
void get_wall(int,int,int,_Bool*,_Bool*,_Bool*);
void get_wall_look(int,int,int,_Bool*,_Bool*,_Bool*);

parameter_speed_corrtime convert_parameter_speed_to_corrtime(const parameter_speed *);

void search_AroundWalkCount(unsigned short *,unsigned short *,unsigned short *,unsigned short *,int,int,int);
void search_AroundDijkstraCount(unsigned short *,unsigned short *,unsigned short *,unsigned short *,int,int,int);



void route_Dijkstra();
void create_DijkstraMap();
void create_DijkstraMap3();
void create_StepCountMap_unknown();

void create_StepCountMap_queue();
void create_StepCountMapBack_queue();
void initStack_walk(STACK_T *);
void pushStack_walk(STACK_T *, unsigned short);
unsigned short popStack_walk(STACK_T *);


void maze_maker(int,int,int,int,int,int);
void maze_maker2(int,int,int,int,int,int);
void maze_makerhosuu(int,int,int);

void maze_makerRun(int,int,int,int,int,int);
void maze_makeronly(int,int,int,int,int,int);
void maze_makerback(int,int,int,int,int,int);

void maze_display(WALL *);
void maze_display_Dijkstra();

void initHeap(MinHeap* heap);
_Bool isHeapEmpty(MinHeap* heap);
void pushHeap(MinHeap* heap, DijkstraNode node);
DijkstraNode popHeap(MinHeap* heap);


#endif /* INC_MAZE_WALL_H_ */
