/*
 * cal_acceleration.h
 *
 *  Created on: 2023/01/16
 *      Author: sf199
 */

#ifndef INC_CAL_ACCELERATION_H_
#define INC_CAL_ACCELERATION_H_


typedef struct {
	float velocity;
	float acceleration;
	float displacement;

}TARGET;

typedef struct{
	float displacement;
	float start_velocity;
	float end_velocity;
	float count_velocity;
	float acceleration;
	float deceleration;

}TRAPEZOID;

typedef struct{
	float displacement;
	float center_velocity;
	float max_turning_velocity;
}MOLLIFIER;

#define MOLLIFIER_INTEGRAL 0.444

extern float mollifier_timer;

extern volatile char g_acc_flag;
extern volatile char g_MotorEnd_flag;

void cal_table(TRAPEZOID,TARGET *);
void cal_table_dis(TRAPEZOID,TARGET *);
void cal_table_max(TRAPEZOID,TARGET *);
void cal_mollifier_table(MOLLIFIER,TARGET *);
float cal_mollifier_velocity(float,float,float);
float cal_mollifier_acceleration(float,float,float);

#endif /* INC_CAL_ACCELERATION_H_ */
