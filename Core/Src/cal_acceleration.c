/*
 * cal_acceleration.c
 *
 *  Created on: 2023/01/16
 *      Author: sf199
 */

#include "cal_acceleration.h"
#include "CL_EnoderGyro.h"
#include "PL_timer.h"

#include "define.h"
#include "FF_motor.h"

#include "math.h"

float mollifier_timer;

volatile char g_acc_flag;
volatile char g_MotorEnd_flag;


void cal_table(TRAPEZOID input,TARGET *target){
float time_over;
if (input.displacement>=0){
	switch (g_acc_flag) {
	case 0:
		//速度FBなし
		break;
	case 1:
		//加速(減速)
			if (target->velocity >= input.count_velocity){
				target->velocity = input.count_velocity;
				target->acceleration = 0;
				g_acc_flag=2;
			}
			else if((input.displacement <= (2*target->velocity*target->velocity
					-input.start_velocity*input.start_velocity
					-input.end_velocity*input.end_velocity)
					/2/input.acceleration)){
				time_over=((2*target->velocity*target->velocity
						-input.start_velocity*input.start_velocity
						-input.end_velocity*input.end_velocity)
						/2/input.acceleration-input.displacement)/target->velocity;
				target->displacement -= 1/2*INTERRUPT_TIME*input.acceleration*(2*time_over);
				target->velocity -= input.acceleration*(2*time_over);

				target->acceleration = -input.acceleration;
				g_acc_flag=3;
			}
		break;
	case 2:
		//定常
		if (input.displacement-target->displacement <=
				(input.count_velocity*input.count_velocity
						-input.end_velocity*input.end_velocity)/2/input.acceleration) {
			time_over=(target->displacement+(input.count_velocity*input.count_velocity
						-input.end_velocity*input.end_velocity)/2
						/input.acceleration-input.displacement)/target->velocity;
			target->displacement -= 1/2*INTERRUPT_TIME*input.acceleration*time_over;
			target->velocity -= input.acceleration*(time_over);

			target->acceleration = -input.acceleration;
			g_acc_flag=3;
		}
		break;
	case 3:
		//減速(加速)
		if (target->velocity <= input.end_velocity){
			target->velocity = input.end_velocity;
			target->acceleration = 0;
			g_acc_flag=4;
		}
		break;
	case 4:
		//終了(0でもいいかも)
		break;
	case 5:
		//加速のみ
		if (target->displacement >= input.displacement){
			target->acceleration = 0;
			g_acc_flag=4;
		}
		break;
	case 6:
		//減速のみ
		if (target->displacement >= input.displacement){
			target->acceleration = 0;
			g_acc_flag=4;
		}
		break;
	}
}else{
	switch (g_acc_flag) {
	case 0:
		//速度FBなし
		break;
	case 1:
		//加速(減速)
			if (target->velocity <= input.count_velocity){
				target->velocity = input.count_velocity;
				target->acceleration = 0;
				g_acc_flag=2;
			}

			else if((-input.displacement <= (2*target->velocity*target->velocity
					-input.start_velocity*input.start_velocity
					-input.end_velocity*input.end_velocity)
					/2/input.acceleration)){
				time_over=(-(2*target->velocity*target->velocity
						-input.start_velocity*input.start_velocity
						-input.end_velocity*input.end_velocity)
						/2/input.acceleration-input.displacement)/target->velocity;
				target->displacement += 1/2*INTERRUPT_TIME*input.acceleration*(2*time_over);
				target->velocity += input.acceleration*(2*time_over);

				target->acceleration = input.acceleration;
				g_acc_flag=3;
			}
		break;
	case 2:
		//定常
		if (-input.displacement+target->displacement <=
				(input.count_velocity*input.count_velocity
						-input.end_velocity*input.end_velocity)/2/input.acceleration) {
			time_over=(target->displacement-(input.count_velocity*input.count_velocity
						-input.end_velocity*input.end_velocity)/2
						/input.acceleration-input.displacement)/target->velocity;
			target->displacement += 1/2*INTERRUPT_TIME*input.acceleration*time_over;
			target->velocity += input.acceleration*(time_over);

			target->acceleration = input.acceleration;
			g_acc_flag=3;
		}
		break;
	case 3:
		//減速(加速)
		if (target->velocity >= input.end_velocity){
			target->velocity = input.end_velocity;
			target->acceleration = 0;
			g_acc_flag=4;
		}
		break;
	case 4:
		//終了(0でもいいかも)
		g_MotorEnd_flag=1;
		break;
	case 5:
		//加速のみ
		if (target->displacement <= input.displacement){
			target->acceleration = 0;
			g_acc_flag=4;
		}
		break;
	case 6:
		//減速のみ
		if (target->displacement <= input.displacement){
			target->acceleration = 0;
			g_acc_flag=4;
		}
		break;
	}

}

}



// 減速をより速くした機能を追加
void cal_table_dis(TRAPEZOID input,TARGET *target){
float time_over;
float acc_distance;//初速度から現在の速度に達するまでに使った距離
float dec_distance;//現在の速度から終端速度になるのに必要なた距離
float velocity=(fusion_speedL+fusion_speedR)/2;
float distance=(fusion_distanceL+fusion_distanceL)/2;
if (input.displacement>=0){
	switch (g_acc_flag) {
	case 0:
		//速度FBなし
		break;
	case 1:
		//加速(減速)
        acc_distance = (target->velocity*target->velocity-input.start_velocity*input.start_velocity)/2/input.acceleration;
        dec_distance = (target->velocity*target->velocity-input.end_velocity*input.end_velocity)/2/input.deceleration;
        if (target->velocity >= input.count_velocity){
            target->velocity = input.count_velocity;
            target->acceleration = 0;
            g_acc_flag=2;
        }
        else if(input.displacement <= (acc_distance + dec_distance)){
            time_over=(acc_distance + dec_distance - input.displacement)/target->velocity;
            target->displacement -= 1/2*INTERRUPT_TIME*input.acceleration*(2*time_over);
            target->velocity -= input.acceleration*(2*time_over);

            target->acceleration = -input.deceleration;
            g_acc_flag=3;
        }
		break;
	case 2:
		//定常
		acc_distance = (input.count_velocity*input.count_velocity-input.start_velocity*input.start_velocity)/2/input.acceleration;
		dec_distance = (input.count_velocity*input.count_velocity-input.end_velocity*input.end_velocity)/2/input.deceleration;
		if (input.displacement-target->displacement <= dec_distance) {
			time_over=(target->displacement+dec_distance-input.displacement)/target->velocity;
			target->displacement -= 1/2*INTERRUPT_TIME*input.acceleration*time_over;
			target->velocity -= input.acceleration*(time_over);

			target->acceleration = -input.deceleration;
			g_acc_flag=3;
		}
		break;
	case 3:
		//減速(加速)
		if (target->velocity <= input.end_velocity){
			target->velocity = input.end_velocity;
			target->acceleration = 0;
			g_acc_flag=4;
		}
		break;
	case 4:
		//終了(0でもいいかも)
		break;
	case 5:
		//加速のみ
		if (target->displacement >= input.displacement){
			target->acceleration = 0;
			g_acc_flag=4;
		}
		break;
	case 6:
		//減速のみ
		if (target->displacement >= input.displacement){
			target->acceleration = 0;
			g_acc_flag=4;
		}
		break;
	}
}else{
	switch (g_acc_flag) {
	case 0:
		//速度FBなし
		break;
	case 1:
		//加速(減速)
		acc_distance = (target->velocity*target->velocity-input.start_velocity*input.start_velocity)/2/input.acceleration;
		dec_distance = (target->velocity*target->velocity-input.end_velocity*input.end_velocity)/2/input.deceleration;
			if (target->velocity <= input.count_velocity){
				target->velocity = input.count_velocity;
				target->acceleration = 0;
				g_acc_flag=2;
			}
			else if(-input.displacement <= (acc_distance + dec_distance)){
				time_over=( -acc_distance - dec_distance - input.displacement)/target->velocity;
				target->displacement += 1/2*INTERRUPT_TIME*input.acceleration*(2*time_over);
				target->velocity += input.acceleration*(2*time_over);

				target->acceleration = input.deceleration;
				g_acc_flag=3;
			}
		break;
	case 2:
		//定常
		acc_distance = (input.count_velocity*input.count_velocity-input.start_velocity*input.start_velocity)/2/input.acceleration;
		dec_distance = (input.count_velocity*input.count_velocity-input.end_velocity*input.end_velocity)/2/input.deceleration;
		if (-input.displacement+target->displacement <= dec_distance) {
			time_over=(target->displacement-dec_distance-input.displacement)/target->velocity;
			target->displacement += 1/2*INTERRUPT_TIME*input.acceleration*time_over;
			target->velocity += input.acceleration*(time_over);

			target->acceleration = input.deceleration;
			g_acc_flag=3;
		}
		break;
	case 3:
		//減速(加速)
		if (target->velocity >= input.end_velocity){
			target->velocity = input.end_velocity;
			target->acceleration = 0;
			g_acc_flag=4;
		}
		break;
	case 4:
		//終了(0でもいいかも)
		g_MotorEnd_flag=1;
		break;
	case 5:
		//加速のみ
		if (target->displacement <= input.displacement){
			target->acceleration = 0;
			g_acc_flag=4;
		}
		break;
	case 6:
		//減速のみ
		if (target->displacement <= input.displacement){
			target->acceleration = 0;
			g_acc_flag=4;
		}
		break;
	}

}

}



// 加速は常にMAX DUTYモード
void cal_table_max(TRAPEZOID input,TARGET *target){
float time_over;
float acc_distance;//初速度から現在の速度に達するまでに使った距離
float dec_distance;//現在の速度から終端速度になるのに必要なた距離
float velocity=kalman_speed;
float distance=kalman_distance;


if (input.displacement>=0){
	switch (g_acc_flag) {
	case 0:
		//速度FBなし
		break;
	case 1:
		//加速(減速)
		if(target->velocity <=3000){
        acc_distance = (target->velocity*target->velocity-input.start_velocity*input.start_velocity)/2/input.acceleration;
        dec_distance = (target->velocity*target->velocity-input.end_velocity*input.end_velocity)/2/input.deceleration;
        if (target->velocity >= input.count_velocity){
            target->velocity = input.count_velocity;
            target->acceleration = 0;
            g_acc_flag=2;
        }
        else if(input.displacement <= (acc_distance + dec_distance)){
            time_over=(acc_distance + dec_distance - input.displacement)/target->velocity;
            target->displacement -= 1/2*INTERRUPT_TIME*input.acceleration*(2*time_over);
            target->velocity -= input.acceleration*(2*time_over);

            target->acceleration = -input.deceleration;
            g_acc_flag=3;
        }
		}else{
			acc_distance = distance+velocity*INTERRUPT_TIME;
			dec_distance = (velocity*velocity-input.end_velocity*input.end_velocity)/2/input.deceleration;
			target->displacement = distance;
			target->velocity = velocity;
			target->acceleration = 10000000;
			if (velocity >= input.count_velocity){
	            target->velocity = input.count_velocity;
	            target->acceleration = 0;
	            g_acc_flag=2;
			}
			else if(input.displacement <= (acc_distance + dec_distance)){
				time_over=(acc_distance + dec_distance - input.displacement)/velocity;
				target->displacement -= 1/2*INTERRUPT_TIME*input.deceleration*(2*time_over);
				target->velocity -= input.deceleration*(2*time_over+INTERRUPT_TIME);

				target->acceleration = -input.deceleration;
				g_acc_flag=3;
			}
		}
		break;
	case 2:
		//定常
		acc_distance = (input.count_velocity*input.count_velocity-input.start_velocity*input.start_velocity)/2/input.acceleration;
		dec_distance = (input.count_velocity*input.count_velocity-input.end_velocity*input.end_velocity)/2/input.deceleration;
		if (input.displacement-target->displacement <= dec_distance) {
			time_over=(target->displacement+dec_distance-input.displacement)/target->velocity;
			target->displacement -= 1/2*INTERRUPT_TIME*input.acceleration*time_over;
			target->velocity -= input.acceleration*(time_over);

			target->acceleration = -input.deceleration;
			g_acc_flag=3;
		}
		break;
	case 3:
		//減速(加速)
		if (target->velocity <= input.end_velocity){
			target->velocity = input.end_velocity;
			target->acceleration = 0;
			g_acc_flag=4;
		}
		break;
	case 4:
		//終了(0でもいいかも)
		break;
	case 5:
		//加速のみ
		if (target->displacement >= input.displacement){
			target->acceleration = 0;
			g_acc_flag=4;
		}
		break;
	case 6:
		//減速のみ
		if (target->displacement >= input.displacement){
			target->acceleration = 0;
			g_acc_flag=4;
		}
		break;
	}
}else{
	switch (g_acc_flag) {
	case 0:
		//速度FBなし
		break;
	case 1:
		//加速(減速)
		acc_distance = (target->velocity*target->velocity-input.start_velocity*input.start_velocity)/2/input.acceleration;
		dec_distance = (target->velocity*target->velocity-input.end_velocity*input.end_velocity)/2/input.deceleration;
			if (target->velocity <= input.count_velocity){
				target->velocity = input.count_velocity;
				target->acceleration = 0;
				g_acc_flag=2;
			}
			else if(-input.displacement <= (acc_distance + dec_distance)){
				time_over=( -acc_distance - dec_distance - input.displacement)/target->velocity;
				target->displacement += 1/2*INTERRUPT_TIME*input.acceleration*(2*time_over);
				target->velocity += input.acceleration*(2*time_over);

				target->acceleration = input.deceleration;
				g_acc_flag=3;
			}
		break;
	case 2:
		//定常
		acc_distance = (input.count_velocity*input.count_velocity-input.start_velocity*input.start_velocity)/2/input.acceleration;
		dec_distance = (input.count_velocity*input.count_velocity-input.end_velocity*input.end_velocity)/2/input.deceleration;
		if (-input.displacement+target->displacement <= dec_distance) {
			time_over=(target->displacement-dec_distance-input.displacement)/target->velocity;
			target->displacement += 1/2*INTERRUPT_TIME*input.acceleration*time_over;
			target->velocity += input.acceleration*(time_over);

			target->acceleration = input.deceleration;
			g_acc_flag=3;
		}
		break;
	case 3:
		//減速(加速)
		if (target->velocity >= input.end_velocity){
			target->velocity = input.end_velocity;
			target->acceleration = 0;
			g_acc_flag=4;
		}
		break;
	case 4:
		//終了(0でもいいかも)
		g_MotorEnd_flag=1;
		break;
	case 5:
		//加速のみ
		if (target->displacement <= input.displacement){
			target->acceleration = 0;
			g_acc_flag=4;
		}
		break;
	case 6:
		//減速のみ
		if (target->displacement <= input.displacement){
			target->acceleration = 0;
			g_acc_flag=4;
		}
		break;
	}

}

}




void cal_mollifier_table(MOLLIFIER input,TARGET *target){

float mollifier_T;
float old_velocity;
float time_delay=12;
float time_delay2=-10;
float time_delay3=3;
float time_delay4=5;
float OverShot=0;
	mollifier_timer+=INTERRUPT_TIME;
		mollifier_T=2*fabs(input.displacement)/MOLLIFIER_INTEGRAL*exp(-1)/input.max_turning_velocity;
		if (mollifier_timer>-mollifier_T/2 && mollifier_timer<mollifier_T/2){
			old_velocity=target->velocity;
			target->velocity = cal_mollifier_velocity(mollifier_timer,mollifier_T,input.displacement);

			if(mollifier_timer<-mollifier_T/2/1.316+time_delay*INTERRUPT_TIME){
				target->acceleration = cal_mollifier_acceleration(-mollifier_T/2/1.316,mollifier_T,input.displacement);
			}else if(mollifier_timer<0){
				target->acceleration = cal_mollifier_acceleration(mollifier_timer-INTERRUPT_TIME*time_delay,mollifier_T,input.displacement);
			}else if(mollifier_timer<mollifier_T/2/1.316+time_delay2*INTERRUPT_TIME){
				target->acceleration = cal_mollifier_acceleration(mollifier_timer-INTERRUPT_TIME*time_delay,mollifier_T,input.displacement);
			}else if(mollifier_timer<mollifier_T/2+time_delay2*INTERRUPT_TIME){
				time_delay=0;
				target->acceleration = cal_mollifier_acceleration(mollifier_T/2/1.316,mollifier_T,input.displacement);
			}else{
				target->acceleration = cal_mollifier_acceleration(mollifier_T/2-INTERRUPT_TIME,mollifier_T,input.displacement);
			}
			target->acceleration = cal_mollifier_acceleration(mollifier_timer+INTERRUPT_TIME*time_delay3,mollifier_T,input.displacement);

			if(mollifier_timer>mollifier_T/2-INTERRUPT_TIME*time_delay4){
				target->acceleration = OverShot*cal_mollifier_acceleration(mollifier_timer,mollifier_T,input.displacement);
			}


			//			if(mollifier_timer>-mollifier_T/2*0.35 && mollifier_timer<mollifier_T/2*0.45){
//							target->acceleration = 0.7*target->acceleration;
//			}
//			if(mollifier_timer>mollifier_T/2*0.6){
//							target->acceleration = 0.4*target->acceleration;
//			}
//			if(mollifier_timer>mollifier_T/2*0.9){
//							target->acceleration = -0.6*target->acceleration;
//			}
		}else{
			old_velocity=target->velocity;
			target->velocity=0;
			target->acceleration = -target->velocity+old_velocity;
			g_acc_flag=4;

		}

}


float cal_mollifier_velocity(float t_now,float mollifier_T,float integral){
	float velocity;
	velocity=(2/mollifier_T)*integral/MOLLIFIER_INTEGRAL*exp(-mollifier_T*mollifier_T/4/(mollifier_T*mollifier_T/4-t_now*t_now));
	return velocity;
}
float cal_mollifier_acceleration(float t_now,float mollifier_T,float integral){
	float acceleration;
	acceleration= integral/MOLLIFIER_INTEGRAL*(-mollifier_T*t_now/(mollifier_T*mollifier_T/4-t_now*t_now)/(mollifier_T*mollifier_T/4-t_now*t_now))*exp(-mollifier_T*mollifier_T/4/(mollifier_T*mollifier_T/4-t_now*t_now));
	return acceleration;
}
