/*
 * FF_motor.c
 *
 *  Created on: 2023/01/21
 *      Author: sf199
 */

#include "FF_motor.h"
#include "CL_EnoderGyro.h"
#include "Control_motor.h"
#include "fail_safe.h"

void feedforward_const_accel(float *feedforward_straight,
		float straight_velocity, float straight_acceleration,
		float *feedforward_turning, float turning_velocity,
		float turning_acceleration) {

float Im;
float ImT;//0.00000116 //kg/m^2 ロータ慣性モーメントターン用
float kbT;//Vs/rad 逆起電力定数 deg->rad->m変換sitenai

if(highspeed_mode == 1){
	Im = 0.00000073*2;
	ImT = 0.00000047;
	kbT = 0.00003;
}else{
	Im = 0.00000038;
	ImT = 0.00000055;
	kbT = 0.00001;
}
if( turning_acceleration > 0 && turning_velocity < 200 && modeacc == 2){
	//ImT = 0.000002;
	ImT = 0.0000008;
}
//if(straight_velocity>1200){
//	Im = 0.00000041;
//}

	if (straight_velocity >= 0) {
		*feedforward_straight = (((Im * ng * straight_acceleration / 1000
				/ TIRE_DIAMETER) + Tw) * Rm / kt)
				+ (straight_velocity / 1000 * ng * kb / TIRE_DIAMETER);
	} else {
		*feedforward_straight = (((Im * ng * straight_acceleration / 1000
				/ TIRE_DIAMETER) - Tw) * Rm / kt)
				+ (straight_velocity / 1000 * ng * kb / TIRE_DIAMETER);
	}
	if (turning_velocity >= 0) {
		*feedforward_turning = (((ImT * ng * turning_acceleration / 1000
				/ TIRE_DIAMETER) + TwT) * Rm / kt)
				+ (turning_velocity / 1000 * ng * kbT / TIRE_DIAMETER);
	} else {
		*feedforward_turning = (((ImT * ng * turning_acceleration / 1000
				/ TIRE_DIAMETER) - TwT) * Rm / kt)
				+ (turning_velocity / 1000 * ng * kbT / TIRE_DIAMETER);
	}
	//*feedforward_straight=0;
	//*feedforward_turning =0;

}
