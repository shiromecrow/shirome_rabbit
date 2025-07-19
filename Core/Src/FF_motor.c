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

float KAFF_straight, KVFF_straight, KFF_straight;
float KAFF_turning, KVFF_turning, KFF_turning;

// float Im;
// float ImT;//0.00000116 //kg/m^2 ロータ慣性モーメントターン用
// float kb;//Vs/rad 逆起電力定数 deg->rad->m変換sitenai
// float kbT;//Vs/rad 逆起電力定数 deg->rad->m変換sitenai

if(highspeed_mode == 1){
	KAFF_straight = 1.4114e-04*0.57;// Im * ng * straight_acceleration / 1000 / TIRE_DIAMETER * Rm / kt
	KVFF_straight = 0.0016;// straight_velocity / 1000 * ng * kb / TIRE_DIAMETER
	KFF_straight = 0;// Tw * Rm / kt
	KAFF_turning = 2.8135e-05*0.57;// ImT * ng * turning_acceleration / 1000 / TIRE_DIAMETER* Rm / kt
	KVFF_turning = 5.9272e-04;// turning_velocity / 1000 * ng * kbT / TIRE_DIAMETER
	KFF_turning = 0;// TwT * Rm / kt
}else{
	KAFF_straight = 2.4178e-05;// Im * ng * straight_acceleration / 1000 / TIRE_DIAMETER * Rm / kt
	KVFF_straight = 1.0989e-06;// straight_velocity / 1000 * ng * kb / TIRE_DIAMETER
	KFF_straight = 0;// Tw * Rm / kt
	KAFF_turning = 1.2725e-05;// ImT * ng * turning_acceleration / 1000 / TIRE_DIAMETER* Rm / kt
	KVFF_turning = 3.3299e-06;// turning_velocity / 1000 * ng * kbT / TIRE_DIAMETER
	KFF_turning = 0;// TwT * Rm / kt
}
if( turning_acceleration > 0 && turning_velocity < 200 && (modeacc == 2 || modeacc == 9)){
	KAFF_turning = 1.2725e-05;// ImT * ng * turning_acceleration / 1000 / TIRE_DIAMETER* Rm / kt
}
	if (straight_velocity >= 0) {
		*feedforward_straight = KAFF_straight * straight_acceleration
								+ KVFF_straight * straight_velocity
								+ KFF_straight;
	} else {
		*feedforward_straight = KAFF_straight * straight_acceleration
								+ KVFF_straight * straight_velocity
								- KFF_straight;
	}
	if (turning_velocity >= 0) {
		*feedforward_turning = KAFF_turning * turning_acceleration
								+ KVFF_turning * turning_velocity
								+ KFF_turning;
	} else {
		*feedforward_turning = KAFF_turning * turning_acceleration
								+ KVFF_turning * turning_velocity
								- KFF_turning;
	}
	//*feedforward_straight=0;
	//*feedforward_turning =0;

}
