/*
 * PID_EncoderGyro.h
 *
 *  Created on: Jan 14, 2023
 *      Author: sf199
 */

#ifndef INC_PID_ENCODERGYRO_H_
#define INC_PID_ENCODERGYRO_H_

struct PID{

	float error;
	float old_error;
	float sigma_error;
	float delta_error;

};

extern struct PID enc;
extern struct PID Gyro;

extern float Ksp,Ksi,Ksd;
extern float Ktp,Kti,Ktd;
extern char Encoder_PID_mode,Gyro_PID_mode,Accel_PID_mode;

void PID_Init();

void clear_Ierror();

void EncoderGyro_PID(float *, float *,float,float,float);



#endif /* INC_PID_ENCODERGYRO_H_ */
