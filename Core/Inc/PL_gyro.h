/*
 * PL_gyro.h
 *
 *  Created on: Dec 24, 2023
 *      Author: sf199
 */

#ifndef INC_PL_GYRO_H_
#define INC_PL_GYRO_H_


#define GRAVITATION 9.80665

typedef struct {

float omega_x;

float omega_y;

float omega_z;

float accel_x;

float accel_y;

float accel_z;

}GYRO_DATA;



extern GYRO_DATA gyro;


void pl_gyro_init( void );

void ICM20602_DataUpdate( void );

#endif /* INC_PL_GYRO_H_ */
