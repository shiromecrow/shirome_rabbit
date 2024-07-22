/*
 * matrix_calculation.h
 *
 *  Created on: Oct 15, 2023
 *      Author: sf199
 */

#ifndef INC_MATRIX_CALCULATION_H_
#define INC_MATRIX_CALCULATION_H_

#include "stm32g4xx_hal.h"



void mat_add(float *, float *, float *, int, int);
void mat_sub(float *, float *, float *, int, int);
void mat_mul(float *, float *, float *, int , int , int , int );
void mat_tran(float *, float *, int , int );
void mat_mul_const(float *,float , float *, int , int );
void mat_inv(float *, float *, int , int );


#endif /* INC_MATRIX_CALCULATION_H_ */
