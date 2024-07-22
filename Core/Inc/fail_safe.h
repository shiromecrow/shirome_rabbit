/*
 * fail_safe.h
 *
 *  Created on: 2023/01/20
 *      Author: sf199
 */

#ifndef INC_FAIL_SAFE_H_
#define INC_FAIL_SAFE_H_

extern char no_safty;
extern unsigned char error_mode;
extern char highspeed_mode;
extern float encoder_PID_error;
extern float gyro_PID_error;
extern float gyro_x_error;
extern float gyro_x_error_highspeed;

void init_FailSafe();

void interrupt_FailSafe();


#endif /* INC_FAIL_SAFE_H_ */
