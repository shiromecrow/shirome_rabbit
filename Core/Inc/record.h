/*
 * record.h
 *
 *  Created on: Jan 12, 2023
 *      Author: sf199
 */

#ifndef INC_RECORD_H_
#define INC_RECORD_H_


#define max_record_num 4
#define max_record_time 3000

extern char record_mode;
extern int record_time;
extern char record_rupe_flag;

void record_data(float *,int);
void record_print();

void record_reset(void);

void interrupt_record(void);


#endif /* INC_RECORD_H_ */
