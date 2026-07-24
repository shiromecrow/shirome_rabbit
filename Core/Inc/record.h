/*
 * record.h
 *
 *  Created on: Jan 12, 2023
 *      Author: sf199
 */

#ifndef INC_RECORD_H_
#define INC_RECORD_H_


#include <stdbool.h>

#define MAX_RECORD_TOTAL  13000   // 総データ数(メモリの限界)

#define RECORD_STOPNUM -1
#define RECORD_STOPMODE 1000

typedef struct {
    void (*record_func)(float* d, const char* header_out[], int* out_count, int* sample_count, int request);
} RecordMode;

extern RecordMode record_modes[];
extern int num_record_modes;


extern short record_mode;
extern int record_time;
extern char record_rupe_flag;



void record_start(int mode_index);
void record_stop(void);
void record_pause(void);
void record_resume(void);

void record_data(float *,int,int);
void record_print();

void record_reset(void);

void interrupt_record(void);


#endif /* INC_RECORD_H_ */
