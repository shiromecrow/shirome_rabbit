/*
 * PL_flash.h
 *
 *  Created on: Jan 9, 2024
 *      Author: sf199
 */

#ifndef INC_PL_FLASH_H_
#define INC_PL_FLASH_H_


#include"maze_wall.h"



extern const uint32_t start_address; //sentor11 start address
extern const uint32_t end_adress; // sector11 end address

extern uint8_t *flash_test;

void test_flash();

void eraseFlash();
void writeFlash(uint32_t , uint64_t *, uint32_t , uint8_t);
void loadFlash(uint32_t, uint64_t *, uint32_t );

void record_in();
void record_out();
void flash_record_init();


#endif /* INC_PL_FLASH_H_ */
