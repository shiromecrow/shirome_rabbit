/*
 * PL_flash.c
 *
 *  Created on: Jan 9, 2024
 *      Author: sf199
 */


/*
 * PL_flash.c
 *
 *  Created on: Jan 12, 2023
 *      Author: sf199
 */


#include "stm32g4xx_hal.h"
#include "PL_flash.h"
#include <string.h>
#include <stdint.h>
#include "stdio.h"
#include "define.h"




const uint32_t start_address = 0x807F000; //bank1 page last start address
const uint32_t end_adress = 0x807FFFF; // bank1 page last end address

void test_flash(void){
		  record_out();
		  maze_display();
		  flash_record_init();
		  record_in();
}


void eraseFlash( void )
{
	FLASH_EraseInitTypeDef erase;
	erase.TypeErase = FLASH_TYPEERASE_PAGES;	// select page
	erase.Banks = FLASH_BANK_1;		       // set bank1
	erase.Page = 254;// set page254(127)
	erase.NbPages = 1;//delete page
	uint32_t pageError = 0;

	HAL_FLASHEx_Erase(&erase, &pageError);	// erase sector
//	printf("page=%x\n\r", pageError);

//	for (int i=0;i<10;i++){
//		printf("nowpage%x=%x\n\r",((start_address+i)&65535), (*(uint8_t*)(start_address+i)));
//	}

}


void writeFlash(uint32_t address, uint64_t *data, uint32_t size,uint8_t erasemode)
{
	HAL_FLASH_Unlock();		// unlock flash
	if(erasemode==1){eraseFlash();}

	for ( uint32_t add = address; add < (address + size); add+=8 ){
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, add, (uint64_t *)data); // write byte
		data=data+4;  // add data pointer
	}

	HAL_FLASH_Lock();		// lock flash

}

void loadFlash(uint32_t address, uint64_t *data, uint32_t size )
{
	memcpy(data, (uint64_t*)address,size); // copy data

}


void record_in(void) {
	int t = 0;
	uint32_t address=start_address;

	while (t <= MAZE_SQUARE_NUM-2) {
		record.row[t] = wall.row[t];
		record.column[t] = wall.column[t];
		t++;
	}
	t = 0;
	while (t <= MAZE_SQUARE_NUM-2) {
		record.row_look[t] = wall.row_look[t];
		record.column_look[t] = wall.column_look[t];
		t++;
	}
	t = 0;

	writeFlash(address, (uint64_t*) record.row[0], 1, ON);
	address+=16;
	t=1;

	while (t <= MAZE_SQUARE_NUM-2) {
	writeFlash(address, (uint64_t*) record.row[t], 1, OFF);
	address+=16;
	t++;
	}

	t=0;
	while (t <= MAZE_SQUARE_NUM-2) {
	writeFlash(address, (uint64_t*) record.column[t], 1, OFF);
	address+=16;
	t++;
	}

	t=0;
	while (t <= MAZE_SQUARE_NUM-2) {
	writeFlash(address, (uint64_t*) record.row_look[t], 1, OFF);
	address+=16;
	t++;
	}

	t=0;
	while (t <= MAZE_SQUARE_NUM-2) {
	writeFlash(address, (uint64_t*) record.column_look[t], 1, OFF);
	address+=16;
	t++;
	}
	printf("%x\n",address);
//	writeFlash(start_address + sizeof(record.row), (uint64_t*) record.column,
//			sizeof(record.column), OFF);
//	writeFlash(start_address + 2 * sizeof(record.row),
//			(uint64_t*) record.row_look, sizeof(record.row_look),
//			OFF);
//	writeFlash(start_address + 3 * sizeof(record.row),
//			(uint64_t*) record.column_look, sizeof(record.column_look),
//			OFF);

//	for (int i=0;i<200;i++){
//		printf("nowpage%x=%x\n\r",((start_address+i)&65535), (*(uint8_t*)(start_address+i)));
//	}

}


void record_out(void) {
//	for (int i=0;i<200;i++){
//		printf("nowpage%x=%x\n\r",((start_address+i)&65535), (*(uint8_t*)(start_address+i)));
//	}

	int t=0;
	uint32_t address=start_address;
	while (t <= MAZE_SQUARE_NUM-2) {
	loadFlash(address, (uint64_t*)&record.row[t], 4);
	address+=16;
	t++;
	}

	t=0;
	while (t <= MAZE_SQUARE_NUM-2) {
	loadFlash(address, (uint64_t*)&record.column[t], 4);
	address+=16;
	t++;
	}

	t=0;
	while (t <= MAZE_SQUARE_NUM-2) {
		loadFlash(address, (uint64_t*)&record.row_look[t], 4);
		address+=16;
		t++;
	}

	t=0;
	while (t <= MAZE_SQUARE_NUM-2) {
		loadFlash(address, (uint64_t*)&record.column_look[t], 4);
		address+=16;
		t++;
	}
//	loadFlash(start_address, (uint64_t*) record.row, sizeof(record.row));
//	loadFlash(start_address + sizeof(record.row), (uint64_t*) record.column,
//			sizeof(record.column));
//	loadFlash(start_address + 2 * sizeof(record.row),
//			(uint64_t*) record.row_look, sizeof(record.row_look));
//	loadFlash(start_address + 3 * sizeof(record.row),
//			(uint64_t*) record.column_look, sizeof(record.column_look));
	t = 0;
	while (t <= MAZE_SQUARE_NUM-2) {
		wall.row[t] = record.row[t];
		wall.column[t] = record.column[t];
		t++;
	}
	t = 0;
	while (t <= MAZE_SQUARE_NUM-2) {
		wall.row_look[t] = record.row_look[t];
		wall.column_look[t] = record.column_look[t];
		t++;
	}

	t = 0;

}

void flash_record_init(void){
	wall.row[0]=	1341141052	;	wall.row_look[0]=	3354918911	;
	wall.row[1]=	195041298	;	wall.row_look[1]=	3354918911	;
	wall.row[2]=	214645792	;	wall.row_look[2]=	3355443199	;
	wall.row[3]=	201357430	;	wall.row_look[3]=	3694985215	;
	wall.row[4]=	167836920	;	wall.row_look[4]=	3728539647	;
	wall.row[5]=	367001596	;	wall.row_look[5]=	4294967295	;
	wall.row[6]=	201052138	;	wall.row_look[6]=	4294967295	;
	wall.row[7]=	402620420	;	wall.row_look[7]=	4294963229	;
	wall.row[8]=	167772170	;	wall.row_look[8]=	3959451677	;
	wall.row[9]=	822083588	;	wall.row_look[9]=	3875565597	;
	wall.row[10]=	176160778	;	wall.row_look[10]=	4018171933	;
	wall.row[11]=	88080388	;	wall.row_look[11]=	4022366237	;
	wall.row[12]=	245374954	;	wall.row_look[12]=	4024467455	;
	wall.row[13]=	492801022	;	wall.row_look[13]=	4261412863	;
	wall.row[14]=	976211580	;	wall.row_look[14]=	4278190079	;
	wall.row[15]=	1582689624	;	wall.row_look[15]=	4294967295	;
	wall.row[16]=	507159888	;	wall.row_look[16]=	3221209087	;
	wall.row[17]=	746937000	;	wall.row_look[17]=	2915565567	;
	wall.row[18]=	12757672	;	wall.row_look[18]=	2378432503	;
	wall.row[19]=	818758992	;	wall.row_look[19]=	4278321143	;
	wall.row[20]=	1007134032	;	wall.row_look[20]=	4278255591	;
	wall.row[21]=	2063620776	;	wall.row_look[21]=	4278219751	;
	wall.row[22]=	29371048	;	wall.row_look[22]=	2214607359	;
	wall.row[23]=	13636724	;	wall.row_look[23]=	2216173055	;
	wall.row[24]=	18876936	;	wall.row_look[24]=	2216169183	;
	wall.row[25]=	58721544	;	wall.row_look[25]=	2216169245	;
	wall.row[26]=	29360868	;	wall.row_look[26]=	2216169469	;
	wall.row[27]=	2062022136	;	wall.row_look[27]=	4294967293	;
	wall.row[28]=	1040179400	;	wall.row_look[28]=	4294967293	;
	wall.row[29]=	2095575548	;	wall.row_look[29]=	4262460465	;
	wall.row[30]=	2147482622	;	wall.row_look[30]=	4294967295	;
	wall.column[0]=	2147475451	;	wall.column_look[0]=	4294967295	;
	wall.column[1]=	1056931876	;	wall.column_look[1]=	2684354559	;
	wall.column[2]=	25100318	;	wall.column_look[2]=	2684354559	;
	wall.column[3]=	243285964	;	wall.column_look[3]=	3213885439	;
	wall.column[4]=	385884032	;	wall.column_look[4]=	3217031167	;
	wall.column[5]=	6	;	wall.column_look[5]=	3120554111	;
	wall.column[6]=	32783	;	wall.column_look[6]=	2583683199	;
	wall.column[7]=	117456926	;	wall.column_look[7]=	2650792063	;
	wall.column[8]=	713064479	;	wall.column_look[8]=	2667569279	;
	wall.column[9]=	1962934302	;	wall.column_look[9]=	4282376319	;
	wall.column[10]=	2055208963	;	wall.column_look[10]=	4288667743	;
	wall.column[11]=	1065295744	;	wall.column_look[11]=	4294967263	;
	wall.column[12]=	11071424	;	wall.column_look[12]=	4043309023	;
	wall.column[13]=	5537670	;	wall.column_look[13]=	4034854879	;
	wall.column[14]=	2768655	;	wall.column_look[14]=	4030660575	;
	wall.column[15]=	1376286	;	wall.column_look[15]=	4028612831	;
	wall.column[16]=	655379	;	wall.column_look[16]=	4027564231	;
	wall.column[17]=	852030	;	wall.column_look[17]=	4027039999	;
	wall.column[18]=	267026459	;	wall.column_look[18]=	4278436095	;
	wall.column[19]=	2122121238	;	wall.column_look[19]=	4278305022	;
	wall.column[20]=	217849916	;	wall.column_look[20]=	2667700478	;
	wall.column[21]=	125177904	;	wall.column_look[21]=	2416439550	;
	wall.column[22]=	35776528	;	wall.column_look[22]=	2416441542	;
	wall.column[23]=	1561624	;	wall.column_look[23]=	2420112582	;
	wall.column[24]=	547267112	;	wall.column_look[24]=	2956967142	;
	wall.column[25]=	535579984	;	wall.column_look[25]=	3221086198	;
	wall.column[26]=	265814688	;	wall.column_look[26]=	3221086198	;
	wall.column[27]=	792896	;	wall.column_look[27]=	2956843504	;
	wall.column[28]=	409534	;	wall.column_look[28]=	2956197872	;
	wall.column[29]=	711038	;	wall.column_look[29]=	2956197886	;
	wall.column[30]=	1071611900	;	wall.column_look[30]=	4294967294	;



}



