/*
 * PL_encoder.c
 *
 *  Created on: Dec 26, 2023
 *      Author: sf199
 */


#include "PL_encoder.h"
#include "spi.h"
#include <stdio.h>

float encoder_R,encoder_L;

uint16_t encoder_read_byte_R(uint16_t address,uint16_t data){

	uint8_t addBuffer[2];
//	uint16_t data;
	uint8_t dataBuffer[2];
	uint16_t parity;

	HAL_GPIO_WritePin( ENCODER_R_CS_GPIO_Port, ENCODER_R_CS_Pin, GPIO_PIN_RESET); //cs = 0;

	address = address | 0x4000;//先頭から2つ目のbitを1に
	parity=0;
	for(int i=0;i<15;i++) parity += (address >> i) & 1;
	address = address | ((parity % 2) << 15);
	addBuffer[0]=address>>8;
	addBuffer[1]=address & 0x00FF;


	HAL_SPI_Transmit(&hspi3, (uint8_t*)addBuffer, 2, 50);
//	HAL_SPI_Transmit(&hspi3, address, 2, 100);

	HAL_GPIO_WritePin( ENCODER_R_CS_GPIO_Port, ENCODER_R_CS_Pin, GPIO_PIN_SET); //cs = 1;

	for(int i=0;i<50;i++){}

	HAL_GPIO_WritePin( ENCODER_R_CS_GPIO_Port, ENCODER_R_CS_Pin, GPIO_PIN_RESET); //cs = 0;

//	data=0xC000;
	dataBuffer[0]=data>>8;
	dataBuffer[1]=data & 0x00FF;
	HAL_SPI_Receive(&hspi3, (uint8_t*)dataBuffer, 2, 50);
	data=((uint16_t)(dataBuffer[0]) << 8) | (uint16_t)(dataBuffer[1]);
//	HAL_SPI_Transmit(&hspi3, data, 2, 100);
	HAL_GPIO_WritePin( ENCODER_R_CS_GPIO_Port, ENCODER_R_CS_Pin, GPIO_PIN_SET); //cs = 1;

	return data;

}


void encoder_write_byte_R(uint16_t address, uint16_t data){

	uint8_t addBuffer[2];
	uint8_t dataBuffer[2];
	uint16_t parity;

	HAL_GPIO_WritePin( ENCODER_R_CS_GPIO_Port, ENCODER_R_CS_Pin, GPIO_PIN_RESET); //cs = 0;

	address = address & 0xBFFF;//先頭から2つ目のbitを1に
	parity=0;
	for(int i=0;i<15;i++) parity += (address >> i) & 1;
	address = address | ((parity % 2) << 15);
	addBuffer[0]=address>>8;
	addBuffer[1]=address & 0x00FF;

	HAL_SPI_Transmit(&hspi3, addBuffer, 2, 50);

	HAL_GPIO_WritePin( ENCODER_R_CS_GPIO_Port, ENCODER_R_CS_Pin, GPIO_PIN_SET); //cs = 1;

	for(int i=0;i<50;i++){}

	HAL_GPIO_WritePin( ENCODER_R_CS_GPIO_Port, ENCODER_R_CS_Pin, GPIO_PIN_RESET); //cs = 0;

	data = data & 0xBFFF;//先頭から2つ目のbitを0に
	parity=0;
	for(int i=0;i<15;i++) parity += (data >> i) & 1;
	data = data | ((parity % 2) << 15);
	dataBuffer[0]=data>>8;
	dataBuffer[1]=data & 0x00FF;
	HAL_SPI_Transmit(&hspi3, dataBuffer, 2, 50);

	HAL_GPIO_WritePin( ENCODER_R_CS_GPIO_Port, ENCODER_R_CS_Pin, GPIO_PIN_SET); //cs = 1;

}


uint16_t encoder_read_byte_L(uint16_t address,uint16_t data){

	uint8_t addBuffer[2];
//	uint16_t data;
	uint8_t dataBuffer[2];
	uint16_t parity;

	HAL_GPIO_WritePin( ENCODER_L_CS_GPIO_Port, ENCODER_L_CS_Pin, GPIO_PIN_RESET); //cs = 0;

	address = address | 0x4000;//先頭から2つ目のbitを1に
	parity=0;
	for(int i=0;i<15;i++) parity += (address >> i) & 1;
	address = address | ((parity % 2) << 15);
	addBuffer[0]=address>>8;
	addBuffer[1]=address & 0x00FF;

	HAL_SPI_Transmit(&hspi3, (uint8_t*)addBuffer, 2, 50);

	HAL_GPIO_WritePin( ENCODER_L_CS_GPIO_Port, ENCODER_L_CS_Pin, GPIO_PIN_SET); //cs = 1;

	for(int i=0;i<50;i++){}

	HAL_GPIO_WritePin( ENCODER_L_CS_GPIO_Port, ENCODER_L_CS_Pin, GPIO_PIN_RESET); //cs = 0;

//	data=0x0000;
	dataBuffer[0]=data>>8;
	dataBuffer[1]=data & 0x00FF;
	HAL_SPI_Receive(&hspi3, (uint8_t*)dataBuffer, 2, 50);
	data=((uint16_t)(dataBuffer[0]) << 8) | (uint16_t)(dataBuffer[1]);
	HAL_GPIO_WritePin( ENCODER_L_CS_GPIO_Port, ENCODER_L_CS_Pin, GPIO_PIN_SET); //cs = 1;

	return data;

}


void encoder_write_byte_L(uint16_t address, uint16_t data){

	uint8_t addBuffer[2];
	uint8_t dataBuffer[2];
	uint16_t parity;

	HAL_GPIO_WritePin( ENCODER_L_CS_GPIO_Port, ENCODER_L_CS_Pin, GPIO_PIN_RESET); //cs = 0;

	address = address & 0xBFFF;//先頭から2つ目のbitを1に
	parity=0;
	for(int i=0;i<15;i++) parity += (address >> i) & 1;
	address = address | ((parity % 2) << 15);
	addBuffer[0]=address>>8;
	addBuffer[1]=address & 0x00FF;

	HAL_SPI_Transmit(&hspi3, (uint8_t*)addBuffer, 2, 50);

	HAL_GPIO_WritePin( ENCODER_L_CS_GPIO_Port, ENCODER_L_CS_Pin, GPIO_PIN_SET); //cs = 1;

	for(int i=0;i<50;i++){}

	HAL_GPIO_WritePin( ENCODER_L_CS_GPIO_Port, ENCODER_L_CS_Pin, GPIO_PIN_RESET); //cs = 0;

	data = data & 0xBFFF;//先頭から2つ目のbitを0に
	parity=0;
	for(int i=0;i<15;i++) parity += (data >> i) & 1;
	data = data | ((parity % 2) << 15);
	dataBuffer[0]=data>>8;
	dataBuffer[1]=data & 0x00FF;
	HAL_SPI_Transmit(&hspi3, (uint8_t*)dataBuffer, 2, 50);

	HAL_GPIO_WritePin( ENCODER_L_CS_GPIO_Port, ENCODER_L_CS_Pin, GPIO_PIN_SET); //cs = 1;

}



void AS5047_DataUpdate(void){

		//encoder_read_byte_L(0x3FFF,0xC000);
		//HAL_Delay(5);
		encoder_R=(float)(encoder_read_byte_R(0x3FFF,0x0000) & 0x3FFF) * 360 / 16384;
		//HAL_Delay(500);

		//encoder_read_byte_R(0x3FFF,0xC000);
		//HAL_Delay(5);
		encoder_L=(float)(encoder_read_byte_L(0x3FFF,0x0000) & 0x3FFF) * 360 / 16384;
		//HAL_Delay(5);

}
