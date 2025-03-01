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


void pl_encoder_init(void)
{

	LL_SPI_Enable(SPI3);

}

void Encoder_Communication(SPI_TypeDef *SPIx ,uint8_t *tx_data, uint8_t *rx_data, uint8_t length, GPIO_TypeDef *GPIOx, uint32_t CS_Pin)
{
  uint8_t count = length;

  HAL_GPIO_WritePin( GPIOx, CS_Pin, GPIO_PIN_RESET); //cs = 0;

  if ( LL_SPI_IsActiveFlag_RXNE(SPIx) == SET ) LL_SPI_ReceiveData8(SPIx);
  if ( LL_SPI_IsEnabled(SPIx) == RESET ) LL_SPI_Enable(SPIx);

  while(count > 0){
    LL_SPI_TransmitData8(SPIx, *tx_data++);
    while( LL_SPI_IsActiveFlag_TXE(SPIx) == RESET );
    while( LL_SPI_IsActiveFlag_RXNE(SPIx) == RESET );
    *rx_data++ = LL_SPI_ReceiveData8(SPIx);
    count--;
  }

  HAL_GPIO_WritePin( GPIOx, CS_Pin, GPIO_PIN_SET); //cs = 1;

}

uint16_t encoder_read_byte_R(uint16_t address){

	uint8_t addBuffer[2];
	uint16_t data;
	uint8_t dataBuffer[2];
	uint16_t parity;
	volatile int i = 0;

	address = address | 0x4000;//先頭から2つ目のbitを1に
	parity=0;
	for(i=0;i<15;i++) parity += (address >> i) & 1;
	address = address | ((parity % 2) << 15);
	addBuffer[0]=address >> 8;
	addBuffer[1]=address & 0x00FF;

	Encoder_Communication(SPI3, (uint8_t*)addBuffer,(uint8_t*)dataBuffer, 2,ENCODER_R_CS_GPIO_Port, ENCODER_R_CS_Pin);
	
	for(i=0;i<50;i++){}

	addBuffer[0]=0 >> 8;
	addBuffer[1]=0 & 0x00FF;
	Encoder_Communication(SPI3, (uint8_t*)addBuffer,(uint8_t*)dataBuffer, 2,ENCODER_R_CS_GPIO_Port, ENCODER_R_CS_Pin);
	data=((uint16_t)(dataBuffer[0]) << 8) | (uint16_t)(dataBuffer[1]);

	return data;

}


uint16_t encoder_read_byte_L(uint16_t address){

	uint8_t addBuffer[2];
	uint16_t data;
	uint8_t dataBuffer[2];
	uint16_t parity;
	volatile int i = 0;

	address = address | 0x4000;//先頭から2つ目のbitを1に
	parity=0;
	for(i=0;i<15;i++) parity += (address >> i) & 1;
	address = address | ((parity % 2) << 15);
	addBuffer[0]=address >> 8;
	addBuffer[1]=address & 0x00FF;

	Encoder_Communication(SPI3, (uint8_t*)addBuffer,(uint8_t*)dataBuffer, 2,ENCODER_L_CS_GPIO_Port, ENCODER_L_CS_Pin);
	
	for(i=0;i<50;i++){}

	addBuffer[0]=0 >> 8;
	addBuffer[1]=0 & 0x00FF;
	Encoder_Communication(SPI3, (uint8_t*)addBuffer,(uint8_t*)dataBuffer, 2,ENCODER_L_CS_GPIO_Port, ENCODER_L_CS_Pin);
	data=((uint16_t)(dataBuffer[0]) << 8) | (uint16_t)(dataBuffer[1]);

	return data;

}


void AS5047_DataUpdate(void){

		//encoder_read_byte_L(0x3FFF,0xC000);
		//HAL_Delay(5);
		encoder_R=(float)(encoder_read_byte_R(0x3FFF) & 0x3FFF) * 360 / 16384;
		//HAL_Delay(500);

		//encoder_read_byte_R(0x3FFF,0xC000);
		//HAL_Delay(5);
		encoder_L=(float)(encoder_read_byte_L(0x3FFF) & 0x3FFF) * 360 / 16384;
		//HAL_Delay(5);

}
