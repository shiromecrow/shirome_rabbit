/*
 * PL_gyro.c
 *
 *  Created on: Dec 24, 2023
 *      Author: sf199
 */


/*
 * PL_gyro.c
 *
 *  Created on: Jan 5, 2023
 *      Author: sf199
 */


#include "PL_gyro.h"
#include "spi.h"
#include <stdio.h>


static uint8_t set_flag = 0;

GYRO_DATA gyro;


void SPI_Communication(SPI_TypeDef *SPIx ,uint8_t *tx_data, uint8_t *rx_data, uint8_t length, GPIO_TypeDef *GPIOx, uint32_t CS_Pin)
{
  uint8_t count = length;

  HAL_GPIO_WritePin( GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET); //cs = 1;

  if ( LL_SPI_IsActiveFlag_RXNE(SPIx) == SET ) LL_SPI_ReceiveData8(SPIx);
  if ( LL_SPI_IsEnabled(SPIx) == RESET ) LL_SPI_Enable(SPIx);

  while(count > 0){
    LL_SPI_TransmitData8(SPIx, *tx_data++);
    while( LL_SPI_IsActiveFlag_TXE(SPIx) == RESET );
    while( LL_SPI_IsActiveFlag_RXNE(SPIx) == RESET );
    *rx_data++ = LL_SPI_ReceiveData8(SPIx);
    count--;
  }

  HAL_GPIO_WritePin( GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET); //cs = 1;

}

uint8_t gyro_read_byte(uint8_t reg)

{

	  uint8_t tx_data[2];
	  uint8_t rx_data[2];

	  tx_data[0] = reg | 0x80;
	  tx_data[1] = 0x00;

	  SPI_Communication(SPI1, tx_data,rx_data, 2, GYRO_CS_GPIO_Port, GYRO_CS_Pin);

	  return rx_data[1];

}

void gyro_write_byte(uint8_t reg, uint8_t val)

{
	  uint8_t tx_data[2];
	  uint8_t rx_data[2];

	  tx_data[0] = reg & 0x7F;
	  tx_data[1] = val;

	  SPI_Communication(SPI1, tx_data,rx_data, 2, GYRO_CS_GPIO_Port, GYRO_CS_Pin);

}

void pl_gyro_init(void)

{

	uint8_t who_am_i = 0;

	LL_SPI_Enable(SPI1);

// check WHO_AM_I (0x75)

	who_am_i = gyro_read_byte(0x0f);

// who am i = 0x12

	printf("\r\nwho_am_i = 0x%x\r\n", who_am_i);

// recheck

	if (who_am_i != 0x6B) {

		HAL_Delay(100);

		who_am_i = gyro_read_byte(0x0f);

		if (who_am_i != 0x6B) {

			while (1) {

				printf("gyro_error\r");

			}

		}

	}

// set gyro config

// GYRO_CONFIG 0x1B

	gyro_write_byte(0x11, 0x80|0x01); // use 4000 dps

	HAL_Delay(50);

// ACCEL_CONFIG 0x1C

	gyro_write_byte(0x10, 0x80|0x0C); // use 4000 dps

	HAL_Delay(50);

	//gyro_write_byte(0x1D, 0x08);
	//HAL_Delay(50);

	set_flag = 1;

}

float ICM20602_GYRO_READ(uint8_t H_reg)

{

	int16_t data = (int16_t) (((uint8_t) gyro_read_byte(H_reg+1) << 8)
			| (uint8_t) gyro_read_byte(H_reg));

	float omega = (float)data * 140.0f/1000.0f;//dps変換 LSB * (mdps/LSB) * (dps/mdps)

	return omega;

}

float ICM20602_ACCEL_READ(uint8_t H_reg)

{

	int16_t data = (int16_t) (((uint8_t) gyro_read_byte(H_reg+1) << 8)
			| (uint8_t) gyro_read_byte(H_reg));

	float accel = (float)data*0.244*GRAVITATION;//mm/s^2変換 LSB * (mg/LSB) * 9.8(mg to mm/s^2)

	return accel;

}

void ICM20602_DataUpdate(void)

{

	if (set_flag == 1) {


		gyro.omega_x = ICM20602_GYRO_READ(0x22);

		gyro.omega_z = ICM20602_GYRO_READ(0x26);


		gyro.accel_y = ICM20602_ACCEL_READ(0x2A);


		//gyro.omega_y = ICM20602_GYRO_READ(0x24);
		//gyro.accel_x = ICM20602_ACCEL_READ(0x28);
		//gyro.accel_z = ICM20602_ACCEL_READ(0x2C);

	}

}
