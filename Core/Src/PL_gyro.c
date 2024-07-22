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

uint8_t gyro_read_byte(uint8_t reg)

{

	uint8_t ret, val;

	HAL_GPIO_WritePin( GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET); //cs = 0;

	ret = reg | 0x80;//先頭のbitを1に

	HAL_SPI_Transmit(&hspi1, &ret, 1, 100);

	HAL_SPI_Receive(&hspi1, &val, 1, 100);

	HAL_GPIO_WritePin( GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET); //cs = 1;

	return val;

}

void gyro_write_byte(uint8_t reg, uint8_t val)

{

	uint8_t ret;

	ret = reg & 0x7F;//先頭のbitを0に

	HAL_GPIO_WritePin( GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi1, &ret, 1, 100);

	HAL_SPI_Transmit(&hspi1, &val, 1, 100);

	HAL_GPIO_WritePin( GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);

}

void pl_gyro_init(void)

{

	uint8_t who_am_i = 0;

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
