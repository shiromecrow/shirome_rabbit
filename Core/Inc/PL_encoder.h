/*
 * PL_encoder.h
 *
 *  Created on: Dec 26, 2023
 *      Author: sf199
 */

#ifndef INC_PL_ENCODER_H_
#define INC_PL_ENCODER_H_

#include "stm32g491xx.h"

extern float encoder_R,encoder_L;

void pl_encoder_init();

void Encoder_Communication(SPI_TypeDef * ,uint8_t *, uint8_t *, uint8_t, GPIO_TypeDef *, uint32_t);

void AS5047_DataUpdate();

#endif /* INC_PL_ENCODER_H_ */
