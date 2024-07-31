/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"

#include "stm32g4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOTOR_R_CWCCW_Pin GPIO_PIN_13
#define MOTOR_R_CWCCW_GPIO_Port GPIOC
#define BLUE_R_LED_Pin GPIO_PIN_14
#define BLUE_R_LED_GPIO_Port GPIOC
#define SENSOR_LED2_Pin GPIO_PIN_15
#define SENSOR_LED2_GPIO_Port GPIOC
#define SENSOR6_Pin GPIO_PIN_0
#define SENSOR6_GPIO_Port GPIOA
#define SENSOR5_Pin GPIO_PIN_1
#define SENSOR5_GPIO_Port GPIOA
#define SENSOR4_Pin GPIO_PIN_2
#define SENSOR4_GPIO_Port GPIOA
#define SENSOR3_Pin GPIO_PIN_3
#define SENSOR3_GPIO_Port GPIOA
#define SENSOR_LED3_Pin GPIO_PIN_4
#define SENSOR_LED3_GPIO_Port GPIOA
#define GYRO_CS_Pin GPIO_PIN_4
#define GYRO_CS_GPIO_Port GPIOC
#define SENSOR2_Pin GPIO_PIN_0
#define SENSOR2_GPIO_Port GPIOB
#define SENSOR1_Pin GPIO_PIN_1
#define SENSOR1_GPIO_Port GPIOB
#define SENSOR_LED1_Pin GPIO_PIN_2
#define SENSOR_LED1_GPIO_Port GPIOB
#define BLUE_L_LED_Pin GPIO_PIN_10
#define BLUE_L_LED_GPIO_Port GPIOB
#define BATT_Pin GPIO_PIN_11
#define BATT_GPIO_Port GPIOB
#define LED8_Pin GPIO_PIN_12
#define LED8_GPIO_Port GPIOB
#define LED7_Pin GPIO_PIN_13
#define LED7_GPIO_Port GPIOB
#define LED6_Pin GPIO_PIN_14
#define LED6_GPIO_Port GPIOB
#define LED5_Pin GPIO_PIN_15
#define LED5_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_6
#define LED4_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_8
#define LED3_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_10
#define LED2_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_11
#define LED1_GPIO_Port GPIOA
#define FAN_MOTOR_Pin GPIO_PIN_12
#define FAN_MOTOR_GPIO_Port GPIOA
#define ENCODER_L_CS_Pin GPIO_PIN_15
#define ENCODER_L_CS_GPIO_Port GPIOA
#define ENCODER_R_CS_Pin GPIO_PIN_4
#define ENCODER_R_CS_GPIO_Port GPIOB
#define MOTOR_L_CWCCW_Pin GPIO_PIN_7
#define MOTOR_L_CWCCW_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
