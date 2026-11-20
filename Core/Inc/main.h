/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define AXIS0_RPWM_Pin GPIO_PIN_0
#define AXIS0_RPWM_GPIO_Port GPIOC
#define AXIS0_LPWM_Pin GPIO_PIN_1
#define AXIS0_LPWM_GPIO_Port GPIOC
#define AXIS1_RPWM_Pin GPIO_PIN_2
#define AXIS1_RPWM_GPIO_Port GPIOC
#define AXIS1_LPWM_Pin GPIO_PIN_3
#define AXIS1_LPWM_GPIO_Port GPIOC
#define AXIS1_EN_Pin GPIO_PIN_2
#define AXIS1_EN_GPIO_Port GPIOF
#define UNDERWATER_Pin GPIO_PIN_0
#define UNDERWATER_GPIO_Port GPIOB
#define SPOTLIGHT_Pin GPIO_PIN_1
#define SPOTLIGHT_GPIO_Port GPIOB
#define LIGHTBAR_Pin GPIO_PIN_2
#define LIGHTBAR_GPIO_Port GPIOB
#define TURN_ON_SONAR_Pin GPIO_PIN_8
#define TURN_ON_SONAR_GPIO_Port GPIOE
#define DIR_LOCK_DRIVER_1_Pin GPIO_PIN_10
#define DIR_LOCK_DRIVER_1_GPIO_Port GPIOE
#define SGU_LIGHT_Pin GPIO_PIN_12
#define SGU_LIGHT_GPIO_Port GPIOB
#define LOCK1_Pin GPIO_PIN_13
#define LOCK1_GPIO_Port GPIOB
#define LOCK2_Pin GPIO_PIN_15
#define LOCK2_GPIO_Port GPIOB
#define RELEASE2_DEPLOYED_Pin GPIO_PIN_15
#define RELEASE2_DEPLOYED_GPIO_Port GPIOD
#define FLASHER_BLUE_Pin GPIO_PIN_6
#define FLASHER_BLUE_GPIO_Port GPIOC
#define FLASHER_YELLOW_Pin GPIO_PIN_7
#define FLASHER_YELLOW_GPIO_Port GPIOC
#define IS_BOUY_2_Pin GPIO_PIN_8
#define IS_BOUY_2_GPIO_Port GPIOA
#define EN_LOCK_DRIVER_2_Pin GPIO_PIN_10
#define EN_LOCK_DRIVER_2_GPIO_Port GPIOA
#define EN_LOCK_DRIVER_1_Pin GPIO_PIN_15
#define EN_LOCK_DRIVER_1_GPIO_Port GPIOA
#define DIR_LOCK_DRIVER_2_Pin GPIO_PIN_0
#define DIR_LOCK_DRIVER_2_GPIO_Port GPIOD
#define IS_BUOY_1_Pin GPIO_PIN_1
#define IS_BUOY_1_GPIO_Port GPIOD
#define ISENSE_L_Pin GPIO_PIN_7
#define ISENSE_L_GPIO_Port GPIOB
#define AXIS0_EN_Pin GPIO_PIN_9
#define AXIS0_EN_GPIO_Port GPIOB
#define ISENSE_R_Pin GPIO_PIN_0
#define ISENSE_R_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
