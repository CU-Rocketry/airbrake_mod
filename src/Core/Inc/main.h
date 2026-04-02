/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EYE_0_Pin GPIO_PIN_4
#define EYE_0_GPIO_Port GPIOE
#define EYE_1_Pin GPIO_PIN_5
#define EYE_1_GPIO_Port GPIOE
#define BATT_VSENSE_Pin GPIO_PIN_0
#define BATT_VSENSE_GPIO_Port GPIOC
#define BATT_ISENSE_Pin GPIO_PIN_1
#define BATT_ISENSE_GPIO_Port GPIOC
#define SERVO_FDBK_Pin GPIO_PIN_2
#define SERVO_FDBK_GPIO_Port GPIOC
#define BARO_NSS_Pin GPIO_PIN_4
#define BARO_NSS_GPIO_Port GPIOA
#define BARO_SCK_Pin GPIO_PIN_5
#define BARO_SCK_GPIO_Port GPIOA
#define BARO_MISO_Pin GPIO_PIN_6
#define BARO_MISO_GPIO_Port GPIOA
#define BARO_MOSI_Pin GPIO_PIN_7
#define BARO_MOSI_GPIO_Port GPIOA
#define BARO_INT_Pin GPIO_PIN_4
#define BARO_INT_GPIO_Port GPIOC
#define BARO_INT_EXTI_IRQn EXTI4_IRQn
#define MAG_INT_Pin GPIO_PIN_10
#define MAG_INT_GPIO_Port GPIOE
#define MAG_INT_EXTI_IRQn EXTI15_10_IRQn
#define MAG_NSS_Pin GPIO_PIN_11
#define MAG_NSS_GPIO_Port GPIOE
#define IMU_NSS_Pin GPIO_PIN_12
#define IMU_NSS_GPIO_Port GPIOB
#define IMU_INT1_Pin GPIO_PIN_8
#define IMU_INT1_GPIO_Port GPIOD
#define IMU_INT2_Pin GPIO_PIN_9
#define IMU_INT2_GPIO_Port GPIOD
#define BTN_0_Pin GPIO_PIN_0
#define BTN_0_GPIO_Port GPIOD
#define BTN_1_Pin GPIO_PIN_1
#define BTN_1_GPIO_Port GPIOD
#define BTN_2_Pin GPIO_PIN_2
#define BTN_2_GPIO_Port GPIOD
#define BTN_3_Pin GPIO_PIN_3
#define BTN_3_GPIO_Port GPIOD
#define MODE_1_Pin GPIO_PIN_4
#define MODE_1_GPIO_Port GPIOD
#define MODE_C_Pin GPIO_PIN_5
#define MODE_C_GPIO_Port GPIOD
#define MODE_2_Pin GPIO_PIN_6
#define MODE_2_GPIO_Port GPIOD
#define MODE_4_Pin GPIO_PIN_7
#define MODE_4_GPIO_Port GPIOD
#define BUZZER_Pin GPIO_PIN_8
#define BUZZER_GPIO_Port GPIOB
#define SERVO_SIG_Pin GPIO_PIN_9
#define SERVO_SIG_GPIO_Port GPIOB
#define SERVO_EN_Pin GPIO_PIN_0
#define SERVO_EN_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
