/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum
{
    LAUNCH_WAIT,
    ASCENT,
    ROCKET_SEPARATION,
    DESCENT,
    HS_RELEASE,
    LANDED
} state_t;

typedef enum
{
	MAIN_CAM = 1,
	BONUS_CAM,
	CAM_OFF
} cam_t;
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
#define LED4_Pin GPIO_PIN_9
#define LED4_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_10
#define LED1_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_11
#define LED2_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_12
#define LED3_GPIO_Port GPIOE
#define BUZZER_Pin GPIO_PIN_12
#define BUZZER_GPIO_Port GPIOB
#define CAM1_Pin GPIO_PIN_5
#define CAM1_GPIO_Port GPIOB
#define CAM2_Pin GPIO_PIN_6
#define CAM2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define TEAM_ID			2032
#define PACKETCOUNT_ADR 0x00
#define REFALT_ADR		0x10
#define STATEIND_ADR 	0x30
#define FLAGTEL_ADR		0x38
#define HSDEPLOY_ADR 	0x40
#define PCDEPLOY_ADR 	0x48
#define BNO055CAL_ADR	0x50

#define FREERTOS_ENABLED
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
