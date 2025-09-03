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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "wing_module_config.h"
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

// Since we have two FIFOs, I'll use one for configuration messages and the other for commands to the module
void can_process_config_message(struct WingModuleConfig* config, CAN_HandleTypeDef* can);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_RED_Pin GPIO_PIN_0
#define LED_RED_GPIO_Port GPIOA
#define LED_WHITE_Pin GPIO_PIN_1
#define LED_WHITE_GPIO_Port GPIOA
#define ADS_RESET_Pin GPIO_PIN_2
#define ADS_RESET_GPIO_Port GPIOA
#define ADS_DATA_RDY_Pin GPIO_PIN_3
#define ADS_DATA_RDY_GPIO_Port GPIOA
#define ADS_CS_Pin GPIO_PIN_4
#define ADS_CS_GPIO_Port GPIOA
#define ANG_POT_Pin GPIO_PIN_0
#define ANG_POT_GPIO_Port GPIOB
#define ADDR_2_Pin GPIO_PIN_1
#define ADDR_2_GPIO_Port GPIOB
#define ADDR_1_Pin GPIO_PIN_2
#define ADDR_1_GPIO_Port GPIOB
#define ADC_CLK_IN_Pin GPIO_PIN_8
#define ADC_CLK_IN_GPIO_Port GPIOA
#define ADDR_0_Pin GPIO_PIN_15
#define ADDR_0_GPIO_Port GPIOA
#define ADDR_3_Pin GPIO_PIN_3
#define ADDR_3_GPIO_Port GPIOB
#define ADDR_4_Pin GPIO_PIN_4
#define ADDR_4_GPIO_Port GPIOB
#define SERVO_PWM_Pin GPIO_PIN_5
#define SERVO_PWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define LED_RED_CHANNEL TIM_CHANNEL_1
#define LED_WHITE_CHANNEL TIM_CHANNEL_2
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
