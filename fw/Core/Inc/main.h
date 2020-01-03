/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32g4xx_ll_pwr.h"

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
#define M_UL_Pin GPIO_PIN_13
#define M_UL_GPIO_Port GPIOC
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOF
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOF
#define BUS_VOLTAGE_Pin GPIO_PIN_0
#define BUS_VOLTAGE_GPIO_Port GPIOA
#define BEMF1_Pin GPIO_PIN_4
#define BEMF1_GPIO_Port GPIOA
#define BEMF2_Pin GPIO_PIN_4
#define BEMF2_GPIO_Port GPIOC
#define BEMF3_Pin GPIO_PIN_11
#define BEMF3_GPIO_Port GPIOB
#define POTI_Pin GPIO_PIN_12
#define POTI_GPIO_Port GPIOB
#define TEMP_Pin GPIO_PIN_14
#define TEMP_GPIO_Port GPIOB
#define M_WL_Pin GPIO_PIN_15
#define M_WL_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_6
#define LED1_GPIO_Port GPIOC
#define M_UH_Pin GPIO_PIN_8
#define M_UH_GPIO_Port GPIOA
#define M_VH_Pin GPIO_PIN_9
#define M_VH_GPIO_Port GPIOA
#define M_WH_Pin GPIO_PIN_10
#define M_WH_GPIO_Port GPIOA
#define CAN_RX_Pin GPIO_PIN_11
#define CAN_RX_GPIO_Port GPIOA
#define M_VL_Pin GPIO_PIN_12
#define M_VL_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define BUTTON_Pin GPIO_PIN_10
#define BUTTON_GPIO_Port GPIOC
#define M_HA_Pin GPIO_PIN_6
#define M_HA_GPIO_Port GPIOB
#define M_HB_Pin GPIO_PIN_7
#define M_HB_GPIO_Port GPIOB
#define M_HC_Pin GPIO_PIN_8
#define M_HC_GPIO_Port GPIOB
#define CAN_TX_Pin GPIO_PIN_9
#define CAN_TX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
