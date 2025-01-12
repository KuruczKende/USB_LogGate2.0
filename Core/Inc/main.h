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
#include "stm32f4xx_hal.h"

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
#define SEG0_Pin GPIO_PIN_2
#define SEG0_GPIO_Port GPIOE
#define SEG1_Pin GPIO_PIN_3
#define SEG1_GPIO_Port GPIOE
#define SEG2_Pin GPIO_PIN_4
#define SEG2_GPIO_Port GPIOE
#define SEG3_Pin GPIO_PIN_5
#define SEG3_GPIO_Port GPIOE
#define WK_UP_Pin GPIO_PIN_0
#define WK_UP_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOA
#define SEGB_Pin GPIO_PIN_7
#define SEGB_GPIO_Port GPIOE
#define SEGC_Pin GPIO_PIN_8
#define SEGC_GPIO_Port GPIOE
#define SEGG_Pin GPIO_PIN_9
#define SEGG_GPIO_Port GPIOE
#define SEGDP_Pin GPIO_PIN_10
#define SEGDP_GPIO_Port GPIOE
#define SEGF_Pin GPIO_PIN_11
#define SEGF_GPIO_Port GPIOE
#define SEGD_Pin GPIO_PIN_12
#define SEGD_GPIO_Port GPIOE
#define SEGA_Pin GPIO_PIN_13
#define SEGA_GPIO_Port GPIOE
#define SEGE_Pin GPIO_PIN_14
#define SEGE_GPIO_Port GPIOE
#define SW0_Pin GPIO_PIN_9
#define SW0_GPIO_Port GPIOD
#define BT_SW_EN_Pin GPIO_PIN_10
#define BT_SW_EN_GPIO_Port GPIOD
#define SW1_Pin GPIO_PIN_11
#define SW1_GPIO_Port GPIOD
#define BT0_Pin GPIO_PIN_12
#define BT0_GPIO_Port GPIOD
#define SW2_Pin GPIO_PIN_13
#define SW2_GPIO_Port GPIOD
#define BT1_Pin GPIO_PIN_14
#define BT1_GPIO_Port GPIOD
#define SW3_Pin GPIO_PIN_15
#define SW3_GPIO_Port GPIOD
#define BT2_Pin GPIO_PIN_6
#define BT2_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
#define K1_Pin WK_UP_Pin
#define K1_GPIO_Port WK_UP_GPIO_Port

void vAlterSegDisp(uint16_t u16Reset, uint16_t u16Set);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
