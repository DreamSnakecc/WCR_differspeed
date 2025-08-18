/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
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
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

//水浸传感器
#define WT_SENSE1_GPIO_PIN      GPIO_PIN_5
#define WT_SENSE1_GPIO_PORT     GPIOE
#define WT_SENSE2_GPIO_PIN      GPIO_PIN_4
#define WT_SENSE2_GPIO_PORT     GPIOE

#define WT_SENSE3_GPIO_PIN      GPIO_PIN_0
#define WT_SENSE3_GPIO_PORT     GPIOF
#define WT_SENSE4_GPIO_PIN      GPIO_PIN_1
#define WT_SENSE4_GPIO_PORT     GPIOF

#define WT_SENSE5_GPIO_PIN      GPIO_PIN_12
#define WT_SENSE5_GPIO_PORT     GPIOE
#define WT_SENSE6_GPIO_PIN      GPIO_PIN_6
#define WT_SENSE6_GPIO_PORT     GPIOE

//行程开关
#define Switch_Sense1_GPIO_PIN  	GPIO_PIN_0
#define Switch_Sense1_GPIO_PORT  	GPIOB

#define Switch_Sense2_GPIO_PIN  	GPIO_PIN_2
#define Switch_Sense2_GPIO_PORT  	GPIOC

#define Switch_Sense3_GPIO_PIN  	GPIO_PIN_1
#define Switch_Sense3_GPIO_PORT  	GPIOB

#define Switch_Sense4_GPIO_PIN  	GPIO_PIN_3
#define Switch_Sense4_GPIO_PORT  	GPIOC

#define Switch_Sense5_GPIO_PIN  	GPIO_PIN_0
#define Switch_Sense5_GPIO_PORT  	GPIOC

#define Switch_Sense6_GPIO_PIN  	GPIO_PIN_4
#define Switch_Sense6_GPIO_PORT  	GPIOC

#define Switch_Sense7_GPIO_PIN  	GPIO_PIN_1
#define Switch_Sense7_GPIO_PORT  	GPIOC

#define Switch_Sense8_GPIO_PIN  	GPIO_PIN_5
#define Switch_Sense8_GPIO_PORT  	GPIOC

#define Switch_Sense9_GPIO_PIN  	GPIO_PIN_9
#define Switch_Sense9_GPIO_PORT  	GPIOI

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

