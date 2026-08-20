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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU6050_I2C.h"
#include "OLED_SPI.h"
#include "scheduler.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum
{
  TASK_LASER = 0,
  TASK_ROTARY_ENCODER,
  TASK_READ_MPU,
  TASK_ZERO_GUIDEWAY,
  TASK_GET_LUT,
  TASK_BALL_STAB,
  TASK_DISPLAY_UART,
  TASK_ADC,
  TASK_GET_BUTTON,
  TASK_COUNT /* 自动等于任务个数 */
} TaskId_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern SPI_HandleTypeDef hspi2;
// extern MPU6050_t MPU6050;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3; // 本项目中张大头连接uart3

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
HAL_StatusTypeDef UART_DMA_printf(UART_HandleTypeDef *huart, const char *fmt, ...);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BTN_Pin GPIO_PIN_1
#define BTN_GPIO_Port GPIOA
#define OLED_SPI_DC_Pin GPIO_PIN_12
#define OLED_SPI_DC_GPIO_Port GPIOB
#define OLED_SPI_RES_Pin GPIO_PIN_14
#define OLED_SPI_RES_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
