/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#define STATUS_LED_Pin GPIO_PIN_13
#define STATUS_LED_GPIO_Port GPIOC
#define BEEPER_Pin GPIO_PIN_14
#define BEEPER_GPIO_Port GPIOC
#define VOLTAGE_DIVIDER_Pin GPIO_PIN_0
#define VOLTAGE_DIVIDER_GPIO_Port GPIOA
#define PA1_Pin GPIO_PIN_1
#define PA1_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define SPI_MPU_NSS_Pin GPIO_PIN_4
#define SPI_MPU_NSS_GPIO_Port GPIOA
#define SPI_MPU_SCK_Pin GPIO_PIN_5
#define SPI_MPU_SCK_GPIO_Port GPIOA
#define SPI_MPU_MISO_Pin GPIO_PIN_6
#define SPI_MPU_MISO_GPIO_Port GPIOA
#define SPI_MPU_MOSI_Pin GPIO_PIN_7
#define SPI_MPU_MOSI_GPIO_Port GPIOA
#define ESC4_Pin GPIO_PIN_0
#define ESC4_GPIO_Port GPIOB
#define PB1_Pin GPIO_PIN_1
#define PB1_GPIO_Port GPIOB
#define SPI_RX_NSS_Pin GPIO_PIN_2
#define SPI_RX_NSS_GPIO_Port GPIOB
#define PB10_Pin GPIO_PIN_10
#define PB10_GPIO_Port GPIOB
#define SPI_OSD_NSS_Pin GPIO_PIN_12
#define SPI_OSD_NSS_GPIO_Port GPIOB
#define SPI_RX_SCK_Pin GPIO_PIN_13
#define SPI_RX_SCK_GPIO_Port GPIOB
#define SPI_RX_MISO_Pin GPIO_PIN_14
#define SPI_RX_MISO_GPIO_Port GPIOB
#define SPI_RX_MOSI_Pin GPIO_PIN_15
#define SPI_RX_MOSI_GPIO_Port GPIOB
#define ESC1_Pin GPIO_PIN_8
#define ESC1_GPIO_Port GPIOA
#define ESC2_Pin GPIO_PIN_9
#define ESC2_GPIO_Port GPIOA
#define ESC3_Pin GPIO_PIN_10
#define ESC3_GPIO_Port GPIOA
#define PA15_Pin GPIO_PIN_15
#define PA15_GPIO_Port GPIOA
#define PB3_Pin GPIO_PIN_3
#define PB3_GPIO_Port GPIOB
#define PB4_Pin GPIO_PIN_4
#define PB4_GPIO_Port GPIOB
#define PB5_Pin GPIO_PIN_5
#define PB5_GPIO_Port GPIOB
#define PB6_Pin GPIO_PIN_6
#define PB6_GPIO_Port GPIOB
#define PB7_Pin GPIO_PIN_7
#define PB7_GPIO_Port GPIOB
#define PB8_Pin GPIO_PIN_8
#define PB8_GPIO_Port GPIOB
#define PB9_Pin GPIO_PIN_9
#define PB9_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
