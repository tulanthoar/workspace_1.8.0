/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2021 STMicroelectronics.
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
#include "stm32h7xx_nucleo.h"

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
// user button on nucleo board
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
// led1 on nucleo board
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
// led2 on nucleo board
#define LD2_Pin GPIO_PIN_1
#define LD2_GPIO_Port GPIOE
// led3 nucleo board
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
// stlink rx and tx pins
#define STLINK_RX_Pin GPIO_PIN_8
#define STLINK_RX_GPIO_Port GPIOD
#define STLINK_TX_Pin GPIO_PIN_9
#define STLINK_TX_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */
// counts the number of elements in an array
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
// convenient definitions for UART1 DMA instance and IRQn
#define USART1_DMA_INSTANCE DMA1_Stream0
#define USART1_DMA_IRQN DMA1_Stream0_IRQn
// convenient definitions for UART3 DMA instance and IRQn
#define USART3_DMA_INSTANCE DMA1_Stream1
#define USART3_DMA_IRQN DMA1_Stream1_IRQn
// convenient definitions for SPI1 DMA instance and IRQn
#define SPI1_DMA_IRQN DMA2_Stream0_IRQn
#define SPI1_DMA_INSTANCE DMA2_Stream0
// convenient definitions for SPI2 DMA instance and IRQn
#define SPI2_DMA_IRQN DMA2_Stream1_IRQn
#define SPI2_DMA_INSTANCE DMA2_Stream1
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
