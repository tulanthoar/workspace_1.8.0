/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file         stm32h7xx_hal_msp.c
 * @brief        This file provides code for the MSP Initialization
 *               and de-Initialization codes.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

//SPI1 is the breadboard converter
extern DMA_HandleTypeDef hdma_spi1_rx;
// SPI2 is the PCB converter
extern DMA_HandleTypeDef hdma_spi2_rx;

//UART1 is the FT232 interface
extern DMA_HandleTypeDef hdma_usart1_tx;
//UART3 is the STLINK interface
extern DMA_HandleTypeDef hdma_usart3_tx;

// values for the pll configuration, see ioc file for values
const unsigned short pll3_m = 4;
const unsigned short pll3_n = 368;
const unsigned short pll3_p = 9;
const unsigned short pll3_q = 8;
const unsigned short pll3_r = 2;

const unsigned short pll2_m = 4;
const unsigned short pll2_n = 280;
const unsigned short pll2_p = 8;
const unsigned short pll2_q = 2;
const unsigned short pll2_r = 2;

/**
 * Initializes the Global MSP.
 */
void HAL_MspInit(void) {
	__HAL_RCC_SYSCFG_CLK_ENABLE();
}

/**
 * @brief SPI MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hspi: SPI handle pointer
 * @retval None
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };
	if (hspi->Instance == SPI1) {

//		configure clock for SPI1
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI1;
//		clock source is pll2p
		PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
//		configure m, n, p, q, r according to ioc file
		PeriphClkInitStruct.PLL2.PLL2M = pll2_m;
		PeriphClkInitStruct.PLL2.PLL2N = pll2_n;
		PeriphClkInitStruct.PLL2.PLL2P = pll2_p;
		PeriphClkInitStruct.PLL2.PLL2Q = pll2_q;
		PeriphClkInitStruct.PLL2.PLL2R = pll2_r;
		//		clock input is 2.08 MHz
		PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_1;
		//		use high frequency VCO mode
		PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
//		no fractional N
		PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
//		use HAL library to configure peripheral clock
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
			Error_Handler();
		}

		/* Peripheral clock enable */
		__HAL_RCC_SPI1_CLK_ENABLE();

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**SPI1 GPIO Configuration
		 PA5     ------> SPI1_SCK
		 PA6     ------> SPI1_MISO
		 PA15 (JTDI)     ------> SPI1_NSS
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_15;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
//		set high frequency
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* SPI1 DMA Init */
		/* SPI1_RX Init */
		hdma_spi1_rx.Instance = SPI1_DMA_INSTANCE;
		hdma_spi1_rx.Init.Request = DMA_REQUEST_SPI1_RX;
//		transfer from SPI peripheral to memory
		hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
//		do not increment peripheral address
		hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
//		do increment memory address
		hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
//		peripheral data aligned to half word
		hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
//		memory aligned to half word
		hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
//		circular mode (upon completion it loops to beginning)
		hdma_spi1_rx.Init.Mode = DMA_CIRCULAR;
//		low priority
		hdma_spi1_rx.Init.Priority = DMA_PRIORITY_LOW;
//		no fifo
		hdma_spi1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		hdma_spi1_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
		hdma_spi1_rx.Init.MemBurst = DMA_MBURST_SINGLE;
		hdma_spi1_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;
//		use HAL to configure DMA
		if (HAL_DMA_Init(&hdma_spi1_rx) != HAL_OK) {
			Error_Handler();
		}
		__HAL_LINKDMA(hspi, hdmarx, hdma_spi1_rx);

		/* SPI1 interrupt Init */
		HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(SPI1_IRQn);
	}
	if (hspi->Instance == SPI2) {

		/** Initializes the peripherals clock
		 */
		//		configure clock for SPI1
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
		//		clock source is pll2p
		PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
		//		configure m, n, p, q, r according to ioc file
		PeriphClkInitStruct.PLL2.PLL2M = pll2_m;
		PeriphClkInitStruct.PLL2.PLL2N = pll2_n;
		PeriphClkInitStruct.PLL2.PLL2P = pll2_p;
		PeriphClkInitStruct.PLL2.PLL2Q = pll2_q;
		PeriphClkInitStruct.PLL2.PLL2R = pll2_r;
		//		clock input is 2.08 MHz
		PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_1;
		//		use high frequency VCO mode
		PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
		//		no fractional N
		PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
		//		use HAL library to configure peripheral clock
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
			Error_Handler();
		}

		/* Peripheral clock enable */
		__HAL_RCC_SPI2_CLK_ENABLE();

		__HAL_RCC_GPIOC_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
		__HAL_RCC_GPIOD_CLK_ENABLE();
		/**SPI2 GPIO Configuration
		 PC2_C     ------> SPI2_MISO
		 PC3_C     ------> SPI2_MOSI
		 PB12     ------> SPI2_NSS
		 PD3     ------> SPI2_SCK
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		//		set high frequency
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_12;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_3;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

		/* SPI2 DMA Init */
		/* SPI2_RX Init */
		hdma_spi2_rx.Instance = SPI2_DMA_INSTANCE;
		hdma_spi2_rx.Init.Request = DMA_REQUEST_SPI2_RX;
//		transfer from SPI peripheral to memory
		hdma_spi2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
//		do not increment peripheral address
		hdma_spi2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
//		do increment memory address
		hdma_spi2_rx.Init.MemInc = DMA_MINC_ENABLE;
//		peripheral data aligned to half word
		hdma_spi2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
//		memory aligned to half word
		hdma_spi2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
//		circular mode (upon completion it loops to beginning)
		hdma_spi2_rx.Init.Mode = DMA_CIRCULAR;
//		low priority
		hdma_spi2_rx.Init.Priority = DMA_PRIORITY_LOW;
//		no fifo
		hdma_spi2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		hdma_spi2_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
		hdma_spi2_rx.Init.MemBurst = DMA_MBURST_SINGLE;
		hdma_spi2_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;
//		use HAL to configure DMA
		if (HAL_DMA_Init(&hdma_spi2_rx) != HAL_OK) {
			Error_Handler();
		}
		__HAL_LINKDMA(hspi, hdmarx, hdma_spi2_rx);

		/* SPI2 interrupt Init */
		HAL_NVIC_SetPriority(SPI2_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(SPI2_IRQn);
	}

}

/**
 * @brief SPI MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * Does not deinit all of the resources
 * @param hspi: SPI handle pointer
 * @retval None
 */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == SPI2) {
		/* USER CODE BEGIN SPI2_MspDeInit 0 */

		/* USER CODE END SPI2_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_SPI2_CLK_DISABLE();

		/**SPI2 GPIO Configuration
		 PC2_C     ------> SPI2_MISO
		 PC3_C     ------> SPI2_MOSI
		 PB12     ------> SPI2_NSS
		 PD3     ------> SPI2_SCK
		 */
		HAL_GPIO_DeInit(GPIOC, GPIO_PIN_2 | GPIO_PIN_3);

		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12);

		HAL_GPIO_DeInit(GPIOD, GPIO_PIN_3);

		/* USER CODE BEGIN SPI2_MspDeInit 1 */

		/* USER CODE END SPI2_MspDeInit 1 */
	}

}

/**
 * @brief UART MSP Initialization
 * This function configures the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };
	if (huart->Instance == USART1) {
		/** Initializes the peripherals clock
		 */
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
//		peripheral uses pll3q
		PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_PLL3;
//		set m, n, p, q, r according to ioc file
		PeriphClkInitStruct.PLL3.PLL3M = pll3_m;
		PeriphClkInitStruct.PLL3.PLL3N = pll3_n;
		PeriphClkInitStruct.PLL3.PLL3P = pll3_p;
		PeriphClkInitStruct.PLL3.PLL3Q = pll3_q;
		PeriphClkInitStruct.PLL3.PLL3R = pll3_r;
//		input clock speed is 2.08 MHz
		PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_1;
//		use high frequency VCO mode
		PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
//		no fractional n
		PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
//		use HAL library to configure peripheral clock
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
			Error_Handler();
		}

		/* Peripheral clock enable */
		__HAL_RCC_USART1_CLK_ENABLE();

		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**USART1 GPIO Configuration
		 PB15     ------> USART1_RX
		 PB6     ------> USART1_TX
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_15;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
//		low frequency
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
		GPIO_InitStruct.Alternate = GPIO_AF4_USART1;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
//		low frequency
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* USART1 DMA Init */
		/* USART1_TX Init */
		hdma_usart1_tx.Instance = DMA1_Stream0;
		hdma_usart1_tx.Init.Request = DMA_REQUEST_USART1_TX;
//		set memory to peripheral transfer
		hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
//		do not increment peripheral memory address
		hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
//		increment memory address
		hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
//		align data by bytes
		hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//		align data by half word
		hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
//		non circular mode
		hdma_usart1_tx.Init.Mode = DMA_NORMAL;
//		low priority
		hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
//		do not use FIFO
		hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//		configure DMA with HAL library
		if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK) {
			Error_Handler();
		}
		__HAL_LINKDMA(huart, hdmatx, hdma_usart1_tx);


	} else if (huart->Instance == USART3) {
		/* USER CODE BEGIN USART3_MspInit 0 */
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART234578;
//		use PLL3Q clock
		PeriphClkInitStruct.Usart234578ClockSelection =
				RCC_USART234578CLKSOURCE_PLL3;
//		configure m, n, p, q, and r according to ioc file
		PeriphClkInitStruct.PLL3.PLL3M = pll3_m;
		PeriphClkInitStruct.PLL3.PLL3N = pll3_n;
		PeriphClkInitStruct.PLL3.PLL3P = pll3_p;
		PeriphClkInitStruct.PLL3.PLL3Q = pll3_q;
		PeriphClkInitStruct.PLL3.PLL3R = pll3_r;
//		input clock is 2.08 MHz
		PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_1;
//		use high frequency VCO
		PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
//		no fractional n
		PeriphClkInitStruct.PLL3.PLL3FRACN = 0;

//		configure peripheral clock with HAL library
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
			Error_Handler();
		}

		/* Peripheral clock enable */
		__HAL_RCC_USART3_CLK_ENABLE();

		__HAL_RCC_GPIOD_CLK_ENABLE();
		/**USART3 GPIO Configuration
		 PD8     ------> USART3_TX
		 PD9     ------> USART3_RX
		 */
		GPIO_InitStruct.Pin = STLINK_RX_Pin | STLINK_TX_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
//		low frequency
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

		/* USART3 DMA Init */
		/* USART3_TX Init */
		hdma_usart3_tx.Instance = USART3_DMA_INSTANCE;
//		use USART3 tx signal
		hdma_usart3_tx.Init.Request = DMA_REQUEST_USART3_TX;
//		transfer from memory to peripheral
		hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
//		do not increment peripheral address
		hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
//		increment memory address
		hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
//		align peripheral data by bytes
		hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//		align memory by half word
		hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
//		do not use circular mode
		hdma_usart3_tx.Init.Mode = DMA_NORMAL;
//		low priority
		hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
//		do not use FIFO
		hdma_usart3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//		initialize DMA using HAL library
		if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK) {
			Error_Handler();
		}
		__HAL_LINKDMA(huart, hdmatx, hdma_usart3_tx);

	}

}

/**
 * @brief UART MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * does not de init all peripherals
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		/* USER CODE BEGIN USART1_MspDeInit 0 */

		/* USER CODE END USART1_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_USART1_CLK_DISABLE();

		/**USART1 GPIO Configuration
		 PB15     ------> USART1_RX
		 PB6     ------> USART1_TX
		 */
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_15 | GPIO_PIN_6);

		/* USART1 DMA DeInit */
		HAL_DMA_DeInit(huart->hdmatx);
		/* USER CODE BEGIN USART1_MspDeInit 1 */

		/* USER CODE END USART1_MspDeInit 1 */
	} else if (huart->Instance == USART3) {
		/* USER CODE BEGIN USART3_MspDeInit 0 */

		/* USER CODE END USART3_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_USART3_CLK_DISABLE();

		/**USART3 GPIO Configuration
		 PD8     ------> USART3_TX
		 PD9     ------> USART3_RX
		 */
		HAL_GPIO_DeInit(GPIOD, STLINK_RX_Pin | STLINK_TX_Pin);

		/* USART3 DMA DeInit */
		HAL_DMA_DeInit(huart->hdmatx);
		/* USER CODE BEGIN USART3_MspDeInit 1 */

		/* USER CODE END USART3_MspDeInit 1 */
	}

}

