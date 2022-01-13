/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "string.h"

//SPI1 is the breadboard converter
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
// SPI2 is the PCB converter
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;

//UART1 is the FT232 interface
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
//UART3 is the STLINK interface
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

// configure the system clock
void SystemClock_Config(void);
// initialize the GPIO
static void MX_GPIO_Init(void);
// initialize the DMA controllers
static void MX_DMA_Init(void);
// initialize the breadboard converter
static void MX_SPI1_Init(void);
// initialize the PCB converter
static void MX_SPI2_Init(void);
// initialize the FT232 UART
static void MX_USART1_UART_Init(void);
// initialize the STLINK UART
static void MX_USART3_UART_Init(void);
// indicate the SPI transfer is half complete,
// used in the first cycle before interrupts are disabled
static void tx_h_complete(DMA_HandleTypeDef *hdma);

// variable set in the tx_h_complete function
enum {
	TRANSFER_WAIT, TRANSFER_H_COMPLETE, TRANSFER_ERROR
};
__IO uint32_t wTransferState = TRANSFER_WAIT;

// the value of this variable is toggled when the user button is pressed
__IO uint32_t UserButtonStatus = 0;
// buffer used to transmit data over UART
ALIGN_32BYTES (__IO uint16_t aTxBuffer[1024]) = {0};
// buffer used to receive data over SPI
ALIGN_32BYTES (__IO uint16_t aRxBuffer[8202]) = {0};
// array to store output values of iir filter, not currently implimented
float yi[8202] = { 0 };

// The oversampling ratio
#define OVERSAMPLING 4

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
//	length of the recieve buffer array
	unsigned short rxCount = COUNTOF(aRxBuffer) - 10;
//	length of transmit buffer
	unsigned short txCount = COUNTOF(aTxBuffer);
//	set the transmit buffer to a known value
	for (int i = 0; i < txCount; ++i) {
		aTxBuffer[i] = i % 16384;
	}

	/* Enable I-Cache---------------------------------------------------------*/
	SCB_EnableICache();
//	Enabling the data cache gives incorrect results
//	  SCB_EnableDCache();

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

//    initialize GPIO
	MX_GPIO_Init();

//	turn off all the PCB LEDs
	GPIOE->BSRR = GPIO_PIN_0 | GPIO_PIN_13 | GPIO_PIN_15;
	GPIOF->BSRR = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15;
	GPIOG->BSRR = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_14;
//	initialize the compensation cell to improve slew rate
	HAL_EnableCompensationCell();
//	initialize the DMA, must be done before other peripherals
	MX_DMA_Init();
//	initialize UART1, going to the ft232 interface
	MX_USART1_UART_Init();
//	initialize URT3, going to the STLINK interface
	MX_USART3_UART_Init();

//	initialize SPI1 interface, going to the breadboard converter
	MX_SPI1_Init();
//	initialize the SPI2 interface, going to the pcb converter
    MX_SPI2_Init();

//  stall until the user button is pressed
	while (UserButtonStatus == 0) {
		BSP_LED_Toggle(LED1);
		HAL_Delay(100);
	}
//	reset the user button status
	UserButtonStatus = 0;
//	reset LEDS
	BSP_LED_Off(LED1);
	BSP_LED_Off(LED2);
	BSP_LED_Off(LED3);

	/* Configure communication direction : 1Line */
	SPI_1LINE_RX(&hspi1);
	SPI_1LINE_RX(&hspi2);

//	configure the half transfer callback function to update the wTransferState value
	hspi1.hdmarx->XferHalfCpltCallback = tx_h_complete;
	hspi2.hdmarx->XferHalfCpltCallback = tx_h_complete;

//	enable the transfer complete and half transfer interupts for SPI1
	SET_BIT(SPI1_DMA_INSTANCE->CR, DMA_IT_TC | DMA_IT_HT);
	SET_BIT(SPI2_DMA_INSTANCE->CR, DMA_IT_TC | DMA_IT_HT);

#ifdef USE_BREADBOARD
	start the DMA transfer on SPI1, use HAL library to perform initial configurations
	if (HAL_DMA_Start(hspi1.hdmarx, (uint32_t) &hspi1.Instance->RXDR,
					(uint32_t) aRxBuffer + 20, rxCount) != HAL_OK) {
//		if the DMA initalization was not OK, set the error bit
		SET_BIT(hspi1.ErrorCode, HAL_SPI_ERROR_DMA);
//		reset SPI ready state
		hspi1.State = HAL_SPI_STATE_READY;
//		call our custom error handler
		Error_Handler();
	}
#else
	//	start the DMA transfer on SPI2, use HAL library to perform initial configurations
		if (HAL_DMA_Start(hspi2.hdmarx, (uint32_t) &hspi2.Instance->RXDR,
						(uint32_t) aRxBuffer + 20, rxCount) != HAL_OK) {
	//		if the DMA initalization was not OK, set the error bit
			SET_BIT(hspi2.ErrorCode, HAL_SPI_ERROR_DMA);
	//		reset SPI ready state
			hspi2.State = HAL_SPI_STATE_READY;
	//		call our custom error handler
			Error_Handler();
		}
#endif

//	set the transfer size to 0 (unlimited)
	MODIFY_REG(hspi1.Instance->CR2, SPI_CR2_TSIZE, 0UL);
	MODIFY_REG(hspi2.Instance->CR2, SPI_CR2_TSIZE, 0UL);

//	enable DMA requests on the SPI instance
	SET_BIT(hspi1.Instance->CFG1, SPI_CFG1_RXDMAEN);
	SET_BIT(hspi2.Instance->CFG1, SPI_CFG1_RXDMAEN);

	/* Enable the SPI Error Interrupt Bit */
	__HAL_SPI_ENABLE_IT(&hspi1, (SPI_IT_OVR | SPI_IT_FRE | SPI_IT_MODF));
	__HAL_SPI_ENABLE_IT(&hspi2, (SPI_IT_OVR | SPI_IT_FRE | SPI_IT_MODF));

	/* Enable SPI peripheral */
	__HAL_SPI_ENABLE(&hspi1);
	__HAL_SPI_ENABLE(&hspi2);

//	start the SPI transfers
#ifdef USE_BREADBOARD
	SET_BIT(hspi1.Instance->CR1, SPI_CR1_CSTART);
#else
	SET_BIT(hspi2.Instance->CR1, SPI_CR1_CSTART);
#endif

//	wait for the first half of the transfer to complete
	while (wTransferState != TRANSFER_H_COMPLETE) {
//		toggle LED2 every 20 ms
		HAL_Delay(20);
		BSP_LED_Toggle(LED3);
	}
//	reset transfer state
	wTransferState = TRANSFER_WAIT;
//	reset LEDs
	BSP_LED_Off(LED1);
	BSP_LED_Off(LED2);
	BSP_LED_Off(LED3);
//	transfer data from rxbuffer to tx buffer
//	j is the index for the rx buffer
	int j = 10;
	aTxBuffer[0] = aRxBuffer[j];
//	i is the index of the tx buffer
//	j increases by the oversampling ratio for each inciment in i
	for (int i = 1; i < txCount; ++i) {
		j += OVERSAMPLING;
		aTxBuffer[i] = aRxBuffer[j];
	}
//	Use the HAL driver to transmit the buffer over DMA
//	HAL will initialize many of the settings for us
	if (HAL_UART_Transmit_DMA(&huart3, (uint8_t*) aTxBuffer, sizeof(aTxBuffer))
			!= HAL_OK) {
//		if it fails, call our error handler
		Error_Handler();
	}
	//	Use the HAL driver to transmit the buffer over DMA
	//	HAL will initialize many of the settings for us
		if (HAL_UART_Transmit_DMA(&huart1, (uint8_t*) aTxBuffer, sizeof(aTxBuffer))
				!= HAL_OK) {
	//		if it fails, call our error handler
			Error_Handler();
		}
//	Suspend interupts that we no longer need, for the purpose of efficiency
//	suspend the systick
	HAL_SuspendTick();
//	suspend UART3 interupts
	HAL_NVIC_DisableIRQ(USART3_IRQn);
	HAL_NVIC_DisableIRQ(USART1_IRQn);
//	suspend DMA interupts for the UART3 channel
	HAL_NVIC_DisableIRQ(USART3_DMA_IRQN);
	HAL_NVIC_DisableIRQ(USART1_DMA_IRQN);
//	suspend DMA interrupts for the SPI channel
	HAL_NVIC_DisableIRQ(SPI1_DMA_IRQN);
	HAL_NVIC_DisableIRQ(SPI2_DMA_IRQN);
//	reset LEDs
	BSP_LED_Off(LED1);
	BSP_LED_Off(LED2);
	BSP_LED_Off(LED3);

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
//		wait for the second half of the receive buffer to finish transferring
#ifdef USE_BREADBOARD
		while ((DMA2->LISR & DMA_FLAG_TCIF0_4) != DMA_FLAG_TCIF0_4) { }
#else
		while ((DMA2->LISR & DMA_FLAG_TCIF1_5) != DMA_FLAG_TCIF1_5) { }
#endif
//		clear the transfer complete flag of the SPI channel
		DMA2->LIFCR = DMA_FLAG_TCIF0_4 | DMA_FLAG_TCIF1_5;
//		the rx buffer index starts at half way through the buffer and goes to the end
		for (int i = 0; i < txCount; ++i) {

			yi[j] = (1.8862132469e-05) * aRxBuffer[j] + (7.5448529875e-05) * aRxBuffer[j-1] \
			+ (1.1317279481e-04) * aRxBuffer[j-2] + (7.5448529875e-05) * aRxBuffer[j-3] \
			+ (1.8862132469e-05) * aRxBuffer[j-4] \
			- (-3.6404314407e+00) * yi[j-1] - (4.9845340532e+00) * yi[j-2] \
			- (-3.0414501690e+00) * yi[j-3] - (6.9764935070e-01) * yi[j-4];
			++j;
			yi[j] = (1.8862132469e-05) * aRxBuffer[j] + (7.5448529875e-05) * aRxBuffer[j-1] \
			+ (1.1317279481e-04) * aRxBuffer[j-2] + (7.5448529875e-05) * aRxBuffer[j-3] \
			+ (1.8862132469e-05) * aRxBuffer[j-4] \
			- (-3.6404314407e+00) * yi[j-1] - (4.9845340532e+00) * yi[j-2] \
			- (-3.0414501690e+00) * yi[j-3] - (6.9764935070e-01) * yi[j-4];
			++j;
			yi[j] = (1.8862132469e-05) * aRxBuffer[j] + (7.5448529875e-05) * aRxBuffer[j-1] \
			+ (1.1317279481e-04) * aRxBuffer[j-2] + (7.5448529875e-05) * aRxBuffer[j-3] \
			+ (1.8862132469e-05) * aRxBuffer[j-4] \
			- (-3.6404314407e+00) * yi[j-1] - (4.9845340532e+00) * yi[j-2] \
			- (-3.0414501690e+00) * yi[j-3] - (6.9764935070e-01) * yi[j-4];
			++j;
			yi[j] = (1.8862132469e-05) * aRxBuffer[j] + (7.5448529875e-05) * aRxBuffer[j-1] \
			+ (1.1317279481e-04) * aRxBuffer[j-2] + (7.5448529875e-05) * aRxBuffer[j-3] \
			+ (1.8862132469e-05) * aRxBuffer[j-4] \
			- (-3.6404314407e+00) * yi[j-1] - (4.9845340532e+00) * yi[j-2] \
			- (-3.0414501690e+00) * yi[j-3] - (6.9764935070e-01) * yi[j-4];

			aTxBuffer[i] = (uint16_t)yi[j];
			++j;
		}
		yi[9] = yi[j-1];
		aRxBuffer[9] = aRxBuffer[j-1];
		yi[8] = yi[j-2];
		aRxBuffer[8] = aRxBuffer[j-2];
		yi[7] = yi[j-3];
		aRxBuffer[7] = aRxBuffer[j-3];
		yi[6] = yi[j-4];
		aRxBuffer[6] = aRxBuffer[j-4];


//		wait for the UARTs to finish transferring
		while ((USART3->ISR & UART_FLAG_TC) != UART_FLAG_TC) { }
		while ((USART1->ISR & UART_FLAG_TC) != UART_FLAG_TC) { }
//		reset the UARTs transfer complete flag
		USART3->ICR = UART_CLEAR_TCF;
		USART1->ICR = UART_CLEAR_TCF;
//		reset the UART's DMA channels transfer complete and half transfer flags
		DMA1->LIFCR = DMA_FLAG_TCIF1_5 | DMA_FLAG_HTIF1_5 | DMA_FLAG_TCIF0_4 | DMA_FLAG_HTIF0_4;
//		reenable the UART DMA channels
		SET_BIT(USART3_DMA_INSTANCE->CR, (DMA_SxCR_EN));
		SET_BIT(USART1_DMA_INSTANCE->CR, (DMA_SxCR_EN));
//		start the UARTs DMA transfer
		SET_BIT(USART3->CR3, USART_CR3_DMAT);
		SET_BIT(USART1->CR3, USART_CR3_DMAT);

//		wait for the first half of the receive buffer to be ready
#ifdef USE_BREADBOARD
		while ((DMA2->LISR & DMA_FLAG_HTIF0_4) != DMA_FLAG_HTIF0_4) {}
#else
		while ((DMA2->LISR & DMA_FLAG_HTIF1_5) != DMA_FLAG_HTIF1_5) {}
#endif
//		reset the SPI DMA channel half transfer flag
		DMA2->LIFCR = DMA_FLAG_HTIF0_4 | DMA_FLAG_HTIF1_5;
//		the starting index for the recieve buffer is 0
		j = 10;
		for (int i = 0; i < txCount; ++i) {
			yi[j] = (1.8862132469e-05) * aRxBuffer[j] + (7.5448529875e-05) * aRxBuffer[j-1] \
			+ (1.1317279481e-04) * aRxBuffer[j-2] + (7.5448529875e-05) * aRxBuffer[j-3] \
			+ (1.8862132469e-05) * aRxBuffer[j-4] \
			- (-3.6404314407e+00) * yi[j-1] - (4.9845340532e+00) * yi[j-2] \
			- (-3.0414501690e+00) * yi[j-3] - (6.9764935070e-01) * yi[j-4];
			++j;
			yi[j] = (1.8862132469e-05) * aRxBuffer[j] + (7.5448529875e-05) * aRxBuffer[j-1] \
			+ (1.1317279481e-04) * aRxBuffer[j-2] + (7.5448529875e-05) * aRxBuffer[j-3] \
			+ (1.8862132469e-05) * aRxBuffer[j-4] \
			- (-3.6404314407e+00) * yi[j-1] - (4.9845340532e+00) * yi[j-2] \
			- (-3.0414501690e+00) * yi[j-3] - (6.9764935070e-01) * yi[j-4];
			++j;
			yi[j] = (1.8862132469e-05) * aRxBuffer[j] + (7.5448529875e-05) * aRxBuffer[j-1] \
			+ (1.1317279481e-04) * aRxBuffer[j-2] + (7.5448529875e-05) * aRxBuffer[j-3] \
			+ (1.8862132469e-05) * aRxBuffer[j-4] \
			- (-3.6404314407e+00) * yi[j-1] - (4.9845340532e+00) * yi[j-2] \
			- (-3.0414501690e+00) * yi[j-3] - (6.9764935070e-01) * yi[j-4];
			++j;
			yi[j] = (1.8862132469e-05) * aRxBuffer[j] + (7.5448529875e-05) * aRxBuffer[j-1] \
			+ (1.1317279481e-04) * aRxBuffer[j-2] + (7.5448529875e-05) * aRxBuffer[j-3] \
			+ (1.8862132469e-05) * aRxBuffer[j-4] \
			- (-3.6404314407e+00) * yi[j-1] - (4.9845340532e+00) * yi[j-2] \
			- (-3.0414501690e+00) * yi[j-3] - (6.9764935070e-01) * yi[j-4];

			  aTxBuffer[i] = (uint16_t)yi[j];
				++j;
		}

//		wait for the UARTs to finish transferring
		while ((USART3->ISR & UART_FLAG_TC) != UART_FLAG_TC) {}
		while ((USART1->ISR & UART_FLAG_TC) != UART_FLAG_TC) {}
//		clear the transfer complete flag of the UARTs
		USART3->ICR = UART_CLEAR_TCF;
		USART1->ICR = UART_CLEAR_TCF;
//		clear the transfer complete and half transfer flags of the UART DMA channels
		DMA1->LIFCR = DMA_FLAG_TCIF1_5 | DMA_FLAG_HTIF1_5 | DMA_FLAG_TCIF0_4 | DMA_FLAG_HTIF0_4;
//		enable the UART DMA channels
		SET_BIT(USART3_DMA_INSTANCE->CR, (DMA_SxCR_EN));
		SET_BIT(USART1_DMA_INSTANCE->CR, (DMA_SxCR_EN));
//		start the UART DMA transfers
		SET_BIT(USART3->CR3, USART_CR3_DMAT);
		SET_BIT(USART1->CR3, USART_CR3_DMAT);
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}
	/** Macro to configure the PLL clock source
	 */
	__HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 115;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1
			| RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}

}

static void MX_SPI1_Init(void) {

	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
//	set mode to master
	hspi1.Init.Mode = SPI_MODE_MASTER;
//	recieve only
	hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
//	14 data bits, which includes the 2 leading 0s
	hspi1.Init.DataSize = SPI_DATASIZE_14BIT;
//	clock polarity is high
	hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
//	data is clocked on the first edge
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
//	slave select is managed by hardware
	hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
//	peripheral clock rate is half of pll clock
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
//	MSB transferred first
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
//	not TI mode
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
//	no CRC
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 0x0;
	hspi1.Init.TxCRCInitializationPattern =
			SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	hspi1.Init.RxCRCInitializationPattern =
			SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
//	the slave select pin will pulse inactive in between frames
//	the length of the pulse is 1 cycle
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
//	slave select active low
	hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
//	request data transfer function after 1 data
	hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
//	no idle time before first transfer
	hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
//	spend two idle cycles between transfers, the slave select is high for one of these cycles
	hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_02CYCLE;
//	no auto suspend on overflow
	hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
//	don't fix IO state
	hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
//	don't swap MISO and MOSI
	hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
//	initialize SPI with HAL library
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

	/* SPI1 parameter configuration*/
	hspi2.Instance = SPI2;
//	set mode to master
	hspi2.Init.Mode = SPI_MODE_MASTER;
//	recieve only
	hspi2.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
//	14 data bits, which includes the 2 leading 0s
	hspi2.Init.DataSize = SPI_DATASIZE_14BIT;
//	clock polarity is high
	hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
//	data is clocked on the first edge
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
//	slave select is managed by hardware
	hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
//	peripheral clock rate is half of pll clock
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
//	MSB transferred first
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
//	not TI mode
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
//	no CRC
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 0x0;
	hspi2.Init.TxCRCInitializationPattern =
			SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	hspi2.Init.RxCRCInitializationPattern =
			SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
//	the slave select pin will pulse inactive in between frames
//	the length of the pulse is 1 cycle
	hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
//	slave select active low
	hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
//	request data transfer function after 1 data
	hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
//	no idle time before first transfer
	hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
//	spend two idle cycles between transfers, the slave select is high for one of these cycles
	hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_02CYCLE;
//	no auto suspend on overflow
	hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
//	don't fix IO state
	hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
//	don't swap MISO and MOSI
	hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
//	initialize SPI with HAL library
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	huart1.Instance = USART1;
//	baud rate is peripheral clock divided by 8, check the ioc file
	huart1.Init.BaudRate = 11978688;
//	8 bits per word
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
//	1 stop bit
	huart1.Init.StopBits = UART_STOPBITS_1;
//	no parity check
	huart1.Init.Parity = UART_PARITY_NONE;
//	UART in both transmit and receive
	huart1.Init.Mode = UART_MODE_TX_RX;
//	no HW flow control
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//	oversample by 8
	huart1.Init.OverSampling = UART_OVERSAMPLING_8;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//	divide the peripheral clock by 1
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
//	no advanced features
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//	initialize peripheral with HAL library
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
//	initialize transfer fifo
	if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
//	initialize receive fifo
	if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
//	enable fifo
	if (HAL_UARTEx_EnableFifoMode(&huart1) != HAL_OK) {
		Error_Handler();
	}
	HAL_NVIC_SetPriority(USART1_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

	huart3.Instance = USART3;
	//	baud rate is peripheral clock divided by 8, check the ioc file
		huart3.Init.BaudRate = 11978688;
	//	8 bits per word
		huart3.Init.WordLength = UART_WORDLENGTH_8B;
	//	1 stop bit
		huart3.Init.StopBits = UART_STOPBITS_1;
	//	no parity check
		huart3.Init.Parity = UART_PARITY_NONE;
	//	UART in both transmit and receive
		huart3.Init.Mode = UART_MODE_TX_RX;
	//	no HW flow control
		huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	//	oversample by 8
		huart3.Init.OverSampling = UART_OVERSAMPLING_8;
		huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	//	divide the peripheral clock by 1
		huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	//	no advanced features
		huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	//	initialize peripheral with HAL library
		if (HAL_UART_Init(&huart3) != HAL_OK) {
			Error_Handler();
		}
	//	initialize transfer fifo
		if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8)
				!= HAL_OK) {
			Error_Handler();
		}
	//	initialize receive fifo
		if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8)
				!= HAL_OK) {
			Error_Handler();
		}
	//	enable fifo
		if (HAL_UARTEx_EnableFifoMode(&huart3) != HAL_OK) {
			Error_Handler();
		}
		HAL_NVIC_SetPriority(USART3_IRQn, 0, 1);
		HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	/* DMA1_Stream1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	/* DMA2_Stream1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);


}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD1_Pin LD3_Pin */
	GPIO_InitStruct.Pin = LD1_Pin | LD3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : GPIOE LEDs */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_13 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : GPIOF LEDs */
	GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pins : GPIOG LEDs */
	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	BSP_LED_Init(LED1);
	BSP_LED_Init(LED2);
	BSP_LED_Init(LED3);
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
}

// callback function for external GPIO interrupt
// toggles the value of UserButtonStatus
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == BUTTON_USER_PIN) {
		UserButtonStatus ^= 1;
	}
}

// callback for SPI DMA half transfer complete
void tx_h_complete(DMA_HandleTypeDef *hdma) {
	wTransferState = TRANSFER_H_COMPLETE;
}

/**
 * @brief  SPI error callbacks.
 * @param  hspi: SPI handle
 * @note   This example shows a simple way to report transfer error, and you can
 *         add your own implementation.
 * @retval None
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	wTransferState = TRANSFER_ERROR;
	Error_Handler();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle) {
	BSP_LED_On(LED3);
	Error_Handler();
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
//	disable interrupts
	__disable_irq();
//	toggle the red LED forever
	while (1) {
		BSP_LED_Toggle(LED3);
		HAL_Delay(100);
	}
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

