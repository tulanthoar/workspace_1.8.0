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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */


SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart3_tx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
//static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
//static void MX_SPI2_Init(void);
static void MX_DMA_Init(void);
static void tx_complete(DMA_HandleTypeDef *hdma);
static void tx_h_complete(DMA_HandleTypeDef *hdma);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */
enum {
  TRANSFER_WAIT,
  TRANSFER_COMPLETE,
  TRANSFER_H_COMPLETE,
  TRANSFER_ERROR
};
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
__IO ITStatus UartReady = RESET;
__IO uint32_t UserButtonStatus = 0;  /* set to 1 after User Button interrupt  */
ALIGN_32BYTES (__IO uint16_t aTxBuffer[1024]) = {0};
ALIGN_32BYTES (__IO uint16_t aRxBuffer[8192]) = {0};
float yi[8192] = {0};
__IO uint32_t wTransferState = TRANSFER_WAIT;
#define OVERSAMPLING 4
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	unsigned short rxCount = COUNTOF(aRxBuffer);
	unsigned short rxOffset = rxCount / 2;
	unsigned short txCount = COUNTOF(aTxBuffer);
  /* USER CODE BEGIN 1 */
  for( int i = 0; i < txCount; ++i ){
	  aTxBuffer[i] = i % 16384;
  }
  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();
//  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  HAL_EnableCompensationCell();
  MX_DMA_Init();
  //  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
//  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  /* Configure User push-button in Interrupt mode */

  while(UserButtonStatus == 0)
  {
    BSP_LED_Toggle(LED1);
    HAL_Delay(100);
  }
  UserButtonStatus = 0;
  uint32_t tickstart = HAL_GetTick();
  for(int i = 0; i < COUNTOF(aTxBuffer); ++i){
      if (UART_WaitOnFlagUntilTimeout(&huart3, UART_FLAG_TXE, RESET, tickstart, 50000) != HAL_OK)
      {
        return HAL_TIMEOUT;
      }
      huart3.Instance->TDR = aTxBuffer[i] & 0xFFU;
      if (UART_WaitOnFlagUntilTimeout(&huart3, UART_FLAG_TXE, RESET, tickstart, 50000) != HAL_OK)
      {
        return HAL_TIMEOUT;
      }
      huart3.Instance->TDR = (aTxBuffer[i] & 0xFF00U)>>8;
  }
  if (UART_WaitOnFlagUntilTimeout(&huart3, UART_FLAG_TC, RESET, tickstart, 50000) != HAL_OK)
  {
    return HAL_TIMEOUT;
  }
  BSP_LED_Off(LED1);
  while(UserButtonStatus == 0)
  {
    BSP_LED_Toggle(LED2);
    HAL_Delay(100);
  }
  UserButtonStatus = 0;
  UartReady = RESET;
  BSP_LED_Off(LED1);
  BSP_LED_Off(LED2);
    /* Process Locked */
    __HAL_LOCK(&hspi1);

    /* Configure communication direction : 1Line */
    if (hspi1.Init.Direction == SPI_DIRECTION_1LINE)
    {
      SPI_1LINE_RX(&hspi1);
    }

    /* Clear RXDMAEN bit */
    CLEAR_BIT(hspi1.Instance->CFG1, SPI_CFG1_RXDMAEN);

    /* Set the SPI Rx DMA transfer complete callback */
    hspi1.hdmarx->XferCpltCallback = tx_complete;
    hspi1.hdmarx->XferHalfCpltCallback = tx_h_complete;

    MODIFY_REG(((DMA_Stream_TypeDef   *)hdma_spi1_rx.Instance)->CR, (DMA_IT_TC | DMA_IT_HT), (DMA_IT_TC | DMA_IT_HT));
    /* Enable the Rx DMA Stream/Channel  */
    if (HAL_OK != HAL_DMA_Start(hspi1.hdmarx, (uint32_t)&hspi1.Instance->RXDR, (uint32_t)aRxBuffer, rxCount))
    {
      /* Update SPI error code */
      SET_BIT(hspi1.ErrorCode, HAL_SPI_ERROR_DMA);
      hspi1.State = HAL_SPI_STATE_READY;
      Error_Handler();
    }

      MODIFY_REG(hspi1.Instance->CR2, SPI_CR2_TSIZE, 0UL);

    /* Enable Rx DMA Request */
    SET_BIT(hspi1.Instance->CFG1, SPI_CFG1_RXDMAEN);

    /* Enable the SPI Error Interrupt Bit */
    __HAL_SPI_ENABLE_IT(&hspi1, (SPI_IT_OVR | SPI_IT_FRE | SPI_IT_MODF));

    /* Enable SPI peripheral */
    __HAL_SPI_ENABLE(&hspi1);

      SET_BIT(hspi1.Instance->CR1, SPI_CR1_CSTART);

  while (wTransferState != TRANSFER_H_COMPLETE)
  {
	    BSP_LED_Toggle(LED3);
  }
  wTransferState = TRANSFER_WAIT;
  int j = 0;
  aTxBuffer[0] = aRxBuffer[0];
  for( int i = 1; i < txCount; ++i, j+= OVERSAMPLING){
	  j+= OVERSAMPLING;
	  aTxBuffer[i] = aRxBuffer[j];
  }
  if(HAL_UART_Transmit_DMA(&huart3, (uint8_t*)aTxBuffer, sizeof(aTxBuffer))!= HAL_OK)
  {
    Error_Handler();
  }
  HAL_NVIC_DisableIRQ(USART3_IRQn);
  HAL_NVIC_DisableIRQ(USART3_DMA_IRQN);
  HAL_NVIC_DisableIRQ(SPI1_DMA_IRQN);
  BSP_LED_Off(LED3);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  while ((DMA2->LISR & DMA_FLAG_TCIF0_4) != DMA_FLAG_TCIF0_4 ) {}
	  DMA2->LIFCR = DMA_FLAG_TCIF0_4;
	  j = rxOffset;
	  aTxBuffer[0] = aRxBuffer[j];
	  for( int i = 1; i < txCount; ++i ){
		  for( int k = 0; k < OVERSAMPLING; ++k, ++j){
			  yi[j] = aRxBuffer[j] * 1.0;
		  }
//		  yi[j] = aRxBuffer[j] * 1.0;
//		  aTxBuffer[i] = (uint16_t)yi[j];
		  aTxBuffer[i] = aRxBuffer[j];
	  }

	  while ((USART3->ISR & UART_FLAG_TC) != UART_FLAG_TC)  {  }
	  USART3->ICR = UART_CLEAR_TCF;
	  DMA1->LIFCR = DMA_FLAG_TCIF1_5 | DMA_FLAG_HTIF1_5;
	  SET_BIT(USART3_DMA_INSTANCE->CR, (DMA_SxCR_EN));
	  SET_BIT(USART3->CR3, USART_CR3_DMAT);

	  while ((DMA2->LISR & DMA_FLAG_HTIF0_4) != DMA_FLAG_HTIF0_4 ) {}
	  DMA2->LIFCR = DMA_FLAG_HTIF0_4;
	  j = 0;
	  aTxBuffer[0] = aRxBuffer[0];
	  for( int i = 1; i < txCount; ++i){
		  for( int k = 0; k < OVERSAMPLING; ++k, ++j){
			  yi[j] = aRxBuffer[j] * 1.0;
		  }
		  aTxBuffer[i] = aRxBuffer[j];
	  }

	  while ((USART3->ISR & UART_FLAG_TC) != UART_FLAG_TC)  {  }
	  USART3->ICR = UART_CLEAR_TCF;
	  DMA1->LIFCR = DMA_FLAG_TCIF1_5 | DMA_FLAG_HTIF1_5;
	  SET_BIT(USART3_DMA_INSTANCE->CR, (DMA_SxCR_EN));
	  SET_BIT(USART3->CR3, USART_CR3_DMAT);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }


}

static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_14BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_02CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 12000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{
  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 11978688;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_8;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_MSBFIRST_INIT;
  huart3.AdvancedInit.MSBFirst = UART_ADVFEATURE_MSBFIRST_DISABLE;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

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
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
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

  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  BSP_LED_Init(LED3);
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == BUTTON_USER_PIN)
  {
    UserButtonStatus ^= 1;
  }
}

//void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
//{
//  /* Turn LED1 on: Transfer in transmission process is complete */
//  BSP_LED_On(LED1);
//  wTransferState = TRANSFER_COMPLETE;
//}

void tx_complete(DMA_HandleTypeDef *hdma)
{
	  /* Turn LED1 on: Transfer in transmission process is complete */
//	  BSP_LED_On(LED1);
	  wTransferState = TRANSFER_COMPLETE;
}

void tx_h_complete(DMA_HandleTypeDef *hdma)
{
	  /* Turn LED1 on: Transfer in transmission process is complete */
//	  BSP_LED_On(LED1);
	  wTransferState = TRANSFER_H_COMPLETE;
}

/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  wTransferState = TRANSFER_ERROR;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  UartReady = SET;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
	BSP_LED_On(LED3);
  Error_Handler();
}
/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	    BSP_LED_Toggle(LED3);
	    HAL_Delay(100);
  }
  /* USER CODE END Error_Handler_Debug */
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

