/*
 * gps.c
 *
 *  Created on: 1 jun. 2017
 *      Author: Larraitz
 */

#include "gps.h"
#include "hw.h"

UART_HandleTypeDef huart1;

void gps_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	USART1_CLK_ENABLE();

	/* Enable GPIO TX/RX clock */
	__GPIOA_CLK_ENABLE();

	/* UART TX GPIO pin configuration  */
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_USART1;

	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* UART RX GPIO pin configuration  */
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Alternate = GPIO_AF4_USART1;

	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void gps_Init(void)
{
	huart1.Instance = USART1;

	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;

	if (HAL_UART_Init(&huart1) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}

	HAL_NVIC_SetPriority(USART1_IRQn, 0x1, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
}
extern char RxBuf[];
extern __IO ITStatus UartReady;
char *linea = "$pSR,f,a,5,6,a\n\r";

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle) {
	if (UartHandle->Instance == USART1) {
		/* Set transmission flag: trasfer complete*/
		UartReady = SET;
		(&huart1)->gState = HAL_UART_STATE_READY;
		HAL_UART_Transmit_IT(&huart1, (uint8_t *) linea, sizeof(linea));
	}
}

/**
 * @brief  Rx Transfer completed callback
 * @param  UartHandle: UART handle
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
	if (UartHandle->Instance == USART1) {
		/* Set transmission flag: trasfer complete*/
		UartReady = SET;
		(&huart1)->RxState = HAL_UART_STATE_READY;
		HAL_UART_Receive_IT(&huart1, (uint8_t *) linea, sizeof(linea));
	}
}

SPI_HandleTypeDef hspi2;

void SPI_Config()
{
    GPIO_InitTypeDef GPIO_InitStruct;

    __SPI2_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();

	/**SPI2 GPIO Configuration
	 PB15     ------> SPI2_MOSI
	 PB14     ------> SPI2_MISO
	 PB13     ------> SPI2_SCK
	 */
	GPIO_InitStruct.Pin = GPIO_PIN_15 | GPIO_PIN_14 |GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF0_SPI2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Peripheral interrupt init*/
	HAL_NVIC_SetPriority(SPI2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(SPI2_IRQn);
}


void SPI_Init(void)
{
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;

  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi2.Init.CRCPolynomial = 10;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLED;

  HAL_SPI_Init(&hspi2);
}


uint8_t aTxBuf[] = "****SPI - Two Boards communication based on Interrupt **** SPI Message ******** SPI Message ******** SPI Message ****";
//extern __IO ITStatus SPIReady;
extern uint8_t aRxBuffer[];
/**
  * @brief  Tx Transfer completed callback
  * @param  hspi: SPI handle.
  * @retval None
  */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
//	SPIReady = SET;
//	HAL_SPI_Transmit_IT(&hspi2, (uint8_t*) aTxBuf, BUFFERSIZE);
	PRINTF("%s\n\r", RxBuf);
	if (HAL_SPI_Receive_IT(&hspi2, (uint8_t *) RxBuf, strlen(RxBuf))!= HAL_OK) {
		/* Transfer error in transmission process */
		Error_Handler();
	}
	while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY) {
	}
}

/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval 0  : pBuffer1 identical to pBuffer2
  *         >0 : pBuffer1 differs from pBuffer2
  */
uint16_t Buffercmp(uint8_t *pBuffer1, uint8_t *pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if ((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}
