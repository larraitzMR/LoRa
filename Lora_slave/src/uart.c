/*
 * uart.c
 *
 *  Created on: 9 ago. 2017
 *      Author: Larraitz
 */

#include "hw.h"
#include "uart.h"

UART_HandleTypeDef huart1;

void UART_Config(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_GPIOA_CLK_ENABLE();

    /* Peripheral clock enable */
    __USART1_CLK_ENABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10    ------> USART1_RX
    */
    GPIO_InitStruct.Pin 	  	= GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode   		= GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull   		= GPIO_PULLUP;
    GPIO_InitStruct.Speed 	  	= GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate 	= GPIO_AF4_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}


/**
  * @brief  Inicializacion de la UART
  * @param  None
  * @retval None
  */
void UART1_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

  HAL_NVIC_SetPriority(USART1_IRQn, 0x1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

extern uint8_t bufferGPS[];

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
 {
	if (UartHandle->Instance == USART1) {
		HAL_NVIC_SetPendingIRQ(USART1_IRQn);
		if (HAL_UART_Receive_IT(&huart1, (uint8_t *) bufferGPS, 80) != HAL_OK) {
				Error_Handler();
			}
	}
}


/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if (UartHandle->Instance == USART1) {
		HAL_NVIC_SetPendingIRQ(USART1_IRQn);
		if (HAL_UART_Receive_IT(&huart1, (uint8_t *) bufferGPS, 80) != HAL_OK) {
				Error_Handler();
			}
	}
}
