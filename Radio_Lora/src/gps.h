/*
 * gps.h
 *
 *  Created on: 1 jun. 2017
 *      Author: Larraitz
 */

#include "hw.h"

#define USART1_CLK_ENABLE()              __USART1_CLK_ENABLE()
#define USART1_RX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()
#define USART1_TX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()

#define USART1_FORCE_RESET()             __USART1_FORCE_RESET()
#define USART1_RELEASE_RESET()           __USART1_RELEASE_RESET()


#define USART1_TX_PIN                  GPIO_PIN_9
#define USART1_TX_GPIO_PORT            GPIOA
#define USART1_TX_AF                   GPIO_AF4_USART1
#define USART1_RX_PIN                  GPIO_PIN_10
#define USART1_RX_GPIO_PORT            GPIOA
#define USART1_RX_AF                   GPIO_AF4_USART1


/* Exported functions ------------------------------------------------------- */

void gps_Config(void);
void gps_Init(void);

void SPI_Init();
void SPI_Config(void);
void Flush_Buffer(uint8_t* pBuffer, uint16_t BufferLength);
uint16_t Buffercmp(uint8_t *pBuffer1, uint8_t *pBuffer2, uint16_t BufferLength);
