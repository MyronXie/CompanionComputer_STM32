/**
  ******************************************************************************
  * File Name		: bsp_usart.c
  * Description		: Drivers for usart (based on HAL)
  *
  * Version			: v0.1
  * Created	Date	: 2017.10.18
  * Revised	Date	: 2017.11.27
  *
  * Author			: Mingye Xie
  ******************************************************************************
  */

#include "bsp_usart.h"

UART_HandleTypeDef huart1,huart3;
uint8_t aRxBuffer;

uint16_t UART_Rx_Status;
uint8_t UART_Rx_Buff[256];

void USART_Init(void)
{
	//USART1 for Pixhawk
	huart1.Instance			=	USART1;
	huart1.Init.BaudRate	=	57600;
	huart1.Init.WordLength	=	UART_WORDLENGTH_8B;
	huart1.Init.StopBits	=	UART_STOPBITS_1;
	huart1.Init.Parity		=	UART_PARITY_NONE;
	huart1.Init.HwFlowCtl	=	UART_HWCONTROL_NONE;
	huart1.Init.Mode		=	UART_MODE_TX_RX;
	HAL_UART_Init(&huart1);
	
	HAL_NVIC_SetPriority(USART1_IRQn, 0, 2);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	
	//USART3 for debug
	huart3.Instance			=	USART3;
	huart3.Init.BaudRate	=	115200;
	huart3.Init.WordLength	=	UART_WORDLENGTH_8B;
	huart3.Init.StopBits	=	UART_STOPBITS_1;
	huart3.Init.Parity		=	UART_PARITY_NONE;
	huart3.Init.HwFlowCtl	=	UART_HWCONTROL_NONE;
	huart3.Init.Mode		=	UART_MODE_TX_RX;
	HAL_UART_Init(&huart3);
	
	HAL_NVIC_SetPriority(USART3_IRQn, 0, 3);
	HAL_NVIC_EnableIRQ(USART3_IRQn);
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	GPIO_InitTypeDef GPIO_InitS;
	
	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_USART3_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	GPIO_InitS.Pin		=	GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitS.Mode		=	GPIO_MODE_AF_PP;
	GPIO_InitS.Pull		=	GPIO_PULLUP;
	GPIO_InitS.Speed	=	GPIO_SPEED_FREQ_HIGH;
	GPIO_InitS.Alternate	=	GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOB,&GPIO_InitS);
	
	GPIO_InitS.Pin		=	GPIO_PIN_10|GPIO_PIN_11;
	GPIO_InitS.Alternate	=	GPIO_AF7_USART3;
	HAL_GPIO_Init(GPIOB,&GPIO_InitS);
	
	
}

int fputc(int ch, FILE *f)			//->printf()
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}

int fgetc(FILE * f)					//->scanf()
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart3,	(uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
/******************************END OF FILE******************************/
