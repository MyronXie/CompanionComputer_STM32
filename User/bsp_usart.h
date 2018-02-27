/**
  ******************************************************************************
  * File Name       : bsp_usart.h
  * Description     : Drivers for usart (based on HAL)
  *
  * Version         : v0.2
  * Created Date    : 2017.10.18
  * Revised Date    : 2018.02.27
  *
  * Author          : Mingye Xie
  ******************************************************************************
  */


#ifndef __BSP_USART_H
#define __BSP_USART_H


#include "stm32f3xx_hal.h"
#include "string.h"

#define BUFFSIZE    300

typedef struct
{
	UART_HandleTypeDef* handle;
    uint8_t buffer[BUFFSIZE];
    uint16_t length;
	uint8_t *front;
	uint8_t *rear;
	uint8_t flag;
}
SerialType;

void USART_Init(void);
void USART_DeInit(void);
void USART_ReInit(void);

uint8_t Serial_Rx_Available(void);
uint8_t Serial_Rx_NextByte(void);
void Serial_Send(SerialType* serial, uint8_t* buf, uint16_t len);

#endif /* __BSP_USART_H */

/******************************END OF FILE******************************/
