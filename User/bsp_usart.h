/**
  ******************************************************************************
  * File Name       : bsp_usart.h
  * Description     : Drivers for usart (based on HAL)
  *
  * Version         : v0.2
  * Created Date    : 2017.10.18
  * Revised Date    : 2018.01.31
  *
  * Author          : Mingye Xie
  ******************************************************************************
  */


#ifndef __BSP_USART_H
#define __BSP_USART_H


#include "stm32f3xx_hal.h"
#include "string.h"

#define BUFFSIZE    300

void USART_Init(void);
void USART_DeInit(void);
void USART_ReInit(void);

uint8_t Serial_Rx_Available(void);
uint8_t Serial_Rx_NextByte(void);

void Serial_Tx_Send(void);
void Serial_Tx_Package(uint8_t* buf, uint16_t length);

#endif /* __BSP_USART_H */

/******************************END OF FILE******************************/
