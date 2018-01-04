/**
  ******************************************************************************
  * File Name		: bsp_usart.h
  * Description		: Drivers for usart (based on HAL)
  *
  * Version			: v0.1.1
  * Created	Date	: 2017.10.18
  * Revised	Date	: 2018.01.04
  *
  * Author			: Mingye Xie
  ******************************************************************************
  */


#ifndef __BSP_USART_H
#define __BSP_USART_H


#include "stm32f3xx_hal.h"


void USART_Init(void);
void USART_DeInit(void);

#endif /* __BSP_USART_H */

/******************************END OF FILE******************************/
