/**
  ******************************************************************************
  * File Name       : bsp_adc.h
  * Description     : Drivers for ADC (based on HAL)
  *
  * Version         : v0.3
  * Created Date    : 2018.03.05
  * Revised Date    : 2018.03.05
  *
  * Author          : Mingye Xie
  ******************************************************************************
  */

#ifndef __BSP_ADC_H
#define __BSP_ADC_H


#include "stm32f3xx_hal.h"


void ADC_Init(void);
void DMA_Init(void);

#endif /* __BSP_ADC_H */

/******************************END OF FILE******************************/
