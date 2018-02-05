/**
  ******************************************************************************
  * File Name       : bsp_tim.h
  * Description     : Drivers for Timer (based on HAL)
  *
  * Version         : v0.2
  * Created Date    : 2017.10.18
  * Revised Date    : 2018.01.24
  *
  * Author          : Mingye Xie
  ******************************************************************************
  */


#ifndef __BSP_TIM_H
#define __BSP_TIM_H

#include "stm32f3xx_hal.h"

void TIM_Init(void);
void TIM_Start(void);

#endif /* __BSP_TIM_H */

/******************************END OF FILE******************************/
