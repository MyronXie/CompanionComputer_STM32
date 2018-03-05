/**
  ******************************************************************************
  * File Name       : CurrMonitor.c
  * Description     : ESC Current Monitor
  *
  * Version         : v0.3
  * Created Date    : 2017.03.05
  * Revised Date    : 2018.03.05
  *
  * Author          : Mingye Xie
  ******************************************************************************
  */

#ifndef __CURRMONITOR_H
#define __CURRMONITOR_H

#include "stm32f3xx_hal.h"
#include "bsp_usart.h"
#include "bsp_adc.h"
#include "System.h"
#include "math.h"

void CurrMonitor_Init(void);
void CurrMonitor_Send(void);

#endif /* __BATTMGMT_H */

/******************************END OF FILE******************************/
