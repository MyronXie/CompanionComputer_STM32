/**
  ******************************************************************************
  * File Name       : CurrMonitor.c
  * Description     : ESC Current Monitor
  *
  * Version         : v0.3
  * Created Date    : 2017.03.05
  * Revised Date    : 2018.04.11
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

#define ESC_NUM           6
#define ESC_SAMPLE_TIMES  10
#define ESC_ARRAY_SIZE    ESC_NUM*ESC_SAMPLE_TIMES

void CurrMonitor_Init(void);
void CurrMonitor_Capture(void);
void CurrMonitor_Send(void);

#endif /* __BATTMGMT_H */

/******************************END OF FILE******************************/
