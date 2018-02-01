/**
  ******************************************************************************
  * File Name		: main.h
  * Description		: CompanionComputer_STM32 main program
  *
  * Version			: v0.2
  * Created	Date	: 2017.11.23
  * Revised	Date	: 2018.02.01
  *
  * Author			: Mingye Xie
  ******************************************************************************
  */

#ifndef __MAIN_H
#define __MAIN_H

// <Dev> Option
//#define INGORE_LOSTCOMM			1
#define ENABLE_LANGINGGEAR		1

#include "stm32f3xx_hal.h"
#include "bsp_usart.h"
#include "bsp_tim.h"
#include "bsp_i2c.h"
#include "bsp_misc.h"
#include "LandingGear.h"
#include "BattMgmt.h"
#include "mavlink.h"
#include "mavlink_helpers.h"

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
