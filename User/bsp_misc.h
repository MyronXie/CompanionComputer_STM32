/**
  ******************************************************************************
  * File Name		: bsp_misc.c
  * Description		: Drivers for Flash & IWDG (based on HAL)
  *
  * Version			: v0.2
  * Created	Date	: 2018.02.01
  * Revised	Date	: 2018.02.01
  *
  * Author			: Mingye Xie
  ******************************************************************************
  */


#ifndef __BSP_MISC_H
#define __BSP_MISC_H


#include "stm32f3xx_hal.h"

#define FLASHADDR 0x0800F800	// Page 31: 0x0800F800-0x0800FFFF (2K)
#define FLASHSIZE 10

void FLASH_SaveParam(uint32_t* param, uint8_t size);
void FLASH_LoadParam(uint32_t* param, uint8_t size);

void IWDG_Init(void);
void IWDG_Feed(void);

#endif /* __BSP_MISC_H */

/******************************END OF FILE******************************/
