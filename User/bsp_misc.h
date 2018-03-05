/**
  ******************************************************************************
  * File Name       : bsp_misc.h
  * Description     : Drivers for LED, Flash, IWDG (based on HAL)
  *
  * Version         : v0.3
  * Created Date    : 2018.02.01
  * Revised Date    : 2018.03.05
  *
  * Author          : Mingye Xie
  ******************************************************************************
  */


#ifndef __BSP_MISC_H
#define __BSP_MISC_H


#include "stm32f3xx_hal.h"

#define FLASHADDR 0x0800F800    // Page 31: 0x0800F800-0x0800FFFF (2K)
#define FLASHSIZE 10

#define LED1    GPIOB,GPIO_PIN_12
#define LED2    GPIOB,GPIO_PIN_12
#define LED3    GPIOB,GPIO_PIN_12
#define LED4    GPIOB,GPIO_PIN_12

#define LED_ON(led)     HAL_GPIO_WritePin(led,GPIO_PIN_SET)
#define LED_OFF(led)    HAL_GPIO_WritePin(led,GPIO_PIN_RESET)
#define LED_TOGGLE(led) HAL_GPIO_TogglePin(led)


void FLASH_SaveParam(uint32_t* param, uint8_t size);
uint32_t FLASH_LoadParam(uint8_t id);

void IWDG_Init(void);
void IWDG_Feed(void);

void LED_Init(void);

#endif /* __BSP_MISC_H */

/******************************END OF FILE******************************/
