/**
  ******************************************************************************
  * File Name       : LandingGear.h
  * Description     : Landing Gear Drivers
  *
  * Version         : v0.3.1
  * Created Date    : 2017.09.25
  * Revised Date    : 2018.04.16
  *
  * Author          : Mingye Xie
  ******************************************************************************
  */

#ifndef __LANDING_GEAR_H
#define __LANDING_GEAR_H


#include "stm32f3xx_hal.h"
#include "bsp_usart.h"
#include "bsp_misc.h"
#include "System.h"
#include "math.h"

/* Parameters of Landing Gear Control */
#define PUL_LEFT_UP     775
#define PUL_LEFT_DOWN   250
#define PUL_LEFT_Range  (PUL_LEFT_UP-PUL_LEFT_DOWN)

#define PUL_RIGHT_UP    775
#define PUL_RIGHT_DOWN  250
#define PUL_RIGHT_Range (PUL_RIGHT_UP-PUL_RIGHT_DOWN)

#define PUL_SCALE_UP    0.01
#define PUL_SCALE_DOWN  0.01

#define LG_CHANGE_DELAY 200     //*10ms = 2s


#ifdef BOARD_REV1
    #define Relay_ON()      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET)
    #define Relay_OFF()     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET)
#elif
    #define Relay_ON()      HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET)
    #define Relay_OFF()     HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET)
#endif

extern uint8_t lgAutoReset;



void LG_Relay_Init(void);
void LG_TIM_Init(void);
void LG_Reset(void);
uint8_t LG_Control(uint8_t pos);

void LandingGear_Init(void);
void LandingGear_Control(uint8_t param);
void LandingGear_Adjustment(void);
void LandingGear_Reset(void);

#endif /* __LANDING_GEAR_H */

/******************************END OF FILE******************************/
