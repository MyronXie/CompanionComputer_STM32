/**
  ******************************************************************************
  * File Name		: LandingGear.h
  * Description		: Landing Gear Drivers
  *
  * Version			: v0.1.1
  * Created	Date	: 2017.09.25
  * Revised	Date	: 2018.01.03
  *
  * Author			: Mingye Xie
  ******************************************************************************
  */

#ifndef __LANDING_GEAR_H
#define __LANDING_GEAR_H


#include "stm32f3xx_hal.h"
#include "math.h"

/* Parameters of Landing Gear Control */
#define PUL_LEFT_UP		1200
#define PUL_LEFT_DOWN	250
#define PUL_LEFT_Range	(PUL_LEFT_UP-PUL_LEFT_DOWN)

#define PUL_RIGHT_UP	1250
#define PUL_RIGHT_DOWN	250
#define PUL_RIGHT_Range	(PUL_RIGHT_UP-PUL_RIGHT_DOWN)

#define PUL_SCALE_UP	0.008
#define PUL_SCALE_DOWN	0.008

#define Relay_ON() 		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET)
#define Relay_OFF()		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET)

uint8_t LandingGear_Control(uint8_t pos, uint8_t* prog);
void LandingGear_Init(void);
void LG_Relay_Init(void);
void LG_TIM_Init(void);
void LandingGear_Reset(void);

#endif /* __LANDING_GEAR_H */

/******************************END OF FILE******************************/
