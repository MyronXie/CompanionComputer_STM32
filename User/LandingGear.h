/**
  ******************************************************************************
  * File Name       : LandingGear.h
  * Description     : Landing Gear Drivers
  *
  * Version         : v0.4
  * Created Date    : 2017.09.25
  * Revised Date    : 2018.04.18
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
#define PUL_LEFT_MAX        1000
#define PUL_LEFT_MIN        475
#define PUL_RIGHT_MAX       1100
#define PUL_RIGHT_MIN       575

#define STEER_LEFT_NORMAL
//#define STEER_LEFT_REVERSE
//#define STEER_RIGHT_NORMAL
#define STEER_RIGHT_REVERSE

#define PUL_SCALE_UP        0.012//0.012
#define PUL_SCALE_DOWN      0.012//0.012

#define LG_CHANGE_DELAY     10//200     //*10ms = 2s
#define LG_RELAY_DELAY      500     //ms

#define STEER_TIANYI_UP     900
#define STEER_TIANYI_DOWN   250

#ifdef STEER_LEFT_NORMAL
#define PUL_LEFT_UP         PUL_LEFT_MAX
#define PUL_LEFT_DOWN       PUL_LEFT_MIN
#else
#define PUL_LEFT_UP         PUL_LEFT_MIN
#define PUL_LEFT_DOWN       PUL_LEFT_MAX
#endif

#ifdef STEER_RIGHT_NORMAL
#define PUL_RIGHT_UP        PUL_RIGHT_MAX
#define PUL_RIGHT_DOWN      PUL_RIGHT_MIN
#else
#define PUL_RIGHT_UP        PUL_RIGHT_MIN
#define PUL_RIGHT_DOWN      PUL_RIGHT_MAX
#endif

#define PUL_LEFT_RANGE      (int16_t)(PUL_LEFT_UP-PUL_LEFT_DOWN)
#define PUL_RIGHT_RANGE     (int16_t)(PUL_RIGHT_UP-PUL_RIGHT_DOWN)

#define LG_POS_UP           1
#define LG_POS_DOWN         0

#define LG_STEER_LEFT       0
#define LG_STEER_RIGHT      1

#define LG_MODE_STANDBY     0
#define LG_MODE_CHANGING    1

#ifdef BOARD_REV1
    #define Relay_ON()      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET)
    #define Relay_OFF()     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET)
#else
#ifdef BOARD_REV2
    #define Relay_ON()      HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET)
    #define Relay_OFF()     HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET)
#endif
#endif

extern uint8_t lgAutoReset;


void LG_Relay_Init(void);
void LG_TIM_Init(void);
void LG_Reset(void);
uint8_t LG_Control(uint8_t pos);
int8_t LG_Step(uint8_t pos,uint8_t type);

void LandingGear_Init(void);
void LandingGear_Control(uint8_t param);
void LandingGear_Adjustment(void);

#endif /* __LANDING_GEAR_H */

/******************************END OF FILE******************************/
