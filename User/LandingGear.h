/**
  ******************************************************************************
  * File Name       : LandingGear.h
  * Description     : Landing Gear Drivers
  *
  * Version         : v0.4
  * Created Date    : 2017.09.25
  * Revised Date    : 2018.04.19
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
#define PUL_LEFT_MAX        975
#define PUL_LEFT_MIN        475
#define PUL_RIGHT_MAX       1075
#define PUL_RIGHT_MIN       575

#define PUL_SCALE_UP        0.002
#define PUL_SCALE_DOWN      0.002

#define LG_CMD_DELAY        3000         //ms
#define LG_RELAY_DELAY      500         //ms

//STEER_LEFT_NORMAL
#define PUL_LEFT_UP         PUL_LEFT_MAX
#define PUL_LEFT_DOWN       PUL_LEFT_MIN

//STEER_RIGHT_REVERSE
#define PUL_RIGHT_UP        PUL_RIGHT_MIN
#define PUL_RIGHT_DOWN      PUL_RIGHT_MAX

#define PUL_LEFT_RANGE      (int16_t)(PUL_LEFT_UP-PUL_LEFT_DOWN)
#define PUL_RIGHT_RANGE     (int16_t)(PUL_RIGHT_UP-PUL_RIGHT_DOWN)

//#define STEER_TIANYI_MAX    900
//#define STEER_TIANYI_MIN    250

#define LG_POS_UP           0x01
#define LG_POS_DOWN         0x00

#define LG_STEER_LEFT       0x00
#define LG_STEER_RIGHT      0x01

#define LG_MODE_STANDBY     0x00
#define LG_MODE_CHANGING    0x01
#define LG_MODE_RESETING    0x02

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
void LG_Control(void);
int16_t LG_PulseStep(uint8_t pos,uint8_t type);
uint8_t LG_AdjustPulse(void);

void LandingGear_Init(void);
void LandingGear_ChangePosition(uint8_t pos);
void LandingGear_Control(void);

#endif /* __LANDING_GEAR_H */

/******************************END OF FILE******************************/
