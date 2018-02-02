/**
  ******************************************************************************
  * File Name		: System.h
  * Description		: System param for CC_STM32
  *
  * Version			: v0.2
  * Created	Date	: 2018.02.02
  * Revised	Date	: 2018.02.02
  *
  * Author			: Mingye Xie
  ******************************************************************************
  */

#ifndef __SYSTEM_H
#define __SYSTEM_H

#include "stm32f3xx_hal.h"
#include "bsp_usart.h"
#include "mavlink.h"
#include "mavlink_helpers.h"

// <Dev> Option

//#define INGORE_LOSTCOMM			1

#define ENABLE_LANGINGGEAR		1

//#define SINGLE_BATTERY		1
#define AUTO_POWEROFF		1
#define INGORE_VDIFF		1


// ErrorCode

#define ERR_BATT_OFFBOARD		0x11
#define ERR_BATT_VDIFF			0x12
#define ERR_BATT_POWERON		0x13
#define ERR_BATT_ENABLEFET		0x14
#define ERR_BATT_INIT			0x15
#define ERR_BATT_POWEROFF		0x16


void Mavlink_SendMessage(mavlink_message_t* msg, uint16_t length);

#endif /* __SYSTEM_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
