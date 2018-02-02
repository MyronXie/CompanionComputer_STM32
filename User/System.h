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
//#define INGORE_LOSTCOMM
//#define SINGLE_BATTERY
//#define INGORE_VDIFF

#define ENABLE_LANGINGGEAR
#define AUTO_POWEROFF



// ErrorCode
#define ERR_SYS_GENERAL			0x01
#define ERR_SYS_SERIAL			0x02
#define ERR_BATT_OFFBOARD		0x11
#define ERR_BATT_VDIFF			0x12
#define ERR_BATT_POWERON		0x13
#define ERR_BATT_ENABLEFET		0x14
#define ERR_BATT_INIT			0x15
#define ERR_BATT_POWEROFF		0x16
#define ERR_LG_RESET			0x21

// Log list
#define LOG_00	""
#define LOG_01	"System Error"
#define LOG_02	"Serial Error"
#define	LOG_10	"Battery Init Success"
#define	LOG_11	"Battery Offboard"
#define	LOG_12	"Voltage difference"
#define	LOG_13	"PowerOn Fail"
#define	LOG_14	"FET Enable Fail"
#define	LOG_15	"Battery Init Fail"
#define	LOG_16	"PowerOff Fail"
#define LOG_20	""
#define LOG_21	"Landing Gear Auto Reset"

extern mavlink_message_t mavMsgTx;
extern uint16_t sendByteCnt;

extern uint8_t sysConnect;					// Flag for system working (Receive first heartbeat from FC)
extern uint8_t sysWarning;					// Counter for fatal error
extern uint8_t sysError;					// Flag for battery , 0 for no problem
extern uint8_t sysReport;					// Flag for report error msg
extern uint16_t sysTicks;					// Record system running time

extern uint8_t msgLostCnt;					// Mavlink Communication Lost Counter

extern char* logList[64];

void System_Heartbeat(void);
void System_ErrorReporter(void);
void System_ErrorHandler(void);
void Mavlink_SendMessage(mavlink_message_t* msg, uint16_t length);

#endif /* __SYSTEM_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
