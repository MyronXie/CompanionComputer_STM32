/**
  ******************************************************************************
  * File Name       : System.h
  * Description     : System param for CC_STM32
  *
  * Version         : v0.2
  * Created Date    : 2018.02.02
  * Revised Date    : 2018.03.01
  *
  * Author          : Mingye Xie
  ******************************************************************************
  */

#ifndef __SYSTEM_H
#define __SYSTEM_H

#include "stm32f3xx_hal.h"
#include "bsp_usart.h"
#include "mavlink.h"
#include "mavlink_helpers.h"
#include "string.h"
#include "stdarg.h"

// <Dev> Option
//#define INGORE_LOSTCOMM
#define SINGLE_BATTERY
//#define INGORE_VDIFF

#define ENABLE_LANGINGGEAR
#define AUTO_POWEROFF

// MsgCode
#define MSG_SYSTEM              0x00
#define MSG_BATTERY             0x10
#define MSG_LANDINGGEAR         0x20

// Error Code
#define NO_ERR                  0x00
#define ERR_SYS_GENERAL         0x01
#define ERR_SYS_SERIAL          0x02
#define ERR_BATT_OFFBOARD       0x11
#define ERR_BATT_VDIFF          0x12
#define ERR_BATT_POWERON        0x13
#define ERR_BATT_ENABLEFET      0x14
#define ERR_BATT_INIT           0x15
#define ERR_BATT_POWEROFF       0x16
#define ERR_BATT_LOSTAIR        0x17
#define ERR_LG_RESET            0x21

// Command Code
#define CMD_FLY_ARM             0x80
#define CMD_FLY_DISARM          0x81

// Log list
#define LOG_00  ""
#define LOG_01  "System Error"
#define LOG_02  "Serial Error"
#define	LOG_10  ""
#define	LOG_11  "Offboard"
#define	LOG_12  "Voltage mismatch"
#define	LOG_13  "Power On Fail"
#define	LOG_14  "FET Enable Fail"
#define	LOG_15  "Init Fail"
#define	LOG_16  "Power Off Fail"
#define LOG_17  "Lost power in the air"
#define LOG_20  ""
#define LOG_21  "Landing Gear Auto Reset"

#define ERR_BATTA               (1<<0)
#define ERR_BATTB               (1<<1)


extern SerialType USART1_Tx,USART3_Tx;
extern mavlink_message_t mavMsgTx;
extern uint16_t sendCnt;

extern uint8_t sysConnect;
extern uint8_t sysWarning;
extern uint8_t sysStatusTemp;
extern uint8_t sysStatus;
extern uint16_t sysTicks;
extern uint8_t sysBattery;
extern uint8_t sysFlying;

extern uint8_t msgLostCnt;

extern char* logList[64];

void System_Heartbeat(void);
void System_StatusReporter(void);
void System_ErrorHandler(void);
void Mavlink_SendLog(uint8_t id, char* msg);
void Mavlink_SendMessage(mavlink_message_t* msg, uint16_t length);

void PRINTLOG(const char *format, ...);
void DELAY_MS(int32_t nms);

#endif /* __SYSTEM_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
