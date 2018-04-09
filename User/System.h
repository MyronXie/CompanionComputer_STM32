/**
  ******************************************************************************
  * File Name       : System.h
  * Description     : System param for CC_STM32
  *
  * Version         : v0.3.1
  * Created Date    : 2018.02.02
  * Revised Date    : 2018.04.09
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
#include "stdlib.h"

// Function Select
#define ENABLE_BATTERYMGMT
//#define ENABLE_LANGINGGEAR
//#define ENABLE_CURRMONITOR
//#define SINGLE_BATTERY

#define BOARD_REV1
//#define BOARD_REV2

// <Dev> Option
#define WITHOUT_BATTERY
//#define INGORE_LOSTCOMM
//#define INGORE_VDIFF
//#define INGORE_POWEROFF
#define ENABLE_BATT_REINIT

//========== Message Code ==========
#define MSG_BLANK               0x00
#define MSG_BATT_INIT           0x01
#define MSG_BATT_ONBOARD        0x02
#define MSG_BATT_PWROFF_START   0x03
#define MSG_BATT_REINIT         0x04
#define MSG_BATT_PWROFF_END     0x05

//=========== Error Code ===========
#define ERR_SYS_GENERAL         0x10
#define ERR_SYS_SERIAL          0x11

#define ERR_LG_RESET            0x18

#define ERR_BATT_INIT           0x20
#define ERR_BATT_OFFBOARD       0x21
#define ERR_BATT_VDIFF          0x22
#define ERR_BATT_POWERON        0x23
#define ERR_BATT_ENABLEFET      0x24
#define ERR_BATT_POWEROFF       0x25
#define ERR_BATT_LOSTPWR        0x26
#define ERR_BATT_STILLPWR       0x27
#define ERR_BATT_UNDERVOLT      0x28
#define ERR_BATT_LOWSOC         0x29
#define ERR_BATT_OVERCURR       0x2A
#define ERR_BATT_OVERTEMP       0x2B
#define ERR_BATT_UNDERTEMP      0x2C
#define ERR_BATT_INNER          0x2D


//========== Command Code ==========
#define CMD_FLY_ARM             0x80
#define CMD_FLY_DISARM          0x81

// Log list
#define MSG_XX  ""
#define MSG_00  ""
#define MSG_01  "Battery Init success"
#define MSG_02  "Battery onboard"
#define MSG_03  "Start Power Off Process"
#define MSG_04  "Battery Reinit"
#define MSG_05  "Power Off Success"

#define MSG_10  "System Error"
#define MSG_11  "Serial Error"

#define MSG_18  "Landing Gear Auto Reset"

#define	MSG_20  "Init Fail"
#define	MSG_21  "Offboard"
#define	MSG_22  "Voltage mismatch"
#define	MSG_23  "Power On Fail"
#define	MSG_24  "FET Enable Fail"
#define	MSG_25  "Power Off Fail"
#define MSG_26  "Lost power in the air"
#define MSG_27  "Still power on"
#define MSG_28  "Undervoltage"
#define MSG_29  "Low battery"
#define MSG_2A  "Overcurrent"
#define MSG_2B  "Overtemperature"
#define MSG_2C  "Undertemperature"
#define MSG_2D  "Inner status error"

#define PARAM_BATT_0    ""
#define PARAM_BATT_1    "Battery A "
#define PARAM_BATT_2    "Battery B "
#define PARAM_BATT_3    "All Battery "

typedef struct
{
    uint8_t cmd;
    uint8_t param;
}MsgType;

typedef struct
{
    uint8_t cmd[10];
    uint8_t param[10];
    uint8_t front;
    uint8_t rear;
}QueueType;

extern SerialType USART1_Tx,USART3_Tx;
extern mavlink_message_t mavMsgTx;
extern uint16_t sendCnt;

extern uint8_t  sysConnect;
extern uint8_t  sysWarning;
extern uint8_t  sysStatus;
extern uint16_t sysTicks;
extern uint8_t  sysArmed;

extern uint8_t msgLostCnt;

extern char* msgList[48];
extern char* paramList[4];

void System_Init(void);

void System_Heartbeat(void);
void System_MsgReporter(void);
void System_ErrorHandler(void);

void Mavlink_SendMessage(mavlink_message_t* msg, uint16_t length);

void PRINTLOG(const char *format, ...);
void ReportMessage(MsgType msg);

void DELAY_MS(int32_t nms);

#endif /* __SYSTEM_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
