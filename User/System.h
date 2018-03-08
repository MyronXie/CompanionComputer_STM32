/**
  ******************************************************************************
  * File Name       : System.h
  * Description     : System param for CC_STM32
  *
  * Version         : v0.3
  * Created Date    : 2018.02.02
  * Revised Date    : 2018.03.08
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

// Function Select
#define ENABLE_BATTERYMGMT
//#define ENABLE_LANGINGGEAR
//#define ENABLE_CURRMONITOR
//#define SINGLE_BATTERY

#define BOARD_REV1
//#define BOARD_REV2

// <Dev> Option
//#define INGORE_LOSTCOMM
//#define INGORE_VDIFF
//#define INGORE_POWEROFF

//========== Message Code ==========
#define MSG_BLANK               0x00
#define MSG_BATT_INIT           0x01
#define MSG_BATT_ONBOARD        0x02
#define MSG_BATT_PWROFF_START   0x03
#define MSG_BATT_REINIT         0x04
#define MSG_BATT_PWROFF_END     0x05
#define MSG_BATT_WAITING        0x0F

#define MSG_SYSTEM              0x10
#define MSG_BATTERY             0x20
#define MSG_LANDINGGEAR         0x30

//=========== Error Code ===========
#define ERR_SYS_GENERAL         0x11
#define ERR_SYS_SERIAL          0x12
#define ERR_BATT_OFFBOARD       0x21
#define ERR_BATT_VDIFF          0x22
#define ERR_BATT_POWERON        0x23
#define ERR_BATT_ENABLEFET      0x24
#define ERR_BATT_INIT           0x25
#define ERR_BATT_POWEROFF       0x26
#define ERR_BATT_LOSTPWR        0x27
#define ERR_LG_RESET            0x31

//========== Command Code ==========
#define CMD_FLY_ARM             0x80
#define CMD_FLY_DISARM          0x81

// Log list
#define MSG_00  ""
#define MSG_01  "Battery Init success"
#define MSG_02  "Battery onboard"
#define MSG_03  "Start Power Off Process"
#define MSG_04  "Battery Reinit"
#define MSG_05  "Power Off Success"
#define MSG_10  ""
#define MSG_11  "System Error"
#define MSG_12  "Serial Error"
#define	MSG_20  ""
#define	MSG_21  "Offboard"
#define	MSG_22  "Voltage mismatch"
#define	MSG_23  "Power On Fail"
#define	MSG_24  "FET Enable Fail"
#define	MSG_25  "Init Fail"
#define	MSG_26  "Power Off Fail"
#define MSG_27  "Lost power in the air"
#define MSG_30  ""
#define MSG_31  "Landing Gear Auto Reset"

#define ERR_BATTA               (1<<0)
#define ERR_BATTB               (1<<1)

typedef struct
{
    uint8_t id;
    char content[100];
}MsgType;

extern SerialType USART1_Tx,USART3_Tx;
extern mavlink_message_t mavMsgTx;
extern uint16_t sendCnt;

extern uint8_t sysConnect;
extern uint8_t sysWarning;
extern uint8_t sysStatusTemp;
extern uint8_t sysStatus;
extern uint16_t sysTicks;
extern uint8_t sysBattery;
extern uint8_t sysArmed;

extern uint8_t msgLostCnt;

extern char* msgList[64];

void System_Heartbeat(void);
void System_StatusReporter(void);
void System_ErrorHandler(void);
void Mavlink_SendMsg(MsgType msg);
void Mavlink_SendLog(uint8_t id, char* content);
void Mavlink_SendMessage(mavlink_message_t* msg, uint16_t length);

void PRINTLOG(const char *format, ...);
void DELAY_MS(int32_t nms);

#endif /* __SYSTEM_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
