/**
  ******************************************************************************
  * File Name		: BattMgmt.h
  * Description		: Battery Management Drivers (through I2C)
  *
  * Version			: v0.2
  * Created	Date	: 2017.09.25
  * Revised	Date	: 2018.02.02
  *
  * Author			: Mingye Xie
  ******************************************************************************
  */

#ifndef __BATTMGMT_H
#define __BATTMGMT_H

#include "stm32f3xx_hal.h"
#include "bsp_usart.h"
#include "bsp_i2c.h"
#include "System.h"
#include "math.h"

typedef struct
{
	uint8_t		id;
	uint8_t 	status;				// High 4-bit for BattA, Low 4-bit for BattB
	uint8_t		lostCnt;
	uint8_t		fet;				// Read from SMBUS
	uint16_t	temperature;
	uint16_t	voltage;
	int16_t		current;
	uint8_t 	soc;
	uint16_t	remainingCapacity;
	uint16_t	fullChargeCapacity;
	uint16_t	designCapacity;
}BattMsg;


//===============Register==============
#define BATT_BatteryMode		0x03
#define BATT_Temperature		0x08
#define BATT_Voltage			0x09
#define BATT_Current			0x0A
#define BATT_MaxError			0x0C
#define BATT_RelativeSOC		0x0D
#define BATT_AbsoluteSOC		0x0E
#define BATT_RemainingCapacity	0x0F
#define BATT_FullChargeCapacity	0x10
#define BATT_BatteryStatus		0x16
#define BATT_CycleCount			0x17
#define BATT_DesignCapacity		0x18
#define BATT_SpecificationInfo	0x1A
#define BATT_SerialNumber		0x1C

#define BATT_PowerControl		0x71
#define BATT_FETStatus			0x72

#define	SerialNumber			0x0001
#define	SpecificationInfo		0x0031

//==========BATT_PowerControl==========
#define BATT_POWERON			0xAA55
#define BATT_POWEROFF			0x55AA
#define BATT_ENABLEFET			0x54AA

//============BATT_FETStatus============
#define PWR_ON					(1<<5)
#define FET_LOCK				(1<<4)
#define PRE_EN					(1<<3)
#define DFET_EN					(1<<2)
#define CFET_EN					(1<<1)
#define FET_EN					(1<<0)

//============BATT_BattStatus============
#define BATT_INUSE				(1<<1)
#define BATT_ONBOARD			(1<<0)

//============battCycleCnt============
#define BATT_SYS_JUDGE			(1<<5)
#define BATT_SYS_BATTB			(1<<4)
#define BATT_SYS_SEND			(1<<3)
#define BATT_SYS_MASK_CMD		0x07


uint8_t Batt_Init(void);

uint8_t Batt_WriteWord(uint8_t _addr, uint8_t _reg, uint16_t _data);
uint8_t Batt_ReadWord(uint8_t _addr, uint8_t _reg, uint16_t* _data);
uint8_t Batt_WriteByte(uint8_t _addr, uint8_t _reg, uint8_t _data);
uint8_t Batt_ReadByte(uint8_t _addr, uint8_t _reg, uint8_t* _data);

void Batt_Measure(BattMsg* _batt, uint8_t _cmd);

uint8_t Battery_Management(void);
void Battery_MavlinkPack(mavlink_battery_status_t* mav);

#endif /* __BATTMGMT_H */

/******************************END OF FILE******************************/
