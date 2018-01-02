/**
  ******************************************************************************
  * File Name		: BattMgmt.h
  * Description		: Battery Management Drivers (through I2C)
  *
  * Version			: v0.1
  * Created	Date	: 2017.09.25
  * Revised	Date	: 2017.11.27
  *
  * Author			: Mingye Xie
  ******************************************************************************
  */

#ifndef __BATTMGMT_H
#define __BATTMGMT_H


#include "stm32f3xx_hal.h"
#include "math.h"


//============I2C Address============
#define BATT_A 		0x16
#define BATT_B	 	0x26


#define BATT_POWERON			0x55AA
#define BATT_POWEROFF			0xAA55
#define BATT_ENABLEFET			0xAA54

//==============Register=============
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
#define BATT_DesignCapacity		0x18
#define BATT_SpecificationInfo	0x1A
#define BATT_SerialNumber		0x1C
#define BATT_PowerControl		0x71


#define	SerialNumber			0x0001
#define	SpecificationInfo		0x0031


void Batt_Init(void);
uint8_t Batt_WriteWord(uint8_t _addr, uint8_t _reg, uint16_t _data);
uint8_t Batt_ReadWord(uint8_t _addr, uint8_t _reg, uint16_t* _data);
uint8_t Batt_WriteByte(uint8_t _addr, uint8_t _reg, uint8_t _data);
uint8_t Batt_ReadByte(uint8_t _addr, uint8_t _reg, uint8_t* _data);

//uint8_t Battery_ReadReg(uint8_t _addr, uint8_t _reg, uint8_t* _data, uint8_t _num);
//uint8_t Battery_WriteReg(uint8_t _addr, uint8_t _reg, uint8_t* _data, uint8_t _num);

#endif /* __BATTMGMT_H */

/******************************END OF FILE******************************/
