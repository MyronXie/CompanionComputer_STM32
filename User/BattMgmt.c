/**
  ******************************************************************************
  * File Name		: BattMgmt.c
  * Description		: Battery Management Drivers (through I2C)
  *
  * Version			: v0.2
  * Created	Date	: 2017.09.25
  * Revised	Date	: 2018.01.23
  *
  * Author			: Mingye Xie
  ******************************************************************************
  */

#include "BattMgmt.h"

extern I2C_HandleTypeDef I2cHandle;


uint8_t Batt_Measure(BattMsg* _batt)
{
	uint8_t status;
	uint16_t regVal;
	
	status = Batt_ReadWord(_batt->id, BATT_FETStatus, &regVal);
	if(!status)	_batt->fetStatus = regVal;
	
	status = Batt_ReadWord(_batt->id, BATT_Voltage, &regVal);
	if(!status)	_batt->voltage = regVal;
			
	status = Batt_ReadWord(_batt->id, BATT_Temperature, &regVal);
	if(!status)	_batt->temperature = (regVal-2731)*10;	// Kelvin -> Celsius
			
	status = Batt_ReadWord(_batt->id, BATT_Current, &regVal);
	if(!status)	_batt->current = regVal;

	status = Batt_ReadWord(_batt->id, BATT_FullChargeCapacity, &regVal);
	if(!status)	_batt->fullChargeCapacity = regVal*10;
	
	status = Batt_ReadWord(_batt->id, BATT_RemainingCapacity, &regVal);
	if(!status)	_batt->remainingCapacity = regVal*10;

	status = Batt_ReadWord(_batt->id, BATT_RelativeSOC, &regVal);
	if(!status)	_batt->soc = regVal;
	
	return status;
}


uint8_t Batt_ReadWord(uint8_t _addr, uint8_t _reg, uint16_t* _data)
{
	return I2C_ReadWord(_addr, _reg, _data);
}

uint8_t Batt_WriteWord(uint8_t _addr, uint8_t _reg, uint16_t _data)
{
	return I2C_WriteWord(_addr, _reg, _data);
}

uint8_t Batt_WriteByte(uint8_t _addr, uint8_t _reg, uint8_t _data)
{
	return I2C_WriteByte(_addr, _reg, _data);
}

uint8_t Batt_ReadByte(uint8_t _addr, uint8_t _reg, uint8_t* _data)
{
	return I2C_ReadByte(_addr, _reg, _data);
}

/******************************END OF FILE******************************/
