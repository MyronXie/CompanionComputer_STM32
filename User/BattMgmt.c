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

BattMsg battA,battB;

uint8_t Batt_Init(void)
{
	uint8_t status = 0;
	uint16_t regVal;
	
	//Battery Message Init
	battA.id					= 0x16;
	battB.id					= 0x26;
	
	Batt_ReadFET(&battA);
	Batt_ReadFET(&battB);
	
	if((battA.status&BATT_OFFOBARD)||(battB.status&BATT_OFFOBARD))	// At least one battery is offboard
	{	
		printf("\r\n!   Battery offboard!");
		if(battA.status&PWR_ON) Batt_WriteWord(battA.id, BATT_PowerControl, BATT_POWEROFF);
		if(battB.status&PWR_ON) Batt_WriteWord(battB.id, BATT_PowerControl, BATT_POWEROFF);	
		HAL_Delay(2000);
		if(battA.status&PWR_ON) Batt_WriteWord(battA.id, BATT_PowerControl, BATT_POWEROFF);
		if(battB.status&PWR_ON) Batt_WriteWord(battB.id, BATT_PowerControl, BATT_POWEROFF);	
		return 0x01;
	}
	else
	{
		if(!((battA.status&PWR_ON)||(battB.status&PWR_ON)))		// All batteries are power off
		{
			printf("\r\n!   Not powered by batteries!");
			return 0x02;
		}
		else													// At least one battery is offboard
		{
			//Read voltage
			status += Batt_ReadWord(battA.id, BATT_Voltage, &regVal);
			if(!status)	battA.voltage = regVal;
			status += Batt_ReadWord(battB.id, BATT_Voltage, &regVal);
			if(!status)	battB.voltage = regVal;
			
			if(!status)
			{
				// Voltage difference small than 200mV
				if(((battA.voltage>battB.voltage)?(battA.voltage-battB.voltage):(battB.voltage-battA.voltage))<200)
				{
					// Power on
					printf("\r\n#   Power On battery");
					if(!(battA.status&PWR_ON)) Batt_WriteWord(battA.id, BATT_PowerControl, BATT_POWERON);
					if(!(battB.status&PWR_ON)) Batt_WriteWord(battB.id, BATT_PowerControl, BATT_POWERON);
					HAL_Delay(2000);	
					if(!(battA.status&PWR_ON)) Batt_WriteWord(battA.id, BATT_PowerControl, BATT_POWERON);
					if(!(battB.status&PWR_ON)) Batt_WriteWord(battB.id, BATT_PowerControl, BATT_POWERON);
					
//					if(!((battA.status&PWR_ON)&&(battB.status&PWR_ON)))
//					{
//						printf("\r\n! Can't power on battery!");
//						if(battA.status&PWR_ON) Batt_WriteWord(battA.id, BATT_PowerControl, BATT_POWEROFF);
//						if(battB.status&PWR_ON) Batt_WriteWord(battB.id, BATT_PowerControl, BATT_POWEROFF);	
//						HAL_Delay(2000);
//						if(battA.status&PWR_ON) Batt_WriteWord(battA.id, BATT_PowerControl, BATT_POWEROFF);
//						if(battB.status&PWR_ON) Batt_WriteWord(battB.id, BATT_PowerControl, BATT_POWEROFF);	
//						return 0x04;
//					}
					
					// Enable FET
					printf("\r\n#   Enable FET");
					Batt_WriteWord(battA.id, BATT_PowerControl, BATT_ENABLEFET);
					Batt_WriteWord(battB.id, BATT_PowerControl, BATT_ENABLEFET);
					HAL_Delay(2000);
					if(!(battA.status&FET_LOCK)) Batt_WriteWord(battA.id, BATT_PowerControl, BATT_POWERON);
					if(!(battB.status&FET_LOCK)) Batt_WriteWord(battB.id, BATT_PowerControl, BATT_POWERON);
				}
				else
				{
					printf("\r\n#   Voltage difference > 200mV! ");
					if(battA.status&PWR_ON) Batt_WriteWord(battA.id, BATT_PowerControl, BATT_POWEROFF);
					if(battB.status&PWR_ON) Batt_WriteWord(battB.id, BATT_PowerControl, BATT_POWEROFF);	
					HAL_Delay(2000);
					if(battA.status&PWR_ON) Batt_WriteWord(battA.id, BATT_PowerControl, BATT_POWEROFF);
					if(battB.status&PWR_ON) Batt_WriteWord(battB.id, BATT_PowerControl, BATT_POWEROFF);	
					return 0x08;
				}
			}
		}
	}
	
	return 0x00;
}


uint8_t Batt_Measure(BattMsg* _batt)
{
	uint8_t status = 0;
	uint16_t regVal;
	
	status += Batt_ReadWord(_batt->id, BATT_FETStatus, &regVal);
	if(!status)	_batt->status = regVal;
	
	status += Batt_ReadWord(_batt->id, BATT_Voltage, &regVal);
	if(!status)	_batt->voltage = regVal;
			
	status += Batt_ReadWord(_batt->id, BATT_Temperature, &regVal);
	if(!status)	_batt->temperature = (regVal-2731)*10;	// Kelvin -> Celsius
			
	status += Batt_ReadWord(_batt->id, BATT_Current, &regVal);
	if(!status)	_batt->current = regVal;

	status += Batt_ReadWord(_batt->id, BATT_FullChargeCapacity, &regVal);
	if(!status)	_batt->fullChargeCapacity = regVal*10;
	
	status += Batt_ReadWord(_batt->id, BATT_RemainingCapacity, &regVal);
	if(!status)	_batt->remainingCapacity = regVal*10;

	status += Batt_ReadWord(_batt->id, BATT_RelativeSOC, &regVal);
	if(!status)	_batt->soc = regVal;
	
	return status;
}

void Batt_ReadFET(BattMsg* _batt)
{
	uint8_t status = 0;
	uint16_t regVal;
	
	status += Batt_ReadWord(_batt->id, BATT_FETStatus, &regVal);
	if(!status)	_batt->status = regVal;
	else 		_batt->status = BATT_OFFOBARD;
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
