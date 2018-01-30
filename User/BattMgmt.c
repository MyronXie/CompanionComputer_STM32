/**
  ******************************************************************************
  * File Name		: BattMgmt.c
  * Description		: Battery Management Drivers (through I2C)
  *
  * Version			: v0.2
  * Created	Date	: 2017.09.25
  * Revised	Date	: 2018.01.30
  *
  * Author			: Mingye Xie
  ******************************************************************************
  */

#include "BattMgmt.h"

BattMsg battA,battB;

uint8_t Batt_Init(void)
{
	uint8_t attemptTimes = 0;
	
	// Battery Message Init
	battA.id					= 0x16;
	battB.id					= 0x26;
	
	#ifdef SINGLE_BATTERY
	printf(": SINGLE_BATTERY");
	#endif
	
	// Read 1st FET status
	Batt_ReadFET(&battA);
	Batt_ReadFET(&battB);
		
	// Check onboard status
	attemptTimes = 0;
	printf("\r\n#   Link Process");
	#ifdef SINGLE_BATTERY
	while(!((battA.status&BATT_ONBOARD)||(battB.status&BATT_ONBOARD)))		// At least one battery is onboard
	#else  //DUAL_BATTERY
	while(!((battA.status&BATT_ONBOARD)&&(battB.status&BATT_ONBOARD)))		// Both batteries should onboard
	#endif
	{
		if(++attemptTimes>=5)
		{			
			if(!((battA.status&BATT_ONBOARD)||(battB.status&BATT_ONBOARD)))
			{
				printf("\r\n!   Both batteries Offboard!");
				return 0x01;
			}
			
			printf("\r\n!   ");
			if(!(battA.status&BATT_ONBOARD)) printf("battA Offboard!");
			else if(battA.fet&PWR_ON) Batt_WriteWord(battA.id, BATT_PowerControl, BATT_POWEROFF);
			if(!(battB.status&BATT_ONBOARD)) printf("battB Offboard!");
			else if(battB.fet&PWR_ON) Batt_WriteWord(battB.id, BATT_PowerControl, BATT_POWEROFF);

			return 0x01;
		}
		
		printf("\r\n#     Attempt#%d",attemptTimes);	
		Batt_ReadFET(&battA);
		Batt_ReadFET(&battB);		
		HAL_Delay(20);				
	}
	
	// Check power status
	#ifndef SINGLE_BATTERY
	if(!((battA.fet&PWR_ON)||(battB.fet&PWR_ON)))		// All batteries are power off
	{
		printf("\r\n!   Not powered by batteries!");
		//return 0x02;
	}
	
	// Check Voltage Difference
	#ifndef INGORE_VDIFF
	
	// Read voltage
	Batt_ReadWord(battA.id, BATT_Voltage, &battA.voltage);
	Batt_ReadWord(battB.id, BATT_Voltage, &battB.voltage);
	
	// Vdiff should small than 100mV
	if(((battA.voltage>battB.voltage)?(battA.voltage-battB.voltage):(battB.voltage-battA.voltage))>=100)
	{
		printf("\r\n#   Vdiff > 100mV!");
		if(battA.fet&PWR_ON) Batt_WriteWord(battA.id, BATT_PowerControl, BATT_POWEROFF);
		if(battB.fet&PWR_ON) Batt_WriteWord(battB.id, BATT_PowerControl, BATT_POWEROFF);
		HAL_Delay(2000);
		if(battA.fet&PWR_ON) Batt_WriteWord(battA.id, BATT_PowerControl, BATT_POWEROFF);
		if(battB.fet&PWR_ON) Batt_WriteWord(battB.id, BATT_PowerControl, BATT_POWEROFF);
		return 0x04;
	}
	#endif //INGORE_VDIFF
	#endif //SINGLE_BATTERY

	// Power ON Process
	attemptTimes = 0;
	printf("\r\n#   Power On Process");
	#ifdef SINGLE_BATTERY
	while(!((battA.fet&PWR_ON)||(battB.fet&PWR_ON)))		// Both battery is powered off
	#else  //DUAL_BATTERY
	while(!((battA.fet&PWR_ON)&&(battB.fet&PWR_ON)))		// At least one battery is powered off
	#endif
	{
		if(++attemptTimes>=5)
		{
			printf("!   Auto Power On Fail!");
			return 0x10;
		}
		
		printf("\r\n#     Attempt#%d",attemptTimes);
		if(!(battA.fet&PWR_ON)) Batt_WriteWord(battA.id, BATT_PowerControl, BATT_POWERON);
		if(!(battB.fet&PWR_ON)) Batt_WriteWord(battB.id, BATT_PowerControl, BATT_POWERON);
		
		HAL_Delay(2000);	
		
		Batt_ReadFET(&battA);
		Batt_ReadFET(&battB);
		printf(": A-0x%02X, B-0x%02X",battA.fet,battB.fet);
					
	}
	
	// Enable FET process
	attemptTimes = 0;
	printf("\r\n#   Enable FET Process");
	#ifdef SINGLE_BATTERY
	while(!((battA.fet&FET_LOCK)||(battB.fet&FET_LOCK)))
	#else  //DUAL_BATTERY
	while(!((battA.fet&FET_LOCK)&&(battB.fet&FET_LOCK)))
	#endif
	{
		if(++attemptTimes>=5)
		{
			printf("!   Enable FET Fail!");
			return 0x20;
		}
		
		printf("\r\n#     Attempt#%d",attemptTimes);
		if(!(battA.fet&FET_LOCK)) Batt_WriteWord(battA.id, BATT_PowerControl, BATT_ENABLEFET);
		if(!(battB.fet&FET_LOCK)) Batt_WriteWord(battB.id, BATT_PowerControl, BATT_ENABLEFET);

		HAL_Delay(2000);
		
		Batt_ReadFET(&battA);
		Batt_ReadFET(&battB);
		printf(": A-0x%02X, B-0x%02X",battA.fet,battB.fet);
	}
	
	Batt_ReadFET(&battA);
	if((battA.status&BATT_ONBOARD)&&(battA.fet&PWR_ON)&&(battA.fet&FET_LOCK))	battA.status |= BATT_INUSE;
	Batt_ReadFET(&battB);
	if((battB.status&BATT_ONBOARD)&&(battB.fet&PWR_ON)&&(battB.fet&FET_LOCK))	battB.status |= BATT_INUSE;
	
	#ifdef SINGLE_BATTERY
	if(!((battA.status&BATT_INUSE)||(battB.status&BATT_INUSE)))
	#else  //DUAL_BATTERY
	if(!((battA.status&BATT_INUSE)&&(battB.status&BATT_INUSE)))
	#endif
	{
		printf("\r\n!   Battery Init Error!");
		return 0x80;
	}
	return 0x00;
}


void Batt_Measure(BattMsg* _batt, uint8_t _cmd)
{
	uint8_t regSta = 0;
	uint16_t regVal;
	
	switch(_cmd)
	{
		case 0x00: 
			regSta = Batt_ReadWord(_batt->id, BATT_FETStatus, &regVal);
			if(!regSta)	_batt->fet = regVal; break;
		
		case 0x01:
			regSta = Batt_ReadWord(_batt->id, BATT_Voltage, &regVal);
			if(!regSta)	_batt->voltage = regVal; break;
		
		case 0x02:
			regSta = Batt_ReadWord(_batt->id, BATT_Temperature, &regVal);
			if(!regSta)	_batt->temperature = (regVal-2731)*10;	break;// Kelvin -> Celsius
		
		case 0x03:
			regSta = Batt_ReadWord(_batt->id, BATT_Current, &regVal);
			if(!regSta)	_batt->current = regVal; break;
		
		case 0x04:
			regSta += Batt_ReadWord(_batt->id, BATT_FullChargeCapacity, &regVal);
			if(!regSta)	_batt->fullChargeCapacity = regVal*10; break;
		
		case 0x05:
			regSta += Batt_ReadWord(_batt->id, BATT_RemainingCapacity, &regVal);
			if(!regSta)	_batt->remainingCapacity = regVal*10; break;
		
		case 0x06:
			regSta += Batt_ReadWord(_batt->id, BATT_DesignCapacity, &regVal);
			if(!regSta)	_batt->designCapacity = regVal*10; break;
		
		case 0x07:
			regSta += Batt_ReadWord(_batt->id, BATT_RelativeSOC, &regVal);
			if(!regSta)	_batt->soc = regVal; break;
		
		default: break;		
	}
	
	if(regSta) _batt->status &= ~BATT_ONBOARD;		// Can't read battery
	else 	   _batt->status |= BATT_ONBOARD;
}

void Batt_ReadFET(BattMsg* _batt)
{
	uint8_t regSta = 0;
	uint16_t regVal;
	
	regSta = Batt_ReadWord(_batt->id, BATT_FETStatus, &regVal);
	if(!regSta)	
	{
		_batt->fet = regVal;
		_batt->status |= BATT_ONBOARD;
	}
	else			// Can't read battery
	{	
		_batt->fet = 0xC0;	// dummy flag, display in boot up
		_batt->status &= ~BATT_ONBOARD;
	}
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
