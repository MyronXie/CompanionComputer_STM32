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
	uint8_t attemptTimes = 0;
	uint16_t regVal;
	
	// Battery Message Init
	battA.id					= 0x16;
	battB.id					= 0x26;
	
	// Read 1st FET status
	Batt_ReadFET(&battA);
	Batt_ReadFET(&battB);
	
	// Check onboard status
	attemptTimes = 0;
	while((battA.status&BATT_OFFOBARD)||(battB.status&BATT_OFFOBARD))
	{
		if(++attemptTimes>=5)
		{
			printf("!   Battery offboard!");
			if(battA.status&PWR_ON) Batt_WriteWord(battA.id, BATT_PowerControl, BATT_POWEROFF);
			if(battB.status&PWR_ON) Batt_WriteWord(battB.id, BATT_PowerControl, BATT_POWEROFF);	
			HAL_Delay(2000);
			if(battA.status&PWR_ON) Batt_WriteWord(battA.id, BATT_PowerControl, BATT_POWEROFF);
			if(battB.status&PWR_ON) Batt_WriteWord(battB.id, BATT_PowerControl, BATT_POWEROFF);	
			return 0x01;
			
		}
		
		printf("\r\n#   Link #%d",attemptTimes);	
		Batt_ReadFET(&battA);
		Batt_ReadFET(&battB);
		if(battA.status&BATT_OFFOBARD) printf(" ,battA offboard");
		if(battB.status&BATT_OFFOBARD) printf(" ,battB offboard");
		
		HAL_Delay(5);				
	}
	
	// Check power status
	if(!((battA.status&PWR_ON)||(battB.status&PWR_ON)))		// All batteries are power off
	{
		printf("\r\n!   Not powered by batteries!");
		return 0x02;
	}

	// Read voltage
	status = Batt_ReadWord(battA.id, BATT_Voltage, &regVal);
	if(!status)	battA.voltage = regVal;
	status = Batt_ReadWord(battB.id, BATT_Voltage, &regVal);
	if(!status)	battB.voltage = regVal;
	

	// Check Voltage : difference <= 200mV
	if(((battA.voltage>battB.voltage)?(battA.voltage-battB.voltage):(battB.voltage-battA.voltage))>=200)
	{
		printf("\r\n#   Voltage difference > 200mV! ");
		if(battA.status&PWR_ON) Batt_WriteWord(battA.id, BATT_PowerControl, BATT_POWEROFF);
		if(battB.status&PWR_ON) Batt_WriteWord(battB.id, BATT_PowerControl, BATT_POWEROFF);	
		HAL_Delay(2000);
		if(battA.status&PWR_ON) Batt_WriteWord(battA.id, BATT_PowerControl, BATT_POWEROFF);
		if(battB.status&PWR_ON) Batt_WriteWord(battB.id, BATT_PowerControl, BATT_POWEROFF);	
		return 0x04;
	}

	
	// Power ON
	attemptTimes = 0;
	while(!((battA.status&PWR_ON)&&(battB.status&PWR_ON)))
	{
		if(++attemptTimes>=5)
		{
			printf("!   Can't power on battery!");
			return 0x10;
		}
		
		printf("\r\n#   Power On #%d",attemptTimes);
		if(!(battA.status&PWR_ON)) Batt_WriteWord(battA.id, BATT_PowerControl, BATT_POWERON);
		else printf(" ,battA ON");
		if(!(battB.status&PWR_ON)) Batt_WriteWord(battB.id, BATT_PowerControl, BATT_POWERON);
		else printf(" ,battB ON");
		
		Batt_ReadFET(&battA);
		Batt_ReadFET(&battB);
		
		HAL_Delay(1500);				
	}
	
	// Enable FET
	attemptTimes = 0;
	while(!((battA.status&FET_LOCK)&&(battB.status&FET_LOCK)))
	{
		if(++attemptTimes>=5)
		{
			printf("!   Can't Enable FET!");
			return 0x20;
		}
		
		printf("\r\n#   Enable FET #%d",attemptTimes);
		if(!(battA.status&FET_LOCK)) Batt_WriteWord(battA.id, BATT_PowerControl, BATT_ENABLEFET);
		else printf(" ,battA ON");
		if(!(battB.status&FET_LOCK)) Batt_WriteWord(battB.id, BATT_PowerControl, BATT_ENABLEFET);
		else printf(" ,battB ON");
		
		Batt_ReadFET(&battA);
		Batt_ReadFET(&battB);
		
		HAL_Delay(2000);				
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
