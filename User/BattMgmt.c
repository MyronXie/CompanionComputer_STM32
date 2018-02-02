/**
  ******************************************************************************
  * File Name		: BattMgmt.c
  * Description		: Battery Management Drivers (through I2C)
  *
  * Version			: v0.2
  * Created	Date	: 2017.09.25
  * Revised	Date	: 2018.02.02
  *
  * Author			: Mingye Xie
  ******************************************************************************
  */

#include "BattMgmt.h"

BattMsg battA,battB;
BattMsg* battX;
uint8_t battCycleCnt = 0;				// Counter for dispatch command for battmgmt system
uint8_t battAutoOff = 0;				// Flag for enable Auto Power Off Function

mavlink_battery_status_t mavBattTx;

uint8_t Batt_Init(void)
{
	uint8_t attemptTimes = 0;
	
	// Battery Message Init
	battA.id				= 0x16;
	battB.id				= 0x26;
	
	#ifdef SINGLE_BATTERY
	printf(" <SINGLE_BATTERY>");
	#endif
	
	// Read FET status
	Batt_Measure(&battA, 0x00);
	Batt_Measure(&battB, 0x00);
		
	// Check onboard status
	attemptTimes = 0;
	#ifdef SINGLE_BATTERY
	while(!((battA.status&BATT_ONBOARD)||(battB.status&BATT_ONBOARD)))		// At least one battery is onboard
	#else  //DUAL_BATTERY
	while(!((battA.status&BATT_ONBOARD)&&(battB.status&BATT_ONBOARD)))		// Both batteries should onboard
	#endif
	{
		if(++attemptTimes>3)
		{			
			if(!((battA.status&BATT_ONBOARD)||(battB.status&BATT_ONBOARD)))
			{
				printf("\r\n [ERR]  Both batteries Offboard!");
			}
			else
			{
				printf("\r\n [ERR]  ");
				if(!(battA.status&BATT_ONBOARD)) printf("battA ");
				if(!(battB.status&BATT_ONBOARD)) printf("battB ");
				printf("Offboard");
			}
			return ERR_BATT_OFFBOARD;
		}
		
		printf("\r\n [ACT]  Link Attempt#%d",attemptTimes);	
		Batt_Measure(&battA, 0x00);
		Batt_Measure(&battB, 0x00);		
		HAL_Delay(50);				
	}
	
	// Check power status
	#ifndef SINGLE_BATTERY
	if(!((battA.fet&PWR_ON)||(battB.fet&PWR_ON)))		// All batteries are power off
	{
		printf("\r\n [INFO] <Dev> Not powered by batteries!");
	}
	
	// Check Voltage Difference
	#ifndef INGORE_VDIFF
	
	// Read voltage
	Batt_ReadWord(battA.id, BATT_Voltage, &battA.voltage);
	Batt_ReadWord(battB.id, BATT_Voltage, &battB.voltage);
	
	// Vdiff should small than 100mV
	if(((battA.voltage>battB.voltage)?(battA.voltage-battB.voltage):(battB.voltage-battA.voltage))>=100)
	{
		printf("\r\n [ERR]  Vdiff > 100mV: %d, %d",battA.voltage,battB.voltage);
		return ERR_BATT_VDIFF;
	}
	#endif //INGORE_VDIFF
	#endif //SINGLE_BATTERY

	// Power ON Process
	attemptTimes = 0;
	#ifdef SINGLE_BATTERY
	while(!((battA.fet&PWR_ON)||(battB.fet&PWR_ON)))		// Both battery is powered off
	#else  //DUAL_BATTERY
	while(!((battA.fet&PWR_ON)&&(battB.fet&PWR_ON)))		// At least one battery is powered off
	#endif
	{
		if(++attemptTimes>4)
		{
			printf("\r\n [ERR]  Auto Power On Fail!");
			return ERR_BATT_POWERON;
		}
		
		printf("\r\n [ACT]  Auto-On Attempt#%d",attemptTimes);
		if(!(battA.fet&PWR_ON)) Batt_WriteWord(battA.id, BATT_PowerControl, BATT_POWERON);
		if(!(battB.fet&PWR_ON)) Batt_WriteWord(battB.id, BATT_PowerControl, BATT_POWERON);
		
		HAL_Delay(2000);	
		
		Batt_Measure(&battA, 0x00);
		Batt_Measure(&battB, 0x00);
		printf(": A-0x%02X, B-0x%02X",battA.fet,battB.fet);
					
	}
	
	// Enable FET process
	attemptTimes = 0;
	#ifdef SINGLE_BATTERY
	while(!((battA.fet&FET_LOCK)||(battB.fet&FET_LOCK)))
	#else  //DUAL_BATTERY
	while(!((battA.fet&FET_LOCK)&&(battB.fet&FET_LOCK)))
	#endif
	{
		if(++attemptTimes>4)
		{
			printf("\r\n [ERR]  Enable FET Fail!");
			return ERR_BATT_ENABLEFET;
		}
		
		printf("\r\n [ACT]  Enable FET Attempt#%d",attemptTimes);
		if(!(battA.fet&FET_LOCK)) Batt_WriteWord(battA.id, BATT_PowerControl, BATT_ENABLEFET);
		if(!(battB.fet&FET_LOCK)) Batt_WriteWord(battB.id, BATT_PowerControl, BATT_ENABLEFET);

		HAL_Delay(2000);
		
		Batt_Measure(&battA, 0x00);
		Batt_Measure(&battB, 0x00);
		printf(": A-0x%02X, B-0x%02X",battA.fet,battB.fet);
	}
	
	Batt_Measure(&battA, 0x00);
	if((battA.status&BATT_ONBOARD)&&(battA.fet&PWR_ON)&&(battA.fet&FET_LOCK))	battA.status |= BATT_INUSE;
	Batt_Measure(&battB, 0x00);
	if((battB.status&BATT_ONBOARD)&&(battB.fet&PWR_ON)&&(battB.fet&FET_LOCK))	battB.status |= BATT_INUSE;
	
	#ifdef SINGLE_BATTERY
	if(!((battA.status&BATT_INUSE)||(battB.status&BATT_INUSE)))
	#else  //DUAL_BATTERY
	if(!((battA.status&BATT_INUSE)&&(battB.status&BATT_INUSE)))
	#endif
	{
		printf("\r\n [ERR]  Battery Init Error!");
		return ERR_BATT_INIT;
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
			if(!regSta)	_batt->fet = regVal; 
			else		_batt->fet = 0xC0;		// dummy flag, display in boot up
			break;
		
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
		
		// can be removed when battery is stable
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


// battCycleCnt(0x00~0x27)
// 
// Measure : 000X 0NNN
// Send    : 000X 1NNN
//  X=0 battA, X=1 battB
//  N=0~7 type
// Judge   : 0010 0NNN
//  N=0~7 judge type
uint8_t Battery_Management(void)
{
	/********** Measure & Send Process **********/
	if(!(battCycleCnt&BATT_SYS_JUDGE))
	{
		// Select Battery
		if(!(battCycleCnt&BATT_SYS_BATTB))		battX = &battA;
		else									battX = &battB;
		
		// Measure Process
		if(!(battCycleCnt&BATT_SYS_SEND))
		{
			if(battX->status&BATT_INUSE) Batt_Measure(battX, battCycleCnt&BATT_SYS_MASK_CMD);
		}
		// Send Process
		else
		{
			switch(battCycleCnt&BATT_SYS_MASK_CMD)
			{
				// Printf data
				case 0x01:
					if(battX->status&BATT_INUSE)
					{
						if(battX->status&BATT_ONBOARD)
						{
							printf("\r\n [INFO] Batt_0x%02x:0x%02x%02x,%d,%d,%d,%d,%d,%d,%d", battX->id, battX->status, battX->fet, battX->temperature, battX->voltage, battX->current, battX->soc, battX->remainingCapacity, battX->fullChargeCapacity, battX->designCapacity);
							battX->lostCnt = 0;
						}
						else
						{
							battX->lostCnt++;
							if(battX->lostCnt<=3) printf("\r\n [INFO] Batt_0x%02x:Lost#%d!", battX->id, battX->lostCnt);
						}
					}
					break;

				default: break;
			}
		}
	}
	/********** Judge Process **********/
	else
	{
		switch(battCycleCnt&BATT_SYS_MASK_CMD)
		{		
			// Pack & Send battery status message
			case 0x01:
				#ifdef SINGLE_BATTERY
				if(!(battX->status&BATT_INUSE))
				{
					if(battA.status&BATT_INUSE)	battX = &battA;
					if(battB.status&BATT_INUSE)	battX = &battB;
				}
				#endif
				Battery_MavlinkPack(&mavBattTx);
				sendByteCnt = mavlink_msg_battery_status_pack(1, 1, &mavMsgTx, mavBattTx.id, mavBattTx.battery_function, mavBattTx.type, mavBattTx.temperature, mavBattTx.voltages, mavBattTx.current_battery, mavBattTx.current_consumed, mavBattTx.energy_consumed, mavBattTx.battery_remaining);
				Mavlink_SendMessage(&mavMsgTx, sendByteCnt);
				break;

			// Battery Link Lost
			case 0x02:
				if((battA.lostCnt>=3)||(battB.lostCnt>=3))
				{
					printf("\r\n [ERR]  Batt: Connect lost");
					return ERR_BATT_OFFBOARD;
				}
				break;
			
			#ifndef SINGLE_BATTERY
			#ifdef AUTO_POWEROFF
			// Judge Auto power off
			case 0x03:
				if((!battAutoOff)&&(!sysError))
				{
					if(((battA.fet&PWR_ON)&&(!(battB.fet&PWR_ON)))||((battB.fet&PWR_ON)&&(!(battA.fet&PWR_ON))))
					{
						battAutoOff = 1;	
					}
				}
				break;
				
			// Auto Power Off Process
			case 0x04:
				if(battAutoOff)
				{
					// Attempt every 2s
					if((battAutoOff+1)%2)
					{
						printf("\r\n [ACT]  Batt: Auto-Off Attempt#%d",(battAutoOff+1)/2);
						if(battA.fet&PWR_ON) Batt_WriteWord(battA.id, BATT_PowerControl, BATT_POWEROFF);
						if(battB.fet&PWR_ON) Batt_WriteWord(battB.id, BATT_PowerControl, BATT_POWEROFF);
					
						Batt_Measure(&battA, 0x00);
						Batt_Measure(&battB, 0x00);
						printf(": A-0x%02x, B-0x%02x",battA.fet,battB.fet);
					}
					
					// All Batteries have powered off
					if(!((battA.fet&PWR_ON)||(battB.fet&PWR_ON)))
					{
						printf("\r\n [INFO] Batt: Auto Power Off Success");
						battAutoOff = 0;
						battA.status &= ~BATT_INUSE;
						battB.status &= ~BATT_INUSE;
					}
					else if(++battAutoOff>=8)
					{
						printf("\r\n [ERR]  Batt: Auto Power Off Fail");
						battAutoOff = 0;
						return ERR_BATT_POWEROFF;
					}
				}
				break;
			#endif //AUTO_POWEROFF
			#endif //SINGLE_BATTERY

		}
	}		
	
	// Increase battCycleCnt
	battCycleCnt = (battCycleCnt+1)%40;
	
	return 0;
}


void Battery_MavlinkPack(mavlink_battery_status_t* mav)
{
	#ifndef SINGLE_BATTERY
	mav->id 				= 0x36;
	mav->battery_function	= MAV_BATTERY_FUNCTION_ALL;
	mav->type				= MAV_BATTERY_TYPE_LIPO;
	mav->temperature		= (battA.temperature + battB.temperature)/2;	// in centi-degrees celsius
	mav->voltages[0]		= (battA.voltage + battB.voltage)/2;			// in mV
	mav->current_battery	= battA.current + battB.current;				// in 10mA
	mav->current_consumed	= (battA.fullChargeCapacity - battA.remainingCapacity)+(battB.fullChargeCapacity - battB.remainingCapacity);	// in mAh
	mav->energy_consumed	= -1;											// -1: does not provide
	mav->battery_remaining	= (battA.soc + battB.soc)/2;					// 0%: 0, 100%: 100

// <Dev> Test Data	
//	mav->id 				= 0x36;
//	mav->battery_function	= MAV_BATTERY_FUNCTION_ALL;
//	mav->type				= MAV_BATTERY_TYPE_LIPO;
//	mav->temperature		= 1234;
//	mav->voltages[0]		= 23456;
//	mav->current_battery	= 987;
//	mav->current_consumed	= 345;
//	mav->energy_consumed	= -1;
//	mav->battery_remaining	= 89;
	#else
	mav->id 				= battX->id;
	mav->battery_function	= battX->status;									// Redefine this param
	mav->type				= sysError;
	mav->temperature		= battX->temperature;
	mav->voltages[0]		= battX->voltage;
	mav->current_battery	= battX->current;
	mav->current_consumed	= battX->fullChargeCapacity - battX->remainingCapacity;
	mav->energy_consumed	= -1;
	mav->battery_remaining	= battX->soc;
	#endif
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
