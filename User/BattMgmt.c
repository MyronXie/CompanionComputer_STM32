/**
  ******************************************************************************
  * File Name       : BattMgmt.c
  * Description     : Battery Management Drivers (through I2C)
  *
  * Version         : v0.2
  * Created Date    : 2017.09.25
  * Revised Date    : 2018.02.28
  *
  * Author          : Mingye Xie
  ******************************************************************************
  */

#include "BattMgmt.h"

BattMsg battA={"battA",0x16};
BattMsg battB={"battB",0x26};
BattMsg *battX;                 // Used for select specific battery
BattMsg *battO;                 // Used for single battery mode
uint8_t battMode = BATT_MODE_NONE;
uint8_t battCycleCnt = BATT_FUNC_CYCLE-1;      // Counter for dispatch command for battmgmt system
uint8_t battAutoOff = 0;        // Flag for enable Auto Power Off Function

mavlink_battery_status_t mavBattTx;

extern TIM_HandleTypeDef htim7;

uint8_t Batt_Init(void)
{
    uint8_t attemptTimes = 0;
    
    #ifdef SINGLE_BATTERY
    PRINTLOG(" <SINGLE_BATTERY>");
    #else  //DUAL_BATTERY
    PRINTLOG(" <DUAL_BATTERY>");
    #endif
    
    // Connect to battery
    Batt_Measure(&battA, BATT_MEAS_FET);
    Batt_Measure(&battB, BATT_MEAS_FET);
        
    // Check onboard status
    attemptTimes = 0;
    #ifdef SINGLE_BATTERY
    while(!((battA.status&BATT_ONBOARD)||(battB.status&BATT_ONBOARD)))      // At least one battery is onboard
    #else  //DUAL_BATTERY
    while(!((battA.status&BATT_ONBOARD)&&(battB.status&BATT_ONBOARD)))      // Both batteries should onboard
    #endif
    {
        if(++attemptTimes>ATTEMPT_TIMES)
        {			
            PRINTLOG("\r\n [ERR]  %s %s Offboard",(!(battA.status&BATT_ONBOARD))?battA.name:"",(!(battB.status&BATT_ONBOARD))?battB.name:"");
            if(!(battA.status&BATT_ONBOARD))    sysBattery|=ERR_BATTA;
            if(!(battB.status&BATT_ONBOARD))    sysBattery|=ERR_BATTB;
            return ERR_BATT_OFFBOARD;
        }
        
        HAL_Delay(20);
        PRINTLOG("\r\n [ACT]  Connecting #%d",attemptTimes);
        Batt_Measure(&battA, BATT_MEAS_FET);
        Batt_Measure(&battB, BATT_MEAS_FET);
        PRINTLOG(": A-0x%02X, B-0x%02X",battA.fet,battB.fet);
    }
    
    // Confirm battery number mode
    if((battA.status&BATT_ONBOARD)&&(battB.status&BATT_ONBOARD))
        battMode = BATT_MODE_DUAL;
    else
    {
        battMode = BATT_MODE_SINGLE;
        if(battA.status&BATT_ONBOARD)   battO = &battA;
        if(battB.status&BATT_ONBOARD)   battO = &battB;
    }
    
    // Check Voltage Difference
    #ifndef INGORE_VDIFF
    if(battMode == BATT_MODE_DUAL)
    {
        // Read voltage
        Batt_Measure(&battA, BATT_MEAS_VOLT);
        Batt_Measure(&battB, BATT_MEAS_VOLT);
        
        // Vdiff should small than 100mV
        if(((battA.voltage>battB.voltage)?(battA.voltage-battB.voltage):(battB.voltage-battA.voltage)) >= VDIFF_TOLERENCE)
        {
            PRINTLOG("\r\n [ERR]  Voltage mismatch: A-%dmV, B-%dmV",battA.voltage,battB.voltage);
            if(battA.voltage>battB.voltage) sysBattery|=ERR_BATTA;
            else                            sysBattery|=ERR_BATTB;
            return ERR_BATT_VDIFF;

        }
    }
    #endif  //INGORE_VDIFF

    // Power ON Process
    attemptTimes = 0;
    if(battMode == BATT_MODE_SINGLE)        // Single battery mode
    {
        while(!(battO->fet&PWR_ON))
        {
            if(++attemptTimes>ATTEMPT_TIMES)
            {
                PRINTLOG("\r\n [ERR]  %s Power On Fail",battO->name);
                return ERR_BATT_POWERON;
            }
            
            PRINTLOG("\r\n [ACT]  Power On   #%d",attemptTimes);
            if(!(battO->fet&PWR_ON)) Batt_WriteWord(battO->id, BATT_PowerControl, BATT_POWERON);
            HAL_Delay(2000);
            Batt_Measure(battO, BATT_MEAS_FET);
            PRINTLOG(": %s-0x%02X",battO->name,battO->fet);	
        }
    }
    else if(battMode == BATT_MODE_DUAL)     // Dual battery mode
    {
        while(!((battA.fet&PWR_ON)&&(battB.fet&PWR_ON)))        // At least one battery is powered off
        {
            if(++attemptTimes>ATTEMPT_TIMES)
            {
                PRINTLOG("\r\n [ERR]  %s %s Power On Fail",(!(battA.fet&PWR_ON))?"battA":"",(!(battB.fet&PWR_ON))?"battB":"");
                if(!(battA.fet&PWR_ON)) sysBattery|=ERR_BATTA;
                if(!(battB.fet&PWR_ON)) sysBattery|=ERR_BATTB;
                return ERR_BATT_POWERON;
            }
            
            PRINTLOG("\r\n [ACT]  Power On   #%d",attemptTimes);
            if(!(battA.fet&PWR_ON)) Batt_WriteWord(battA.id, BATT_PowerControl, BATT_POWERON);
            if(!(battB.fet&PWR_ON)) Batt_WriteWord(battB.id, BATT_PowerControl, BATT_POWERON);
            HAL_Delay(2000);
            Batt_Measure(&battA, BATT_MEAS_FET);
            Batt_Measure(&battB, BATT_MEAS_FET);
            PRINTLOG(": A-0x%02X, B-0x%02X",battA.fet,battB.fet);	
        }
    }

    // Enable FET process
    attemptTimes = 0;
    if(battMode == BATT_MODE_SINGLE)        // Single battery mode
    {
        while(!(battO->fet&FET_LOCK))
        {
            if(++attemptTimes>ATTEMPT_TIMES)
            {
                PRINTLOG("\r\n [ERR]  %s Enable FET Fail",battO->name);
                return ERR_BATT_ENABLEFET;
            }
            
            PRINTLOG("\r\n [ACT]  Enable FET #%d",attemptTimes);
            if(!(battO->fet&FET_LOCK))   Batt_WriteWord(battO->id, BATT_PowerControl, BATT_ENABLEFET);
            HAL_Delay(2000);
            Batt_Measure(battO, BATT_MEAS_FET);
            PRINTLOG(": %s-0x%02X",battO->name,battO->fet);	
        }
    }
    else if(battMode == BATT_MODE_DUAL)     // Dual battery mode
    {
        while(!((battA.fet&FET_LOCK)&&(battB.fet&FET_LOCK)))        // At least one battery disable fet
        {
            if(++attemptTimes>ATTEMPT_TIMES)
            {
                PRINTLOG("\r\n [ERR]  %s %s Enable FET Fail",(!(battA.fet&FET_LOCK))?battA.name:"",(!(battB.fet&FET_LOCK))?battB.name:"");
                if(!(battA.fet&FET_LOCK))   sysBattery|=ERR_BATTA;
                if(!(battB.fet&FET_LOCK))   sysBattery|=ERR_BATTB;
                return ERR_BATT_ENABLEFET;
            }
            
            PRINTLOG("\r\n [ACT]  Enable FET Attempt#%d",attemptTimes);
            if(!(battA.fet&FET_LOCK))   Batt_WriteWord(battA.id, BATT_PowerControl, BATT_ENABLEFET);
            if(!(battB.fet&FET_LOCK))   Batt_WriteWord(battB.id, BATT_PowerControl, BATT_ENABLEFET);
            HAL_Delay(2000);
            Batt_Measure(&battA, BATT_MEAS_FET);
            Batt_Measure(&battB, BATT_MEAS_FET);
            PRINTLOG(": A-0x%02X, B-0x%02X",battA.fet,battB.fet);
        }
    }
    
    // Check battery init status    
    if(battMode == BATT_MODE_SINGLE)        // Single battery mode
    {
        Batt_Measure(battO, BATT_MEAS_FET);
        if((battO->status&BATT_ONBOARD)&&(battO->fet&PWR_ON)&&(battO->fet&FET_LOCK))   battO->status |= BATT_INUSE;
        
        if(!(battO->status&BATT_INUSE))
        {
            PRINTLOG("\r\n [ERR]  %s Battery Init Error",battO->name);
            return ERR_BATT_INIT;
        }
    }
    else if(battMode == BATT_MODE_DUAL)     // Dual battery mode
    {
        Batt_Measure(&battA, BATT_MEAS_FET);
        Batt_Measure(&battB, BATT_MEAS_FET);
        if((battA.status&BATT_ONBOARD)&&(battA.fet&PWR_ON)&&(battA.fet&FET_LOCK))   battA.status |= BATT_INUSE;
        if((battB.status&BATT_ONBOARD)&&(battB.fet&PWR_ON)&&(battB.fet&FET_LOCK))   battB.status |= BATT_INUSE;
        
        if(!((battA.status&BATT_INUSE)&&(battB.status&BATT_INUSE)))
        {
            PRINTLOG("\r\n [ERR]  %s %s Battery Init Error",(!(battA.status&BATT_INUSE))?"battA":"",(!(battB.status&BATT_INUSE))?"battB":"");
            if(!(battA.status&BATT_INUSE))  sysBattery|=ERR_BATTA;
            if(!(battB.status&BATT_INUSE))  sysBattery|=ERR_BATTB;
            return ERR_BATT_INIT;
        }
    }

    PRINTLOG("\r\n [INFO] Battery Init Success");
    return NO_ERR;
}


void Batt_Measure(BattMsg* _batt, uint8_t _cmd)
{
    uint8_t regSta = 0;
    uint16_t regVal;
    
    switch(_cmd)
    {
        case BATT_MEAS_FET: 
            regSta = Batt_ReadWord(_batt->id, BATT_FETStatus, &regVal);
            if(!regSta) _batt->fet = regVal; 
            else        _batt->fet = BATT_DUMMY;          // dummy flag, display in boot up
            break;
        
        case BATT_MEAS_VOLT:
            regSta = Batt_ReadWord(_batt->id, BATT_Voltage, &regVal);
            if(!regSta) _batt->voltage = regVal; break;
        
        case BATT_MEAS_TEMP:
            regSta = Batt_ReadWord(_batt->id, BATT_Temperature, &regVal);
            if(!regSta) _batt->temperature = (regVal-2731)*10;	break;  // Kelvin -> Celsius
        
        case BATT_MEAS_CURR:
            regSta = Batt_ReadWord(_batt->id, BATT_Current, &regVal);
            if(!regSta) _batt->current = regVal; break;

        case BATT_MEAS_SOC:
            regSta += Batt_ReadWord(_batt->id, BATT_RelativeSOC, &regVal);
            if(!regSta) _batt->soc = regVal; break;
        
        case BATT_MEAS_RCAP:
            regSta += Batt_ReadWord(_batt->id, BATT_RemainingCapacity, &regVal);
            if(!regSta) _batt->remainingCapacity = regVal*10; break;

        case BATT_MEAS_FCCAP:
            regSta += Batt_ReadWord(_batt->id, BATT_FullChargeCapacity, &regVal);
            if(!regSta) _batt->fullChargeCapacity = regVal*10; break;

        case BATT_MEAS_DCAP:
            regSta += Batt_ReadWord(_batt->id, BATT_DesignCapacity, &regVal);
            if(!regSta) _batt->designCapacity = regVal*10; break;

        default: break;		
    }
    
    if(regSta)  _batt->status &= ~BATT_ONBOARD;      // Can't connect to battery
    else        _batt->status |= BATT_ONBOARD;
}


// battCycleCnt(0x00~0x27)
// 
// Measure : 000X 0NNN
// Send    : 000X 1NNN
//  X=0 battA, X=1 battB
//  N=0~7 type
// Judge   : 0010 0NNN
//  N=0~7 judge type
//
uint8_t Battery_Management(void)
{
    // Increase battCycleCnt
    battCycleCnt = (battCycleCnt+1)%BATT_FUNC_CYCLE;
    
    /********** Measure & Send Process **********/
    if(!(battCycleCnt&BATT_SYS_JUDGE))
    {
        // Select Battery
        if(!(battCycleCnt&BATT_SYS_BATTB))      battX = &battA;
        else                                    battX = &battB;
        
        // Measure Process
        if(!(battCycleCnt&BATT_SYS_SEND))
        {
            if(battX->status&BATT_ONBOARD)  Batt_Measure(battX, battCycleCnt&BATT_SYS_MASK_CMD);
        }
        // Send Process
        else
        {
            switch(battCycleCnt&BATT_SYS_MASK_CMD)
            {
                // print data
                case 0x01:
                    if(battX->status&BATT_INUSE)
                    {
                        if(battX->status&BATT_ONBOARD)
                        {
                            PRINTLOG("\r\n [INFO] %s: 0x%02X%02X,%d,%d,%d,%d,%d,%d,%d", 
                                    battX->name, battX->status, battX->fet, battX->temperature, battX->voltage, battX->current,
                                    battX->soc, battX->remainingCapacity, battX->fullChargeCapacity, battX->designCapacity);
                            battX->lostCnt = 0;
                        }
                        else
                        {
                            if(battX->lostCnt < ATTEMPT_TIMES)
                            {
                                battX->lostCnt++;
                                PRINTLOG("\r\n [INFO] %s Lost #%d!", battX->name, battX->lostCnt);
                            }
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
                Battery_MavlinkPack(&mavBattTx,battMode);
                sendCnt = mavlink_msg_battery_status_pack(1, 1, &mavMsgTx, mavBattTx.id, mavBattTx.battery_function, mavBattTx.type,
                                                                mavBattTx.temperature, mavBattTx.voltages, mavBattTx.current_battery,
                                                                mavBattTx.current_consumed, mavBattTx.energy_consumed, mavBattTx.battery_remaining);
                Mavlink_SendMessage(&mavMsgTx, sendCnt);
                break;

            // Battery Link Lost
            case 0x02:
                if(battMode == BATT_MODE_SINGLE)        // Single battery mode
                {
                    if(battO->lostCnt == ATTEMPT_TIMES)
                    {
                        PRINTLOG("\r\n [ERR]  %s Connect lost",battO->name);
                        battMode = BATT_MODE_NONE;
                        return ERR_BATT_OFFBOARD;
                    }
                }
                else if(battMode == BATT_MODE_DUAL)     // Dual battery mode
                {
                    if((battA.lostCnt == ATTEMPT_TIMES)||(battB.lostCnt == ATTEMPT_TIMES))
                    {
                        PRINTLOG("\r\n [ERR]  %s %s Connect lost",(battA.lostCnt==ATTEMPT_TIMES)?battA.name:"",(battB.lostCnt==ATTEMPT_TIMES)?battB.name:"");
                        if(battA.lostCnt == ATTEMPT_TIMES) sysBattery|=ERR_BATTA;
                        if(battB.lostCnt == ATTEMPT_TIMES) sysBattery|=ERR_BATTB;
                        return ERR_BATT_OFFBOARD;
                    }
                }
                break;
            
            #ifdef AUTO_POWEROFF
            // Judge Auto power off
            case 0x03:
                if((battMode == BATT_MODE_DUAL)&&(!battAutoOff))
                {
                    // One battery is powered down
                    if(((battA.fet&PWR_ON)&&(!(battB.fet&PWR_ON)))||((battB.fet&PWR_ON)&&(!(battA.fet&PWR_ON))))
                    {
                        if(sysFlying)   // Need a flag to ensure can/can not power off battery
                        {
                            PRINTLOG("\r\n [ERR]  %s %s Lost power in the air",(!(battA.fet&PWR_ON))?"battA":"",(!(battB.fet&PWR_ON))?"battB":"");
                            if(!(battA.fet&PWR_ON)) sysBattery|=ERR_BATTA;
                            if(!(battB.fet&PWR_ON)) sysBattery|=ERR_BATTB;
                            return ERR_BATT_LOSTAIR;
                        }
                        else
                        {
                            battAutoOff = 1;
                            Mavlink_SendLog(MSG_BATTERY, "Start Auto Power Off Process");
                        }
                    }
                }
                break;
                
            // Auto Power Off Process
            case 0x04:
                if((battAutoOff<8)&&(battAutoOff>0))
                {
                    // All Batteries have powered off
                    if(!((battA.fet&PWR_ON)||(battB.fet&PWR_ON)))
                    {
                        PRINTLOG("\r\n [INFO] Batt: Auto Power Off Success");
                        battAutoOff = 0;
                        battA.status &= ~BATT_INUSE;
                        battB.status &= ~BATT_INUSE;
                        battMode = BATT_MODE_NONE;
                    }
                    else if(++battAutoOff==8)
                    {
                        PRINTLOG("\r\n [ERR]  Batt: Auto Power Off Fail");
                        if(battA.fet&PWR_ON)    sysBattery|=ERR_BATTA;
                        if(battB.fet&PWR_ON)    sysBattery|=ERR_BATTB;
                        //Need a state to record one is on and one is off
                        return ERR_BATT_POWEROFF;
                    }
                    
                    // Attempt every 2s
                    if((battAutoOff+1)%2)
                    {
                        PRINTLOG("\r\n [ACT]  Batt: Auto-Off   #%d",(battAutoOff+1)/2);
                        if(battA.fet&PWR_ON)    Batt_WriteWord(battA.id, BATT_PowerControl, BATT_POWEROFF);
                        if(battB.fet&PWR_ON)    Batt_WriteWord(battB.id, BATT_PowerControl, BATT_POWEROFF);
                    
                        Batt_Measure(&battA, BATT_MEAS_FET);
                        Batt_Measure(&battB, BATT_MEAS_FET);
                        PRINTLOG(": A-0x%02X, B-0x%02X",battA.fet,battB.fet);
                    }
                }
                break;
            #endif  //AUTO_POWEROFF
                
            // Re-connect battery    
            case 0x05:
                if(battMode == BATT_MODE_NONE)
                {
                    Batt_Measure(&battA, BATT_MEAS_FET);
                    Batt_Measure(&battB, BATT_MEAS_FET);
                    #ifdef SINGLE_BATTERY
                    if(((battA.status&BATT_ONBOARD)||(battB.status&BATT_ONBOARD)))
                    {
                        if(battA.status&BATT_ONBOARD)   battO = &battA;
                        if(battB.status&BATT_ONBOARD)   battO = &battB;
                        PRINTLOG("\r\n [INFO] %s Re-Connect",battO->name);
                        //HAL_TIM_Base_Stop_IT(&htim7);
                        //PRINTLOG("\r\n [INFO] ReInit: Battery");
                        //Batt_Init();
                        //HAL_TIM_Base_Start_IT(&htim7);
                        
                        battMode = BATT_MODE_SINGLE;
                    }
                    #endif
                }
                break;
                
            default:break;
            
        }
    }		
  
    return NO_ERR;
}


void Battery_MavlinkPack(mavlink_battery_status_t* mav,uint8_t num)
{
    if(num == 2)
    {
        mav->id                 = 0x36;
        mav->battery_function   = sysBattery;                                   // Redefine this param
        mav->type               = MAV_BATTERY_TYPE_LIPO;
        mav->temperature        = (battA.temperature + battB.temperature)/2;    // in centi-degrees celsius
        mav->voltages[0]        = (battA.voltage + battB.voltage)/2;            // in mV
        mav->current_battery    = battA.current + battB.current;                // in 10mA
        mav->current_consumed   = (battA.fullChargeCapacity - battA.remainingCapacity)+(battB.fullChargeCapacity - battB.remainingCapacity);	// in mAh
        mav->energy_consumed    = -1;                                           // -1: does not provide
        mav->battery_remaining  = (battA.soc + battB.soc)/2;                    // 0%: 0, 100%: 100

//  <Dev> Test Data	
//      mav->id                 = 0x36;
//      mav->battery_function   = MAV_BATTERY_FUNCTION_ALL;
//      mav->type               = MAV_BATTERY_TYPE_LIPO;
//      mav->temperature        = 1234;
//      mav->voltages[0]        = 23456;
//      mav->current_battery    = 987;
//      mav->current_consumed   = 345;
//      mav->energy_consumed    = -1;
//      mav->battery_remaining  = 89;
    }
    else
    {
        mav->id                 = battO->id;
        mav->battery_function   = sysBattery;
        mav->type               = MAV_BATTERY_TYPE_LIPO;
        mav->temperature        = battO->temperature;
        mav->voltages[0]        = battO->voltage;
        mav->current_battery    = battO->current;
        mav->current_consumed   = battO->fullChargeCapacity - battX->remainingCapacity;
        mav->energy_consumed    = -1;
        mav->battery_remaining  = battO->soc;
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
