/**
  ******************************************************************************
  * File Name       : BattMgmt.c
  * Description     : Battery Management Drivers (through I2C)
  *
  * Version         : v0.3
  * Created Date    : 2017.09.25
  * Revised Date    : 2018.03.02
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
uint8_t battPwrOff = 0;        // Flag for enable Auto Power Off Function

mavlink_battery_status_t mavBattTx;

extern TIM_HandleTypeDef htim7;

uint8_t battReinit = 0;

uint8_t Batt_Init(void)
{
    static uint32_t timeTick = 0;
    static uint8_t stage = 0;
    static uint8_t attemptTimes = 0;
    
    switch(stage)
    {
        case BATT_INIT_BEGIN:
            #ifdef SINGLE_BATTERY
            PRINTLOG(" <SINGLE_BATTERY>");
            #else  //DUAL_BATTERY
            PRINTLOG(" <DUAL_BATTERY>");
            #endif
            attemptTimes = 0;
            stage = BATT_INIT_CNCT;
        
        case BATT_INIT_CNCT:      
            // Connect to battery
            Batt_Measure(&battA, BATT_MEAS_FET);
            Batt_Measure(&battB, BATT_MEAS_FET);
            PRINTLOG("\r\n [ACT]  Connecting #%d: A-0x%02X, B-0x%02X",attemptTimes,battA.fet,battB.fet);
            // Check onboard status
            #ifdef SINGLE_BATTERY
            if((battA.status&BATT_ONBOARD)||(battB.status&BATT_ONBOARD))      // At least one battery is onboard
            #else  //DUAL_BATTERY
            if((battA.status&BATT_ONBOARD)&&(battB.status&BATT_ONBOARD))      // Both batteries should onboard
            #endif
            {
                stage = BATT_INIT_MODE;   attemptTimes = 0;   return BATT_WAITING;
            }
            else 
            {
                attemptTimes++;
                timeTick = HAL_GetTick();
                stage = BATT_INIT_CNCT_WAIT;
            }
            
        case 0x02:
            // Waiting for delay            
            if(attemptTimes>ATTEMPT_TIMES)
            {
                PRINTLOG("\r\n [ERR]  %s %s Offboard",(!(battA.status&BATT_ONBOARD))?battA.name:"",(!(battB.status&BATT_ONBOARD))?battB.name:"");
                if(!(battA.status&BATT_ONBOARD))    sysBattery|=ERR_BATTA;
                if(!(battB.status&BATT_ONBOARD))    sysBattery|=ERR_BATTB;
                if((battA.status&BATT_ONBOARD)||(battB.status&BATT_ONBOARD))
                {
                    if(battA.status&BATT_ONBOARD)   battO = &battA;
                    if(battB.status&BATT_ONBOARD)   battO = &battB;
                    battMode = BATT_MODE_DUAL_ONE;
                }
                stage = BATT_INIT_BEGIN;
                return ERR_BATT_OFFBOARD;
            }
            if(HAL_GetTick() - timeTick < CNCT_DELAY)
            {
                stage = BATT_INIT_CNCT_WAIT;   return BATT_WAITING;
            }
            else
            {
                stage = BATT_INIT_CNCT;   return BATT_WAITING;
            }

        case BATT_INIT_MODE:
            // Confirm battery number mode
            if((battA.status&BATT_ONBOARD)&&(battB.status&BATT_ONBOARD))
                battMode = BATT_MODE_DUAL;
            else
            {
                battMode = BATT_MODE_SINGLE;
                if(battA.status&BATT_ONBOARD)   battO = &battA;
                if(battB.status&BATT_ONBOARD)   battO = &battB;
            }
            stage = BATT_INIT_VDIFF;
            
        case BATT_INIT_VDIFF:
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
                    stage = BATT_INIT_BEGIN;
                    return ERR_BATT_VDIFF;
                }
            }
            #endif  //INGORE_VDIFF
            stage = BATT_INIT_PWRON;
                     
        case BATT_INIT_PWRON:
            // Power ON Process
            if(battMode == BATT_MODE_SINGLE)        // Single battery mode
            {
                Batt_Measure(battO, BATT_MEAS_FET);
                PRINTLOG("\r\n [ACT]  Power On   #%d: %s-0x%02X",attemptTimes,battO->name,battO->fet);                
                if(battO->fet&PWR_ON)
                {
                    stage = BATT_INIT_ENFET;   attemptTimes = 0;   return BATT_WAITING;
                }
                else
                {
                    Batt_WriteWord(battO->id, BATT_PowerControl, BATT_POWERON);
                    attemptTimes++;
                    timeTick = HAL_GetTick();
                    stage = BATT_INIT_PWRON_WAIT;
                }
            }
            else if(battMode == BATT_MODE_DUAL)          // Dual battery mode
            {
                Batt_Measure(&battA, BATT_MEAS_FET);
                Batt_Measure(&battB, BATT_MEAS_FET);
                PRINTLOG("\r\n [ACT]  Power On   #%d: A-0x%02X, B-0x%02X",attemptTimes,battA.fet,battB.fet);	                
                if((battA.fet&PWR_ON)&&(battB.fet&PWR_ON))
                {
                    stage = BATT_INIT_ENFET;   attemptTimes = 0;   return BATT_WAITING;
                }
                else
                {
                    if(!(battA.fet&PWR_ON)) Batt_WriteWord(battA.id, BATT_PowerControl, BATT_POWERON);
                    if(!(battB.fet&PWR_ON)) Batt_WriteWord(battB.id, BATT_PowerControl, BATT_POWERON);
                    attemptTimes++;
                    timeTick = HAL_GetTick();
                    stage = BATT_INIT_PWRON_WAIT;
                }
            }

        case BATT_INIT_PWRON_WAIT:
            // Waiting for delay            
            if(attemptTimes>ATTEMPT_TIMES)
            {
                if(battMode == BATT_MODE_SINGLE)        // Single battery mode
                {
                    PRINTLOG("\r\n [ERR]  %s Power On Fail",battO->name);
                    if(battO == &battA) sysBattery|=ERR_BATTA;
                    if(battO == &battB) sysBattery|=ERR_BATTB;
                }
                else if(battMode == BATT_MODE_DUAL)     // Dual battery mode
                {
                    PRINTLOG("\r\n [ERR]  %s %s Power On Fail",(!(battA.fet&PWR_ON))?"battA":"",(!(battB.fet&PWR_ON))?"battB":"");
                    if(!(battA.fet&PWR_ON)) sysBattery|=ERR_BATTA;
                    if(!(battB.fet&PWR_ON)) sysBattery|=ERR_BATTB;
                }
                return ERR_BATT_POWERON;
            }
            if(HAL_GetTick() - timeTick < PWRON_DELAY)
            {
                stage = BATT_INIT_PWRON_WAIT;   return BATT_WAITING;
            }
            else
            {
                stage = BATT_INIT_PWRON;   return BATT_WAITING;
            }

        case BATT_INIT_ENFET:
            // Enable FET process
            if(battMode == BATT_MODE_SINGLE)        // Single battery mode
            {
                Batt_Measure(battO, BATT_MEAS_FET);
                PRINTLOG("\r\n [ACT]  Enable FET #%d: %s-0x%02X",attemptTimes,battO->name,battO->fet);	               
                if(battO->fet&FET_LOCK)
                {
                    stage = BATT_INIT_CHECK;    attemptTimes = 0;   return BATT_WAITING;
                }
                else
                {
                    Batt_WriteWord(battO->id, BATT_PowerControl, BATT_ENABLEFET);
                    attemptTimes++;
                    timeTick = HAL_GetTick();
                    stage = BATT_INIT_ENFET_WAIT;
                }
            }
            else if(battMode == BATT_MODE_DUAL)          // Dual battery mode
            {
                Batt_Measure(&battA, BATT_MEAS_FET);
                Batt_Measure(&battB, BATT_MEAS_FET);
                PRINTLOG("\r\n [ACT]  Enable FET #%d: A-0x%02X, B-0x%02X",attemptTimes,battA.fet,battB.fet);	                
                if((battA.fet&FET_LOCK)&&(battB.fet&FET_LOCK))
                {
                    stage = BATT_INIT_CHECK;    attemptTimes = 0;   return BATT_WAITING;
                }
                else
                {
                    if(!(battA.fet&FET_LOCK)) Batt_WriteWord(battA.id, BATT_PowerControl, BATT_ENABLEFET);
                    if(!(battB.fet&FET_LOCK)) Batt_WriteWord(battB.id, BATT_PowerControl, BATT_ENABLEFET);
                    attemptTimes++;
                    timeTick = HAL_GetTick();
                    stage = BATT_INIT_ENFET_WAIT;
                }
            }

        case BATT_INIT_ENFET_WAIT:
            // Waiting for delay            
            if(attemptTimes>ATTEMPT_TIMES)
            {
                if(battMode == BATT_MODE_SINGLE)        // Single battery mode
                {
                    PRINTLOG("\r\n [ERR]  %s Enable FET Fail",battO->name);
                    if(battO == &battA) sysBattery|=ERR_BATTA;
                    if(battO == &battB) sysBattery|=ERR_BATTB;
                }
                else if(battMode == BATT_MODE_DUAL)     // Dual battery mode
                {
                    PRINTLOG("\r\n [ERR]  %s %s Enable FET Fail",(!(battA.fet&PWR_ON))?"battA":"",(!(battB.fet&PWR_ON))?"battB":"");
                    if(!(battA.fet&PWR_ON)) sysBattery|=ERR_BATTA;
                    if(!(battB.fet&PWR_ON)) sysBattery|=ERR_BATTB;
                }
                stage = BATT_INIT_BEGIN;
                return ERR_BATT_ENABLEFET;
            }
            if(HAL_GetTick() - timeTick < ENFET_DELAY)
            {
                stage = BATT_INIT_ENFET_WAIT;   return BATT_WAITING;
            }
            else
            {
                stage = BATT_INIT_ENFET;   return BATT_WAITING;
            }

        case BATT_INIT_CHECK:
            // Check battery init status    
            if(battMode == BATT_MODE_SINGLE)        // Single battery mode
            {
                Batt_Measure(battO, BATT_MEAS_FET);
                if((battO->status&BATT_ONBOARD)&&(battO->fet&PWR_ON)&&(battO->fet&FET_LOCK))   battO->status |= BATT_INUSE;
                
                if(!(battO->status&BATT_INUSE))
                {
                    PRINTLOG("\r\n [ERR]  %s Battery Init Error",battO->name);
                    stage = BATT_INIT_BEGIN;
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
                    stage = BATT_INIT_BEGIN;
                    return ERR_BATT_INIT;
                }
            }
            stage = BATT_INIT_END;
            
        case BATT_INIT_END:
            PRINTLOG("\r\n [INFO] Battery Init Success");
            stage = BATT_INIT_BEGIN;
            return NO_ERR;

        default:
            stage = BATT_INIT_BEGIN;
            return ERR_BATT_INIT;
    }
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
                    Batt_Measure(battX, BATT_MEAS_FET);
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
                if(sysConnect)
                {
                    Battery_MavlinkPack(&mavBattTx,battMode);
                    sendCnt = mavlink_msg_battery_status_pack(1, 1, &mavMsgTx, mavBattTx.id, mavBattTx.battery_function, mavBattTx.type,
                                                                    mavBattTx.temperature, mavBattTx.voltages, mavBattTx.current_battery,
                                                                    mavBattTx.current_consumed, mavBattTx.energy_consumed, mavBattTx.battery_remaining);
                    Mavlink_SendMessage(&mavMsgTx, sendCnt);
                }
                break;

            // Battery Link Lost
            case 0x02:
                if(battMode == BATT_MODE_SINGLE)        // Single battery mode
                {
                    if(battO->lostCnt == ATTEMPT_TIMES)
                    {
                        PRINTLOG("\r\n [ERR]  %s Connect lost",battO->name);
                        if(battO == &battA) sysBattery|=ERR_BATTA;
                        if(battO == &battB) sysBattery|=ERR_BATTB;
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
                if((battMode == BATT_MODE_DUAL)&&(!battPwrOff))
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
                            battPwrOff = 1;
                            Mavlink_SendLog(MSG_BATTERY, "Start Auto Power Off Process");
                        }
                    }
                }
                break;
            #endif
                
            // Re-connect battery    
            case 0x05:
                if(!battReinit)
                {
                    Batt_Measure(&battA, BATT_MEAS_FET);
                    Batt_Measure(&battB, BATT_MEAS_FET);
                    if(battMode == BATT_MODE_NONE)
                    {
                        #ifdef SINGLE_BATTERY
                        if(((battA.status&BATT_ONBOARD)||(battB.status&BATT_ONBOARD)))
                        {
                            if(battA.status&BATT_ONBOARD)   battO = &battA;
                            if(battB.status&BATT_ONBOARD)   battO = &battB;
                            PRINTLOG("\r\n [INFO] %s Re-connect",battO->name);
                            battReinit = 1;
                            HAL_TIM_Base_Stop_IT(&htim7);
                            Mavlink_SendLog(0x10, "Reinit battery");
                        }
                        #endif
                    }
                    else if(battMode == BATT_MODE_DUAL_ONE)
                    {
                        if(((battA.status&BATT_ONBOARD)&&(battB.status&BATT_ONBOARD)))
                        {
                            PRINTLOG("\r\n [INFO] All Battery Re-connect");
                            battReinit = 1;
                            HAL_TIM_Base_Stop_IT(&htim7);
                            Mavlink_SendLog(0x10, "Reinit battery");
                        }
                    }
                }      
                break;
                
            default:break;
        }
    }		
    return NO_ERR;
}

uint8_t Batt_PowerOff(void)
{
    static uint32_t timeTick = 0;
    static uint8_t stage = 0;
    static uint8_t attemptTimes = 0;
    
    switch(stage)
    {
        case BATT_PWROFF_CHECK:
            Batt_Measure(&battA, BATT_MEAS_FET);
            Batt_Measure(&battB, BATT_MEAS_FET);
            PRINTLOG("\r\n [ACT]  Power Off   #%d",attemptTimes);	                
            if(!((battA.fet&PWR_ON)||(battB.fet&PWR_ON)))
            {
                PRINTLOG("\r\n [INFO] Batt: Auto Power Off Success");
                battA.status &= ~BATT_INUSE;
                battB.status &= ~BATT_INUSE;
                battMode = BATT_MODE_NONE;
                battPwrOff = 0;
                attemptTimes = 0;
                return NO_ERR;
            }
            else
            {
                if(battA.fet&PWR_ON)    Batt_WriteWord(battA.id, BATT_PowerControl, BATT_POWEROFF);
                if(battB.fet&PWR_ON)    Batt_WriteWord(battB.id, BATT_PowerControl, BATT_POWEROFF);
                attemptTimes++;
                timeTick = HAL_GetTick();
                stage = BATT_PWROFF_WAIT;
            }
            
        case BATT_PWROFF_WAIT:
            if(attemptTimes>ATTEMPT_TIMES)
            {
                PRINTLOG("\r\n [ERR]  Batt: Auto Power Off Fail");
                if(battA.fet&PWR_ON)    sysBattery|=ERR_BATTA;
                if(battB.fet&PWR_ON)    sysBattery|=ERR_BATTB;
                attemptTimes = 0;
                battPwrOff = 2;
                return ERR_BATT_POWEROFF;
            }
            if(HAL_GetTick() - timeTick < PWROFF_DELAY)
            {
                stage = BATT_PWROFF_WAIT;   return BATT_WAITING;
            }
            else
            {
                stage = BATT_PWROFF_CHECK;   return BATT_WAITING;
            }
            
        default:
            stage = BATT_PWROFF_CHECK;
            return ERR_BATT_POWEROFF;
    }
}


void Battery_MavlinkPack(mavlink_battery_status_t* mav,uint8_t num)
{
    if(num == BATT_MODE_DUAL)
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
