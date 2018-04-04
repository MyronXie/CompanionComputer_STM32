/**
  ******************************************************************************
  * File Name       : BattMgmt.c
  * Description     : Battery Management Drivers (through I2C)
  *
  * Version         : v0.3.1
  * Created Date    : 2017.09.25
  * Revised Date    : 2018.04.04
  *
  * Author          : Mingye Xie
  ******************************************************************************
  */

#include "BattMgmt.h"

BattMsg battA={"battA",0x16,0x03,0,0x34,2340,24828,-30,94,13140,13980,16000};   // Test data, will replaced by default data
BattMsg battB={"battB",0x26,0x03,0,0x34,2350,24893,-31,95,13680,14520,16000};   // in release version
BattMsg *battX=NULL;                            // Used for select specific battery
BattMsg *battO=NULL,*battQ=NULL;                // Used for single battery mode
uint8_t battMode = BATT_NONE;                   // Battery Mode (SINGLE/DUAL)
uint8_t battPwrOff = 0;                         // Flag for Power Off Function
uint8_t battInit = 1;                           // Flag for Batt Init Function

mavlink_battery_status_t mavBattTx;

extern TIM_HandleTypeDef htim7;

void Battery_Init(void)
{
    static uint32_t timeTick = 0;               // Record delay time
    static uint8_t stage = BATT_INIT_BEGIN;     // Record running stage
    static uint8_t atmpTimes = 0;               // Record attempt times
    
    MsgType battMsg = {0,0};

    switch(stage)                               // Finite State Machine
    {
        case BATT_INIT_BEGIN:
            #ifdef SINGLE_BATTERY
            PRINTLOG(" <SINGLE_BATTERY>");
            #else  //DUAL_BATTERY
            PRINTLOG(" <DUAL_BATTERY>");
            #endif
            stage = BATT_CNCT_CHECK, atmpTimes = 0;
            break;

        case BATT_CNCT_CHECK:
            // Connect to battery
            Batt_Measure(&battA, BATT_MEAS_FET);
            Batt_Measure(&battB, BATT_MEAS_FET);
            PRINTLOG("\r\n [ACT]  Connecting #%d: A-0x%02X, B-0x%02X", atmpTimes, battA.fet, battB.fet);

            // Check onboard status
            #ifdef SINGLE_BATTERY
            if((battA.status&BATT_ONBOARD)||(battB.status&BATT_ONBOARD))        // At least one battery is onboard
            {
                battMode = BATT_SINGLE;
                battO = (battA.status&BATT_ONBOARD)?&battA:&battB;              // Select specific battery
                stage = BATT_PWRON_CHECK, atmpTimes = 0;
            }
            #else  //DUAL_BATTERY
            if((battA.status&BATT_ONBOARD)&&(battB.status&BATT_ONBOARD))        // Both batteries should onboard
            {
                battMode = BATT_DUAL;
                stage = BATT_VDIFF_CHECK;
            }
            #endif
            else
            {
                atmpTimes++;
                timeTick = HAL_GetTick();
                stage = BATT_CNCT_WAIT;
            }
            break;

        case BATT_CNCT_WAIT:
            // Waiting for delay
            if(HAL_GetTick() - timeTick >= CNCT_DELAY)   stage = BATT_CNCT_CHECK; // Continue waiting
            
            if(atmpTimes >= CNCT_ATTEMPT)
            {
                PRINTLOG("\r\n [ERR]  %s %s Offboard",(!(battA.status&BATT_ONBOARD))?battA.name:"",(!(battB.status&BATT_ONBOARD))?battB.name:"");

                if((battA.status&BATT_ONBOARD)||(battB.status&BATT_ONBOARD))    // If one battery is onboard
                {
                    battMode = BATT_DUAL_ONLYONE;
                    battO = (battA.status&BATT_ONBOARD)?&battA:&battB;          // Select specific battery
                }
                
                stage = BATT_INIT_BEGIN;
                
                battMsg.cmd     = ERR_BATT_OFFBOARD;
                battMsg.param   = (!(battA.status&BATT_ONBOARD))&ERR_BATTA + (!(battB.status&BATT_ONBOARD))&ERR_BATTB;
            }
            break;

        case BATT_VDIFF_CHECK:
            #ifndef INGORE_VDIFF
            if(battMode == BATT_DUAL)
            {
                // Read voltage
                Batt_Measure(&battA, BATT_MEAS_VOLT);
                Batt_Measure(&battB, BATT_MEAS_VOLT);

                // Check Voltage difference
                if(((battA.voltage>battB.voltage)?(battA.voltage-battB.voltage):(battB.voltage-battA.voltage)) >= VDIFF_INIT_TOL)
                {
                    PRINTLOG("\r\n [ERR]  Voltage mismatch: A-%dmV, B-%dmV",battA.voltage,battB.voltage);
                    battMode = BATT_DUAL_VDIFF;
                    battO = (battA.fet&PWR_ON)?&battA:&battB;                   // Select specific battery

                    stage = BATT_INIT_BEGIN;
                    
                    battMsg.cmd     = ERR_BATT_VDIFF;
                    battMsg.param   = (battA.voltage<battB.voltage)?ERR_BATTA:ERR_BATTB;
                    break;
                }
            }
            #endif  //INGORE_VDIFF
            stage = BATT_PWRON_CHECK, atmpTimes = 0;
            break;

        case BATT_PWRON_CHECK:
            // Power ON Process
            if(battMode == BATT_SINGLE)            // Single battery mode
            {
                Batt_Measure(battO, BATT_MEAS_FET);
                PRINTLOG("\r\n [ACT]  Power On   #%d: %s-0x%02X", atmpTimes, battO->name, battO->fet);
                if(battO->fet&PWR_ON)
                {
                    stage = BATT_ENFET_CHECK, atmpTimes = 0;
                }
                else
                {
                    Batt_WriteWord(battO->id, BATT_PowerControl, BATT_POWERON);     // Attempt power on
                    atmpTimes++;
                    timeTick = HAL_GetTick();
                    stage = BATT_PWRON_WAIT;
                }
            }
            else if(battMode == BATT_DUAL)          // Dual battery mode
            {
                Batt_Measure(&battA, BATT_MEAS_FET);
                Batt_Measure(&battB, BATT_MEAS_FET);
                PRINTLOG("\r\n [ACT]  Power On   #%d: A-0x%02X, B-0x%02X",atmpTimes,battA.fet,battB.fet);
                if((battA.fet&PWR_ON)&&(battB.fet&PWR_ON))
                {
                    stage = BATT_ENFET_CHECK, atmpTimes = 0;
                }
                else
                {
                    if(!(battA.fet&PWR_ON)) Batt_WriteWord(battA.id, BATT_PowerControl, BATT_POWERON);
                    if(!(battB.fet&PWR_ON)) Batt_WriteWord(battB.id, BATT_PowerControl, BATT_POWERON);
                    atmpTimes++;
                    timeTick = HAL_GetTick();
                    stage = BATT_PWRON_WAIT;
                }
            }
            break;

        case BATT_PWRON_WAIT:
            // Waiting for delay
            if(HAL_GetTick() - timeTick >= PWRON_DELAY)  stage = BATT_PWRON_CHECK;

            if(atmpTimes > PWRON_ATTEMPT)
            {
                if(battMode == BATT_SINGLE)            // Single battery mode
                {
                    PRINTLOG("\r\n [ERR]  %s Power On Fail",battO->name);
                    battMsg.param = (battO == &battA)?ERR_BATTA:ERR_BATTB;
                }
                else if(battMode == BATT_DUAL)         // Dual battery mode
                {
                    PRINTLOG("\r\n [ERR]  %s %s Power On Fail",(!(battA.fet&PWR_ON))?"battA":"",(!(battB.fet&PWR_ON))?"battB":"");
                    battMsg.param   = (!(battA.fet&PWR_ON))&ERR_BATTA + (!(battB.fet&PWR_ON))&ERR_BATTB;
                }
                battMsg.cmd = ERR_BATT_POWERON;
                stage = BATT_INIT_BEGIN;
            }
            break;

        case BATT_ENFET_CHECK:
            // Enable FET process
            if(battMode == BATT_SINGLE)                // Single battery mode
            {
                Batt_Measure(battO, BATT_MEAS_FET);
                PRINTLOG("\r\n [ACT]  Enable FET #%d: %s-0x%02X",atmpTimes,battO->name,battO->fet);
                if(battO->fet&FET_LOCK)                         stage = BATT_INIT_CHECK;
                else
                {
                    Batt_WriteWord(battO->id, BATT_PowerControl, BATT_ENABLEFET);
                    atmpTimes++;
                    timeTick = HAL_GetTick();
                    stage = BATT_ENFET_WAIT;
                }
            }
            else if(battMode == BATT_DUAL)             // Dual battery mode
            {
                Batt_Measure(&battA, BATT_MEAS_FET);
                Batt_Measure(&battB, BATT_MEAS_FET);
                PRINTLOG("\r\n [ACT]  Enable FET #%d: A-0x%02X, B-0x%02X",atmpTimes,battA.fet,battB.fet);
                if((battA.fet&FET_LOCK)&&(battB.fet&FET_LOCK))  stage = BATT_INIT_CHECK;
                else
                {
                    if(!(battA.fet&FET_LOCK)) Batt_WriteWord(battA.id, BATT_PowerControl, BATT_ENABLEFET);
                    if(!(battB.fet&FET_LOCK)) Batt_WriteWord(battB.id, BATT_PowerControl, BATT_ENABLEFET);
                    atmpTimes++;
                    timeTick = HAL_GetTick();
                    stage = BATT_ENFET_WAIT;
                }
            }
            break;

        case BATT_ENFET_WAIT:
            // Waiting for delay
            if(HAL_GetTick() - timeTick >= ENFET_DELAY)  stage = BATT_ENFET_CHECK;
        
            if(atmpTimes > ENFET_ATTEMPT)
            {
                stage = BATT_INIT_BEGIN;
                
                battMsg.cmd = ERR_BATT_ENABLEFET;
                if(battMode == BATT_SINGLE)            // Single battery mode
                {
                    PRINTLOG("\r\n [ERR]  %s Enable FET Fail",battO->name);
                    battMsg.param   = (battO == &battA)?ERR_BATTA:ERR_BATTB;
                }
                else if(battMode == BATT_DUAL)         // Dual battery mode
                {
                    PRINTLOG("\r\n [ERR]  %s %s Enable FET Fail",(!(battA.fet&PWR_ON))?"battA":"",(!(battB.fet&PWR_ON))?"battB":"");
                    battMsg.param   = (!(battA.fet&FET_EN))&ERR_BATTA + (!(battB.fet&FET_EN))&ERR_BATTB;
                }
            }
            break;

        case BATT_INIT_CHECK:
            // Check battery init status
            if(battMode == BATT_SINGLE)                // Single battery mode
            {
                Batt_Measure(battO, BATT_MEAS_FET);
                if((battO->status&BATT_ONBOARD)&&(battO->fet&PWR_ON)&&(battO->fet&FET_LOCK))   battO->status |= BATT_INUSE;

                if(!(battO->status&BATT_INUSE))
                {
                    PRINTLOG("\r\n [ERR]  %s Battery Init Error",battO->name);
                    stage = BATT_INIT_BEGIN;
                    battMsg.cmd = ERR_BATT_INIT;
                    break;
                }
            }
            else if(battMode == BATT_DUAL)             // Dual battery mode
            {
                Batt_Measure(&battA, BATT_MEAS_FET);
                Batt_Measure(&battB, BATT_MEAS_FET);
                if((battA.status&BATT_ONBOARD)&&(battA.fet&PWR_ON)&&(battA.fet&FET_LOCK))   battA.status |= BATT_INUSE;
                if((battB.status&BATT_ONBOARD)&&(battB.fet&PWR_ON)&&(battB.fet&FET_LOCK))   battB.status |= BATT_INUSE;

                if(!((battA.status&BATT_INUSE)&&(battB.status&BATT_INUSE)))
                {
                    PRINTLOG("\r\n [ERR]  %s %s Battery Init Error",(!(battA.status&BATT_INUSE))?"battA":"",(!(battB.status&BATT_INUSE))?"battB":"");
                    stage = BATT_INIT_BEGIN;
                    battMsg.cmd     = ERR_BATT_INIT;
                    battMsg.param   = (!(battA.status&BATT_INUSE))&ERR_BATTA + (!(battB.status&BATT_INUSE))&ERR_BATTB;
                    break;
                }
            }

            PRINTLOG("\r\n [INFO] Battery Init Success");
            stage = BATT_INIT_BEGIN;
            battMsg.cmd = MSG_BATT_INIT;
            break;

        default:
            stage = BATT_INIT_BEGIN;
            break;
    }
    
    if(battMsg.cmd)
    {
        ReportMessage(battMsg);
        battInit = 0;
    }
}


void Batt_Measure(BattMsg* batt, uint8_t cmd)
{
    uint8_t regSta = 0;
    uint16_t regVal;
//    uint8_t regTemp[4];

    #ifndef WITHOUT_BATTERY
    switch(cmd)
    {
        case BATT_MEAS_FET:
            regSta = Batt_ReadWord(batt->id, BATT_FETStatus, &regVal);
            if(!regSta) batt->fet = regVal;
            else        batt->fet = BATT_DUMMY;          // dummy flag, display in boot up
            break;

        case BATT_MEAS_VOLT:
            regSta = Batt_ReadWord(batt->id, BATT_Voltage, &regVal);
            if(!regSta) batt->voltage = regVal; break;

        case BATT_MEAS_TEMP:
            regSta = Batt_ReadWord(batt->id, BATT_Temperature, &regVal);
            if(!regSta) batt->temperature = (regVal-2731)*10;	break;  // Kelvin -> Celsius

        case BATT_MEAS_CURR:
            regSta = Batt_ReadWord(batt->id, BATT_Current, &regVal);
            if(!regSta) batt->current = regVal; break;

        case BATT_MEAS_SOC:
            regSta += Batt_ReadWord(batt->id, BATT_RelativeSOC, &regVal);
            if(!regSta) batt->soc = regVal; break;

        case BATT_MEAS_RCAP:
            regSta += Batt_ReadWord(batt->id, BATT_RemainingCapacity, &regVal);
            if(!regSta) batt->remainingCapacity = regVal*10; break;

        case BATT_MEAS_FCCAP:
            regSta += Batt_ReadWord(batt->id, BATT_FullChargeCapacity, &regVal);
            if(!regSta) batt->fullChargeCapacity = regVal*10; break;

        case BATT_MEAS_DCAP:
            regSta += Batt_ReadWord(batt->id, BATT_DesignCapacity, &regVal);
            if(!regSta) batt->designCapacity = regVal*10; break;

//        case BATT_MEAS_SAFESTA:
//            regSta += Batt_ReadBlock(batt->id, 0x50, regTemp, 4);
//            if(!regSta) batt->safetyStatus      = (uint32_t)(((regTemp[0])<<24)+((regTemp[1])<<16)+((regTemp[2])<<8)+(regTemp[3])); break;
//
//        case BATT_MEAS_PFSTA:
//            regSta += Batt_ReadBlock(batt->id, 0x52, regTemp, 4);
//            if(!regSta) batt->pfStatus          = (uint32_t)(((regTemp[0])<<24)+((regTemp[1])<<16)+((regTemp[2])<<8)+(regTemp[3])); break;
//
//        case BATT_MEAS_OPSSTA:
//            regSta += Batt_ReadBlock(batt->id, 0x54, regTemp, 4);//BATT_OperationStatus
//            if(!regSta) batt->operationStatus   = (uint32_t)(((regTemp[0])<<24)+((regTemp[1])<<16)+((regTemp[2])<<8)+(regTemp[3])); break;

        default: break;
    }

    if(regSta)  batt->status &= ~BATT_ONBOARD;      // Can't connect to battery
    else        batt->status |=  BATT_ONBOARD;
    
    #else
    //batt->status |=  BATT_ONBOARD;
    batt->remainingCapacity -= rand()%2;
    #endif
}


// battCycleCnt(0x00~0x27)
// Measure : 000X NNNN
//           X=0 battA, X=1 battB
//           N=0~15 type
// Judge   : 0010 0NNN
//           N=0~7 judge type
void Battery_Management(void)
{
    static uint8_t battCycleCnt = BATT_FUNC_CYCLE-1;       // Counter for dispatch command for battmgmt system
    MsgType battMsg = {0,0};
    
    // Increase battCycleCnt
    battCycleCnt = (battCycleCnt+1)%BATT_FUNC_CYCLE;

    /********** Measure & Send Process **********/
    if(!(battCycleCnt&BATT_SYS_JUDGE))
    {
        // Select Battery
        if(!(battCycleCnt&BATT_SYS_BATTB))      battX = &battA;
        else                                    battX = &battB;

        // Measure Process
        if(battX->status&BATT_ONBOARD)  Batt_Measure(battX, battCycleCnt&BATT_SYS_MASK_CMD);

        // Send Process
        switch(battCycleCnt&BATT_SYS_MASK_CMD)
        {
            // print data
            case BATT_MGMT_SEND_LOG:
                if(battX->status&BATT_ONBOARD)
                {
                    PRINTLOG("\r\n [INFO] %s: 0x%02X,0x%02X,%d,%d,%d,%d,%d,%d,%d",
                            battX->name, battX->status, battX->fet, battX->temperature, battX->voltage, battX->current,
                            battX->soc, battX->remainingCapacity, battX->fullChargeCapacity, battX->designCapacity);
//                    PRINTLOG("\r\n [INFO] %s: 0x%02X,0x%02X,%d,%d,%d,%d,%d,%d,%d,0x%08X,0x%08X,0x%08X",
//                            battX->name, battX->status, battX->fet, battX->temperature, battX->voltage, battX->current,
//                            battX->soc, battX->remainingCapacity, battX->fullChargeCapacity, battX->designCapacity,
//                            battX->safetyStatus,battX->pfStatus,battX->operationStatus);
                    Battery_MavlinkPack(&mavBattTx,battX);
                    if(battX == &battA)  // (Mav#147)
                    sendCnt = mavlink_msg_battery_status_pack(1, 1, &mavMsgTx, mavBattTx.id, mavBattTx.battery_function, mavBattTx.type,
                                                                    mavBattTx.temperature, mavBattTx.voltages, mavBattTx.current_battery,
                                                                    mavBattTx.current_consumed, mavBattTx.energy_consumed, mavBattTx.battery_remaining);
                    else // battX == &battB (Mav#601)
                    sendCnt = mavlink_msg_battery_status_2_pack(1, 1, &mavMsgTx, mavBattTx.id, mavBattTx.battery_function, mavBattTx.type,
                                                                    mavBattTx.temperature, mavBattTx.voltages, mavBattTx.current_battery,
                                                                    mavBattTx.current_consumed, mavBattTx.energy_consumed, mavBattTx.battery_remaining);
                    Mavlink_SendMessage(&mavMsgTx, sendCnt);
                }
                break;

             case BATT_MGMT_CNCT_COUNT:
                Batt_Measure(battX, BATT_MEAS_FET);
                if(battX->status&BATT_INUSE)
                {
                    if(battX->status&BATT_ONBOARD)
                    {
                        if(battX->lostCnt == CNCT_ATTEMPT)  battMsg.cmd = MSG_BATT_ONBOARD;
                        battX->lostCnt = 0;
                    }
                    else
                    {
                        if(++battX->lostCnt < CNCT_ATTEMPT)   PRINTLOG("\r\n [INFO] %s Lost #%d!", battX->name, battX->lostCnt);
                    }
                }
                break;

            default: break;
        }

    }

    /********** Judge Process **********/
    else
    {
        switch(battCycleCnt&BATT_SYS_MASK_CMD)
        {
            // Battery Link Lost
            case BATT_MGMT_CNCT_LOST:
                // =============== DEBUG =============== 
                if(sysTicks == 8) battA.status&=~BATT_ONBOARD;
                if(sysTicks == 12) battB.status&=~BATT_ONBOARD;
                // =============== DEBUG =============== 
                if(battMode == BATT_SINGLE)        // Single battery mode
                {
                    if(battO->lostCnt == CNCT_ATTEMPT)
                    {
                        PRINTLOG("\r\n [ERR]  %s Connect lost",battO->name);
                        battMode = BATT_NONE;
                        battMsg.cmd     = ERR_BATT_OFFBOARD;
                        battMsg.param   = (battO == &battA)?ERR_BATTA:ERR_BATTB;
                    }
                }
                else if(battMode == BATT_DUAL)     // Dual battery mode
                {
                    if((battA.lostCnt == CNCT_ATTEMPT)||(battB.lostCnt == CNCT_ATTEMPT))
                    {
                        PRINTLOG("\r\n [ERR]  %s %s Connect lost",(battA.lostCnt>=CNCT_ATTEMPT)?battA.name:"",(battB.lostCnt>=CNCT_ATTEMPT)?battB.name:"");
                        battMsg.cmd     = ERR_BATT_OFFBOARD;
                        battMsg.param   = (battA.lostCnt >= CNCT_ATTEMPT)&ERR_BATTA + (battB.lostCnt >= CNCT_ATTEMPT)&ERR_BATTB;
                    }
                }
                break;

            // Power off battery when Vdiff is too high
            case BATT_MGMT_VDIFF_CHECK:
                if(battMode == BATT_DUAL)
                {
                    if(((battA.voltage>battB.voltage)?(battA.voltage-battB.voltage):(battB.voltage-battA.voltage)) >= VDIFF_RUNNING_TOL)
                    {
                        PRINTLOG("\r\n [ERR]  Voltage mismatch: A-%dmV, B-%dmV",battA.voltage,battB.voltage);
                        battMode = BATT_DUAL_VDIFF;
                        battPwrOff = 1;
                        battMsg.cmd = ERR_BATT_OFFBOARD;
                        if(battA.voltage < battB.voltage)             // Select specific battery
                        {
                            battMsg.param|=ERR_BATTA;
                            battO = &battA;
                            battQ = &battB;
                        }
                        else
                        {
                            battMsg.param|=ERR_BATTB;
                            battO = &battB;
                            battQ = &battA;
                        }
                    }
                }
                else if(battMode == BATT_DUAL_VDIFF)
                {
                    if((!battPwrOff)&&(battQ!=NULL))
                    {
                        battO = battQ;
                        battQ = NULL;
                    }
                }
                break;

            #ifndef INGORE_POWEROFF
            // Judge Auto power off
            case BATT_MGMT_PWROFF:
                if((battMode == BATT_DUAL)&&(!battPwrOff))
                {
                    // Prevent the situation that one battery lost connect
                    if((battA.status&BATT_ONBOARD)&&(battB.status&BATT_ONBOARD))
                    {
                        // One battery is powered down
                        if(((battA.fet&PWR_ON)&&(!(battB.fet&PWR_ON)))||((battB.fet&PWR_ON)&&(!(battA.fet&PWR_ON))))
                        {
                            if(sysArmed)   // Ensure can/can not power off battery
                            {
                                PRINTLOG("\r\n [ERR]  %s %s Lost power in the air",(!(battA.fet&PWR_ON))?"battA":"",(!(battB.fet&PWR_ON))?"battB":"");
                                battMsg.cmd     = ERR_BATT_LOSTPWR;
                                battMsg.param   = (!(battA.fet&PWR_ON))&ERR_BATTA + (!(battB.fet&PWR_ON))&ERR_BATTB;
                            }
                            else
                            {
                                battPwrOff = 1;
                                battMsg.cmd = MSG_BATT_PWROFF_START;
                            }
                        }
                    }
                }
                break;
            #endif

            // Re-connect battery
            case BATT_MGMT_RECNCT:
                #ifdef ENABLE_BATT_REINIT
                if(!battInit)
                {
                    Batt_Measure(&battA, BATT_MEAS_FET);
                    Batt_Measure(&battB, BATT_MEAS_FET);
                    if(battMode == BATT_NONE)
                    {
                        #ifdef SINGLE_BATTERY
                        if(((battA.status&BATT_ONBOARD)||(battB.status&BATT_ONBOARD)))
                        {
                            if(battA.status&BATT_ONBOARD)   battO = &battA;
                            if(battB.status&BATT_ONBOARD)   battO = &battB;
                            PRINTLOG("\r\n [INFO] %s Re-connect",battO->name);
                            battInit = 1;
                            battMsg.cmd = MSG_BATT_REINIT;
                        }
                        #endif
                    }
                    else if(battMode == BATT_DUAL_ONLYONE)
                    {
                        if(((battA.status&BATT_ONBOARD)&&(battB.status&BATT_ONBOARD)))
                        {
                            PRINTLOG("\r\n [INFO] All Battery Re-connect");
                            battInit = 1;
                            battMsg.cmd = MSG_BATT_REINIT;
                        }
                    }
                }
                #endif //ENABLE_BATT_REINIT
                break;

            // Check power after pwer off process
            case BATT_MGMT_PWRCHECK:
                if(battMode == BATT_NONE)
                {
                    // If these code can be reached, means power is still on
                    if((!((battA.fet&PWR_ON)||(battA.fet&FET_EN)))&&(!((battB.fet&PWR_ON)||(battB.fet&FET_EN))))
                        battMsg.cmd     = ERR_BATT_STILLPWR;
                        battMsg.param   = (!((battA.fet&PWR_ON)||(battA.fet&FET_EN)))&ERR_BATTA 
                                        + (!((battB.fet&PWR_ON)||(battA.fet&FET_EN)))&ERR_BATTB;
                }
                break;

            default:    break;
        }
    }

    if(battMsg.cmd)     ReportMessage(battMsg);
}

void Batt_PowerOff(void)
{
    static uint32_t timeTick = 0;
    static uint8_t stage = BATT_PWROFF_CHECK;
    static uint8_t atmpTimes = 0;

    MsgType battMsg = {0,0};
    
    switch(stage)
    {
        case BATT_PWROFF_CHECK:
            Batt_Measure(&battA, BATT_MEAS_FET);
            Batt_Measure(&battB, BATT_MEAS_FET);
            PRINTLOG("\r\n [ACT]  Power Off   #%d: A-0x%02X, B-0x%02X",atmpTimes,battA.fet,battB.fet);

            // Close single battery
            if(battMode == BATT_SINGLE||battMode == BATT_DUAL_ONLYONE||battMode == BATT_DUAL_VDIFF)
            {
                if(!(battO->fet&PWR_ON))
                {
                    PRINTLOG("\r\n [INFO] Batt: Power Off Success");
                    battO->status &= ~BATT_INUSE;
                    battMode = BATT_NONE;
                    battPwrOff = 0;
                    stage = BATT_PWROFF_CHECK, atmpTimes = 0;
                    battMsg.cmd = MSG_BATT_PWROFF_END;
                }
                else
                {
                    Batt_WriteWord(battO->id, BATT_PowerControl, BATT_POWEROFF);
                    atmpTimes++;
                    timeTick = HAL_GetTick();
                    stage = BATT_PWROFF_WAIT;
                }
            }
            // Close dual battery
            else if(battMode == BATT_DUAL)
            {
                if(!((battA.fet&PWR_ON)||(battB.fet&PWR_ON)))
                {
                    PRINTLOG("\r\n [INFO] Batt: Power Off Success");
                    battA.status &= ~BATT_INUSE;
                    battB.status &= ~BATT_INUSE;
                    battMode = BATT_NONE;
                    battPwrOff = 0;
                    stage = BATT_PWROFF_CHECK, atmpTimes = 0;
                    battMsg.cmd = MSG_BATT_PWROFF_END;  // Actually this message won't send because power down
                }
                else
                {
                    if(battA.fet&PWR_ON)    Batt_WriteWord(battA.id, BATT_PowerControl, BATT_POWEROFF);
                    if(battB.fet&PWR_ON)    Batt_WriteWord(battB.id, BATT_PowerControl, BATT_POWEROFF);
                    atmpTimes++;
                    timeTick = HAL_GetTick();
                    stage = BATT_PWROFF_WAIT;
                }
            }
            else if(battMode == BATT_NONE)        // Single battery mode
            {
                battPwrOff = 0;
                stage = BATT_PWROFF_CHECK, atmpTimes = 0;
                battMsg.cmd = ERR_BATT_POWEROFF;
            }
            break;

        case BATT_PWROFF_WAIT:
            if(HAL_GetTick() - timeTick >= PWROFF_DELAY)     stage = BATT_PWROFF_CHECK;

            if(atmpTimes > PWROFF_ATTEMPT)
            {
                PRINTLOG("\r\n [ERR]  Batt: Power Off Fail");
                stage = BATT_PWROFF_CHECK, atmpTimes = 0;
                battPwrOff = 2;
                battMsg.cmd     = ERR_BATT_POWEROFF;
                battMsg.param   = (battA.fet&PWR_ON)&ERR_BATTA + (battB.fet&PWR_ON)&ERR_BATTB;
            }
            break;

        default:
            stage = BATT_PWROFF_CHECK;
            break;
    }
    
    if(battMsg.cmd)     ReportMessage(battMsg);
}

// Send battery message individually
void Battery_MavlinkPack(mavlink_battery_status_t* mav, BattMsg* batt)
{
    mav->id                 = batt->id;
    mav->battery_function   = MAV_BATTERY_FUNCTION_ALL;
    mav->type               = MAV_BATTERY_TYPE_LIPO;
    mav->temperature        = batt->temperature;
    mav->voltages[0]        = batt->voltage;
    mav->current_battery    = batt->current;
    mav->current_consumed   = batt->fullChargeCapacity - batt->remainingCapacity;
    mav->energy_consumed    = -1;
    mav->battery_remaining  = batt->soc;
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

uint8_t Batt_ReadBlock(uint8_t _addr, uint8_t _reg, uint8_t* _data, uint8_t _num)
{
    return I2C_ReadBlock(_addr, _reg, _data, _num);
}

/******************************END OF FILE******************************/
