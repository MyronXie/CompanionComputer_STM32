/**
  ******************************************************************************
  * File Name       : BattMgmt.c
  * Description     : Battery Management Drivers (through I2C)
  *
  * Version         : v0.3.1
  * Created Date    : 2017.09.25
  * Revised Date    : 2018.04.11
  *
  * Author          : Mingye Xie
  ******************************************************************************
  */

#include "BattMgmt.h"
// <Debug> Test data, will replaced by default data in release version
BattMsg battA={0,"battA",0x16,0x00,0,0x14,2340,24828,-30,94,13140,13980,16000}; 
BattMsg battB={1,"battB",0x26,0x00,0,0x00,2350,24893,-31,95,13680,14520,16000};
BattMsg *battX=NULL;                            // Used for select specific battery
BattMsg *battO=NULL,*battQ=NULL;                // Used for single battery mode
BattModeType battMode = BATT_MODE_NONE;         // Battery Mode (SINGLE/DUAL)
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
            // Display Setting Battery Mode
            #ifdef SINGLE_BATTERY
            PRINTLOG(" <SINGLE_BATTERY>");
            #else  //DUAL_BATTERY
            PRINTLOG(" <DUAL_BATTERY>");
            #endif

            stage = BATT_CNCT_CHECK, atmpTimes = 0;
            break;

        case BATT_CNCT_CHECK:
            // Attepmt Connect to battery
            Batt_Measure(&battA, BATT_MEAS_FET);
            Batt_Measure(&battB, BATT_MEAS_FET);
            PRINTLOG("\r\n [ACT]  Connecting #%d: A-0x%02X, B-0x%02X", atmpTimes, battA.fet, battB.fet);

            // <Debug> Test Battery Init Process
            #ifdef WITHOUT_BATTERY  
            if(atmpTimes == 0)  battA.status |=  BATT_ONBOARD;
            if(atmpTimes == 0)  battB.status |=  BATT_ONBOARD;
            #endif
        
            // Check onboard status
            #ifdef SINGLE_BATTERY
            if((battA.status&BATT_ONBOARD)||(battB.status&BATT_ONBOARD))        // At least one battery is onboard
            {
                battMode = BATT_MODE_SINGLE;
                battO = (battA.status&BATT_ONBOARD)?&battA:&battB;              // Select specific battery
                stage = BATT_PWRON_CHECK, atmpTimes = 0;
            }
            #else  //DUAL_BATTERY
            if((battA.status&BATT_ONBOARD)&&(battB.status&BATT_ONBOARD))        // Both batteries should onboard
            {
                battMode = BATT_MODE_DUAL;
                stage = BATT_VDIFF_CHECK;
            }
            #endif
            else
            {
                // Record current timeTick, step into delay process
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
                if((battA.status&BATT_ONBOARD)||(battB.status&BATT_ONBOARD))    // If one battery is onboard
                {
                    battMode = BATT_MODE_DUAL_ONLY1;
                    battO = (battA.status&BATT_ONBOARD)?&battA:&battB;          // Select specific battery
                }
                else
                {
                    battMode = BATT_MODE_NONE;
                    battO = NULL;
                }

                battMsg.cmd     = ERR_BATT_OFFBOARD;
                battMsg.param   = Batt_Judge(BATT_MODE_DUAL, BATT_JUDGE_ONBOARD);   // Specific
                PRINTLOG("\r\n [ERR]  %sOffboard",paramList[battMsg.param]);

                stage = BATT_INIT_BEGIN;
                battInit = 0;
            }            
            break;

        case BATT_VDIFF_CHECK:
            #ifndef INGORE_VDIFF
            if(battMode == BATT_MODE_DUAL)
            {
                // Read voltage
                Batt_Measure(&battA, BATT_MEAS_VOLT);
                Batt_Measure(&battB, BATT_MEAS_VOLT);

                // Check Voltage difference
                if(Batt_Judge(battMode, BATT_JUDGE_VDIFF_INIT))
                {
                    battO = (battA.fet&PWR_ON)?&battA:&battB;                   // Select specific battery

                    battMsg.cmd     = ERR_BATT_VDIFF;
                    battMsg.param   = Batt_Judge(battMode, BATT_JUDGE_VDIFF_INIT);
                    PRINTLOG("\r\n [ERR]  Voltage mismatch: A-%dmV, B-%dmV",battA.voltage,battB.voltage);

                    battMode = BATT_MODE_DUAL_VDIFF;
                    stage = BATT_INIT_BEGIN;
                    battInit = 0;
                    break;
                }
            }
            #endif  //INGORE_VDIFF
            stage = BATT_PWRON_CHECK, atmpTimes = 0;
            break;

        case BATT_PWRON_CHECK:
            // <Debug> Test Battery Init Process    
            #ifdef WITHOUT_BATTERY
            if(atmpTimes == 0)  if(battA.fet<0x25)  battA.fet = 0x25;
            if(atmpTimes == 1)  if(battB.fet<0x25)  battB.fet = 0x25;
            #endif
        
            // Read battery fet
            Batt_Measure(&battA, BATT_MEAS_FET);
            Batt_Measure(&battB, BATT_MEAS_FET);
            PRINTLOG("\r\n [ACT]  Power On   #%d: A-0x%02X, B-0x%02X",atmpTimes,battA.fet,battB.fet);

            if(Batt_Judge(battMode, BATT_JUDGE_PWRON))
            {
                // Attempt power on
                if(!(battA.fet&PWR_ON)) Batt_WriteWord(battA.id, BATT_PowerControl, BATT_POWERON);
                if(!(battB.fet&PWR_ON)) Batt_WriteWord(battB.id, BATT_PowerControl, BATT_POWERON);

                // Record current timeTick, step into delay process
                atmpTimes++;
                timeTick = HAL_GetTick();
                stage = BATT_PWRON_WAIT;                
            }
            else
            {
                stage = BATT_ENFET_CHECK, atmpTimes = 0;
            }
            break;

        case BATT_PWRON_WAIT:
            // Waiting for delay
            if(HAL_GetTick() - timeTick >= PWRON_DELAY)  stage = BATT_PWRON_CHECK;

            if(atmpTimes > PWRON_ATTEMPT)
            {
                battMsg.cmd     = ERR_BATT_POWERON;
                battMsg.param   = Batt_Judge(battMode, BATT_JUDGE_PWRON);
                PRINTLOG("\r\n [ERR]  %sPower On Fail",paramList[battMsg.param]);

                battMode = BATT_MODE_DUAL_ONLY1;
                battInit = 2;
                stage = BATT_INIT_BEGIN;
            }
            break;

        case BATT_ENFET_CHECK:   
            // <Debug> Test Battery Init Process    
            #ifdef WITHOUT_BATTERY
            if(atmpTimes == 0) if(battA.fet<0x34) battA.fet = 0x34;
            if(atmpTimes == 1) if(battB.fet<0x34) battB.fet = 0x34;
            #endif

            // Read battery fet
            Batt_Measure(&battA, BATT_MEAS_FET);
            Batt_Measure(&battB, BATT_MEAS_FET);
            PRINTLOG("\r\n [ACT]  Enable FET #%d: A-0x%02X, B-0x%02X",atmpTimes,battA.fet,battB.fet);

            if(Batt_Judge(battMode, BATT_JUDGE_FETEN))
            {
                // Attempt enable fet
                if(!(battA.fet&FET_LOCK)) Batt_WriteWord(battA.id, BATT_PowerControl, BATT_ENABLEFET);
                if(!(battB.fet&FET_LOCK)) Batt_WriteWord(battB.id, BATT_PowerControl, BATT_ENABLEFET);

                // Record current timeTick, step into delay process
                atmpTimes++;
                timeTick = HAL_GetTick();
                stage = BATT_ENFET_WAIT;
            }
            else
            {
                stage = BATT_INIT_CHECK, atmpTimes = 0;
            }
            break;

        case BATT_ENFET_WAIT:
            // Waiting for delay
            if(HAL_GetTick() - timeTick >= ENFET_DELAY)  stage = BATT_ENFET_CHECK;
        
            if(atmpTimes > ENFET_ATTEMPT)
            {
                battMsg.cmd     = ERR_BATT_ENABLEFET;
                battMsg.param   = Batt_Judge(battMode, BATT_JUDGE_FETEN);
                PRINTLOG("\r\n [ERR]  %sEnable FET Fail",paramList[battMsg.param]);

                battMode = BATT_MODE_DUAL_ONLY1;
                battInit = 2;
                stage = BATT_INIT_BEGIN;
            }
            
            break;

        case BATT_INIT_CHECK:
            // Read battery fet
            Batt_Measure(&battA, BATT_MEAS_FET);
            Batt_Measure(&battB, BATT_MEAS_FET);
            // Set INUSE flag
            if((battA.status&BATT_ONBOARD)&&(battA.fet&PWR_ON)&&(battA.fet&FET_LOCK))   battA.status |= BATT_INUSE;
            if((battB.status&BATT_ONBOARD)&&(battB.fet&PWR_ON)&&(battB.fet&FET_LOCK))   battB.status |= BATT_INUSE;

            if(Batt_Judge(battMode, BATT_JUDGE_INUSE))
            {
                battMsg.cmd     = ERR_BATT_INIT;
                battMsg.param   = Batt_Judge(battMode, BATT_JUDGE_INUSE);
                PRINTLOG("\r\n [ERR]  %sBattery Init Error",paramList[battMsg.param]); 
                battInit = 2;
            }
            else
            {
                battMsg.cmd     = MSG_BATT_INIT;
                PRINTLOG("\r\n [INFO] Battery Init Success");
                battInit = 0;
            }
            stage = BATT_INIT_BEGIN;
            break;

        default:
            stage = BATT_INIT_BEGIN;
            break;
    }
    
    if(battMsg.cmd) ReportMessage(battMsg);
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
    static MsgType battMsgLst = {0,0};
    static uint8_t reportTimes = 0;
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
                        if(battX->lostCnt == CNCT_ATTEMPT)
                            battMsg.cmd = MSG_BATT_ONBOARD;     // Avoid frequent report
                        battX->lostCnt = 0;
                    }
                    else
                    {
                        if(++battX->lostCnt < CNCT_ATTEMPT)
                            PRINTLOG("\r\n [INFO] %s Lost #%d!", battX->name, battX->lostCnt);
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
                if(battMode == BATT_MODE_DUAL||battMode == BATT_MODE_SINGLE)
                {
                    if(Batt_Judge(battMode, BATT_JUDGE_OFFBOARD))
                    {
                        battMsg.cmd     = ERR_BATT_OFFBOARD;
                        battMsg.param   = Batt_Judge(battMode, BATT_JUDGE_OFFBOARD);
                        PRINTLOG("\r\n [ERR]  %sConnect lost",paramList[battMsg.param]);
                        if(battMode == BATT_MODE_SINGLE)    battMode = BATT_MODE_NONE;
                        if(battMode == BATT_MODE_DUAL)      battMode = BATT_MODE_DUAL_ONLY1;
                    }
                }

                break;

            // Power off battery when Vdiff is too high
            case BATT_MGMT_VDIFF_CHECK:
                if(battMode == BATT_MODE_DUAL)
                {
                    if(Batt_Judge(battMode, BATT_JUDGE_VDIFF_RUN))
                    {
                        battMsg.cmd     = ERR_BATT_VDIFF;
                        battMsg.param   = Batt_Judge(battMode, BATT_JUDGE_VDIFF_RUN);

                        PRINTLOG("\r\n [ERR]  Voltage mismatch: A-%dmV, B-%dmV",battA.voltage,battB.voltage);

                        // if(battMsg.param == battA.index)           // Select specific battery
                        // {
                        //     battO = &battA;     battQ = &battB;
                        // }
                        // else
                        // {
                        //     battO = &battB;     battQ = &battA;
                        // }

                        // battPwrOff = 1;
                    }
                }
                else if(battMode == BATT_MODE_DUAL_VDIFF)
                {
                    // if((!battPwrOff)&&(battQ!=NULL))                // Change inactive battery to active
                    // {
                    //     battO = battQ;
                    //     battQ = NULL;
                    // }
                }
                break;

            #ifndef INGORE_POWEROFF
            // Judge Auto power off
            case BATT_MGMT_PWROFF:
                if((battMode == BATT_MODE_DUAL)&&(!battPwrOff))
                {
                    // Prevent the situation that one battery lost connect
                    if(!Batt_Judge(battMode, BATT_JUDGE_ONBOARD))
                    {
                        // One battery is powered down
                        if(((battA.fet&PWR_ON)&&(!(battB.fet&PWR_ON)))||((battB.fet&PWR_ON)&&(!(battA.fet&PWR_ON))))
                        {
                            if(sysArmed)   // Ensure can/can not power off battery
                            {
                                // These code can be reached when battery lost power in armed status, should not power off the other battery
                                battMsg.cmd     = ERR_BATT_LOSTPWR;
                                battMsg.param   = Batt_Judge(battMode, BATT_JUDGE_PWRON);
                                PRINTLOG("\r\n [ERR]  %sLost power in the air",paramList[battMsg.param]);
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
                    if(battMode == BATT_MODE_NONE)
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
                    else if(battMode == BATT_MODE_DUAL_ONLY1)
                    {
                        //<Dev> Need to re-config when can re-init
                        if(((battA.status&BATT_ONBOARD)&&(battB.status&BATT_ONBOARD)&&(!battInit)))
                        {
                            PRINTLOG("\r\n [INFO] All Battery Re-connect");
                            battInit = 1;
                            battMsg.cmd = MSG_BATT_REINIT;
                        }
                    }
                }
                #endif //ENABLE_BATT_REINIT
                break;


            case BATT_MGMT_PWRCHECK:
                // Check power after pwer off process
                if(battMode == BATT_MODE_NONE)
                {
                    // If these code can be reached, means battery power is still on
                    #ifndef WITHOUT_BATTERY
                    if((!((battA.fet&PWR_ON)||(battA.fet&FET_EN)))&&(!((battB.fet&PWR_ON)||(battB.fet&FET_EN))))
                        battMsg.cmd     = ERR_BATT_STILLPWR;    //ERR_BATT_POWEROFF
                        battMsg.param   = Batt_Judge(battMode, BATT_JUDGE_PWRCHECK);
                    #endif
                }
                else if(battMode == BATT_MODE_DUAL)
                {
                    if(Batt_Judge(battMode, BATT_JUDGE_UNDERVOLT))
                    {
                        battMsg.cmd     = ERR_BATT_UNDERVOLT;
                        battMsg.param   = Batt_Judge(battMode, BATT_JUDGE_UNDERVOLT);
                    }
                    else if(Batt_Judge(battMode, BATT_JUDGE_LOWSOC))
                    {
                        battMsg.cmd     = ERR_BATT_LOWSOC;
                        battMsg.param   = Batt_Judge(battMode, BATT_JUDGE_LOWSOC);
                    }
                    else if(Batt_Judge(battMode, BATT_JUDGE_OVERCURR))
                    {
                        battMsg.cmd     = ERR_BATT_OVERCURR;
                        battMsg.param   = Batt_Judge(battMode, BATT_JUDGE_OVERCURR);
                    }
                    else if(Batt_Judge(battMode, BATT_JUDGE_OVERTEMP))
                    {
                        battMsg.cmd     = ERR_BATT_OVERTEMP;
                        battMsg.param   = Batt_Judge(battMode, BATT_JUDGE_OVERTEMP);
                    }
                    else if(Batt_Judge(battMode, BATT_JUDGE_UNDERTEMP))
                    {
                        battMsg.cmd     = ERR_BATT_UNDERTEMP;
                        battMsg.param   = Batt_Judge(battMode, BATT_JUDGE_UNDERTEMP);
                    }
                    
                    if(battMsgLst.cmd!=battMsg.cmd)
                    {
                        reportTimes = 0;
                        battMsgLst.cmd=battMsg.cmd;
                    }
                    else if((++reportTimes)%3)   battMsg.cmd=0;  // Clear msg to avoid frequent report

                }
                break;
                
            case BATT_MGMT_DEBUG:
                #ifdef WITHOUT_BATTERY
                srand(sysTicks);
                battA.voltage -= rand()%3;
                battB.voltage -= rand()%3;
                battA.remainingCapacity -= rand()%9;
                battB.remainingCapacity -= rand()%9;
                battA.soc = battA.remainingCapacity*100/battA.fullChargeCapacity;
                battB.soc = battB.remainingCapacity*100/battB.fullChargeCapacity;
                #endif            
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
            // <Debug> Test Battery Init Process
            #ifdef WITHOUT_BATTERY  
            if(atmpTimes == 1)  battA.fet = 0x14;
            if(atmpTimes == 1)  battB.fet = 0x14;
            #endif

            Batt_Measure(&battA, BATT_MEAS_FET);
            Batt_Measure(&battB, BATT_MEAS_FET);
            PRINTLOG("\r\n [ACT]  Power Off   #%d: A-0x%02X, B-0x%02X",atmpTimes,battA.fet,battB.fet);

            if(battMode == BATT_MODE_DUAL||battMode == BATT_MODE_SINGLE)
            {
                if(!Batt_Judge(battMode, BATT_JUDGE_PWROFF))
                {
                    PRINTLOG("\r\n [INFO] Batt: Power Off Success");
                    battA.status &= ~BATT_INUSE;
                    battB.status &= ~BATT_INUSE;

                    // Actually this message won't send because power down
                    battMsg.cmd = MSG_BATT_PWROFF_END;
                    battMsg.param = Batt_Judge(battMode, BATT_JUDGE_PWROFF);

                    battMode = BATT_MODE_DUAL_VDIFF;
                    battPwrOff = 0;
                    stage = BATT_PWROFF_CHECK, atmpTimes = 0;
                }
                else
                {
                    // Power off battery
                    if(battA.fet&PWR_ON)    Batt_WriteWord(battA.id, BATT_PowerControl, BATT_POWEROFF);
                    if(battB.fet&PWR_ON)    Batt_WriteWord(battB.id, BATT_PowerControl, BATT_POWEROFF);
                    atmpTimes++;
                    timeTick = HAL_GetTick();
                    stage = BATT_PWROFF_WAIT;
                }
            }
            else if(battMode == BATT_MODE_NONE)        // Single battery mode
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

                battMsg.cmd     = ERR_BATT_POWEROFF;
                battMsg.param   = Batt_Judge(battMode, BATT_JUDGE_PWROFF);

                battPwrOff = 2;
            }
            break;

        default:
            stage = BATT_PWROFF_CHECK;
            break;
    }
    
    if(battMsg.cmd)     ReportMessage(battMsg);
}

// Indicate which battery have specific error, if no error, return zero
uint8_t Batt_Judge(BattModeType mode, BattJudgeType judge)
{
    uint8_t result = 0;

    if(mode==BATT_MODE_SINGLE||mode==BATT_MODE_DUAL_VDIFF)
    {
        switch(judge)
        {
            case BATT_JUDGE_ONBOARD:
                result=((!(battO->status&BATT_ONBOARD))?(1<<(battO->index)):0); break;
        
            case BATT_JUDGE_PWRON:
                result=((!(battO->fet&PWR_ON))?(1<<(battO->index)):0); break;
            
            case BATT_JUDGE_FETEN:
                result=((!(battO->fet&FET_LOCK))?(1<<(battO->index)):0); break;

            case BATT_JUDGE_INUSE:
                result=((!(battO->status&BATT_INUSE))?(1<<(battO->index)):0); break;
            
            case BATT_JUDGE_OFFBOARD:
                result=(((battO->lostCnt>=CNCT_ATTEMPT))?(1<<(battO->index)):0); break;

            default: 
                PRINTLOG("\r\n [DEBUG] Batt_Judge(0x%02X,0x%02X)",mode,judge);
                break;
        }
    }
    else if(mode==BATT_MODE_DUAL)
    {
        switch(judge)
        {
            case BATT_JUDGE_ONBOARD:
                result=((!(battA.status&BATT_ONBOARD))?(1<<INDEX_BATTA):0)
                    +((!(battB.status&BATT_ONBOARD))?(1<<INDEX_BATTB):0); break;

            case BATT_JUDGE_PWRON:
                result=((!(battA.fet&PWR_ON))?(1<<INDEX_BATTA):0)
                    +((!(battB.fet&PWR_ON))?(1<<INDEX_BATTB):0); break;

            case BATT_JUDGE_FETEN:
                result=((!(battA.fet&FET_LOCK))?(1<<INDEX_BATTA):0)
                    +((!(battB.fet&FET_LOCK))?(1<<INDEX_BATTB):0); break;

            case BATT_JUDGE_INUSE:
                result=((!(battA.status&BATT_INUSE))?(1<<INDEX_BATTA):0)
                    +((!(battB.status&BATT_INUSE))?(1<<INDEX_BATTB):0); break;

            case BATT_JUDGE_VDIFF_INIT:
                result=(((battB.voltage-battA.voltage)>=TOL_VDIFF_INIT)?(1<<INDEX_BATTA):0)
                    +(((battA.voltage-battB.voltage)>=TOL_VDIFF_INIT)?(1<<INDEX_BATTB):0); break;

            case BATT_JUDGE_VDIFF_RUN:
                result=(((battB.voltage-battA.voltage)>=TOL_VDIFF_RUN)?(1<<INDEX_BATTA):0)
                    +(((battA.voltage-battB.voltage)>=TOL_VDIFF_RUN)?(1<<INDEX_BATTB):0); break;

            case BATT_JUDGE_OFFBOARD:
                result=((battA.lostCnt>=CNCT_ATTEMPT)?(1<<INDEX_BATTA):0)
                    +((battB.lostCnt>=CNCT_ATTEMPT)?(1<<INDEX_BATTB):0); break;

            case BATT_JUDGE_PWROFF:
                result=((battA.fet&PWR_ON)?(1<<INDEX_BATTA):0)
                    +((battB.fet&PWR_ON)?(1<<INDEX_BATTB):0); break;

            case BATT_JUDGE_PWRCHECK:
                result=((!((battA.fet&PWR_ON)||(battA.fet&FET_EN)))?(1<<INDEX_BATTA):0)
                    +((!((battB.fet&PWR_ON)||(battB.fet&FET_EN)))?(1<<INDEX_BATTB):0); break;
                
            case BATT_JUDGE_UNDERVOLT:
                result=((battA.voltage<=TOL_UNDERVOLT)?(1<<INDEX_BATTA):0)
                    +((battB.voltage<=TOL_UNDERVOLT)?(1<<INDEX_BATTB):0); break;

            case BATT_JUDGE_LOWSOC:
                result=((battA.soc<=TOL_LOWSOC)?(1<<INDEX_BATTA):0)
                    +((battB.soc<=TOL_LOWSOC)?(1<<INDEX_BATTB):0); break;

            case BATT_JUDGE_OVERCURR:
                result=((battA.current<=TOL_OVERCURR)?(1<<INDEX_BATTA):0)
                    +((battB.current<=TOL_OVERCURR)?(1<<INDEX_BATTB):0); break;

            case BATT_JUDGE_OVERTEMP:
                result=((battA.temperature>=TOL_OVERTEMP)?(1<<INDEX_BATTA):0)
                    +((battB.temperature>=TOL_OVERTEMP)?(1<<INDEX_BATTB):0); break;

            case BATT_JUDGE_UNDERTEMP:
                result=((battA.temperature<=TOL_UNDERTEMP)?(1<<INDEX_BATTA):0)
                    +((battB.temperature<=TOL_UNDERTEMP)?(1<<INDEX_BATTB):0); break;
    
            default:
                PRINTLOG("\r\n [DEBUG] Batt_Judge(0x%02Xd,0x%02X)",mode,judge);
                break;
        }
    }

    else 
    {
        PRINTLOG("\r\n [DEBUG] Batt_Judge(0x%02X,0x%02X)",mode,judge);
    }

    return result;
}

void Console_BattMgmt(uint8_t cmd)
{
    switch(cmd)
    { 
        case 'A':   battA.status|=BATT_ONBOARD; break;
        case 'a':   battA.status&=~BATT_ONBOARD; break;
        case 'B':   battB.status|=BATT_ONBOARD; break;
        case 'b':   battB.status&=~BATT_ONBOARD; break;
        case 'X':   battB.fet|=PWR_ON; break;
        case 'x':   battB.fet&=~PWR_ON; break;
        case 'V':   battA.voltage+=2000;    break;
        case 'v':   battA.voltage-=2000;    break;
        case '1':   battA.voltage=20000;battB.voltage=20000;    break;
        case '2':   battA.temperature=4000;break;
        case '3':   battA.temperature=1000;  break;
        case '4':   battA.current=-8001;  break;
        case '5':   battA.remainingCapacity=1000;  break;
    }
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
