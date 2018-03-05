/**
  ******************************************************************************
  * File Name       : System.c
  * Description     : System param for CC_STM32
  *
  * Version         : v0.3
  * Created Date    : 2018.02.02
  * Revised Date    : 2018.03.01
  *
  * Author          : Mingye Xie
  ******************************************************************************
  */

#include "System.h"

uint8_t sysConnect  = 0;        // Flag for system working (Receive first heartbeat from FC)
uint8_t sysWarning  = 0;        // Counter for fatal error
uint8_t sysStatusTemp = 0;
uint8_t sysStatusLst = 0;        // Flag for battery , 0 for no problem
uint8_t sysStatus   = 0;        // Flag for battery , 0 for no problem
uint16_t sysTicks   = 0;        // Record system running time
uint8_t sysBattery  = 0;
uint8_t sysFlying   = 0;        // Flag for record whether drone is in the air

uint8_t msgLostCnt = 0;         // Mavlink Communication Lost Counter

mavlink_message_t mavMsgTx;     // Send mavlink massage
uint16_t sendCnt = 0;

char* logList[64]={
    LOG_00,LOG_01,LOG_02,"","","","","","","","","","","","","",
    LOG_10,LOG_11,LOG_12,LOG_13,LOG_14,LOG_15,LOG_16,LOG_17,"","","","","","","","",
    LOG_20,LOG_21};

char logSend[100]={""};

extern uint8_t LandingGear_Reset(void);

void System_Heartbeat(void)
{	
    PRINTLOG("\r\n\r\n [HRT]  #%d",++sysTicks);      // Record running time
    sendCnt = mavlink_msg_heartbeat_pack(1, 1, &mavMsgTx, MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_PX4, 81, 1016, MAV_STATE_STANDBY);
    Mavlink_SendMessage(&mavMsgTx, sendCnt);
}

void System_StatusReporter(void)
{
    if(sysConnect)           // Must report after connected with FMU, otherwise it's dummy
    {
        if(sysStatus&&(sysStatusLst!=sysStatus))    // Prevent report same error message continuously
        {
            if((sysStatus>=0x10)&&(sysStatus<0x20))
            {
                if((sysBattery&ERR_BATTA)&&((sysBattery&ERR_BATTB)))    sprintf(logSend,"All battery ");
                else if((sysBattery&ERR_BATTA))                         sprintf(logSend,"battery A ");
                else if((sysBattery&ERR_BATTB))                         sprintf(logSend,"battery B ");
                else                                                    sprintf(logSend,"");
                sysBattery = 0;
                strcat(logSend,logList[sysStatus]);
                Mavlink_SendLog(sysStatus, logSend);
                PRINTLOG("\r\n [INFO] Status Reporter_0x%X: %s", sysStatus, logSend);
            }
            else
            {
                Mavlink_SendLog(sysStatus, logList[sysStatus]);
                PRINTLOG("\r\n [INFO] Status Reporter_0x%X: %s", sysStatus, logList[sysStatus]);
            }
        }
        sysStatusLst = sysStatus;
    }
}

void Mavlink_SendLog(uint8_t id, char* msg)
{
    uint16_t cnt;
    cnt = mavlink_msg_stm32_f3_command_pack(1, 1, &mavMsgTx, id, msg);
    Mavlink_SendMessage(&mavMsgTx, cnt);
}

void System_ErrorHandler(void)
{
    #ifndef INGORE_LOSTCOMM
    if(sysConnect)      msgLostCnt++;           // msgLostCnt will reset if receive mavlink msg
    #endif

    // Lost connect with from FMU
    if(msgLostCnt==3)
    {
        sysConnect = 0;
        PRINTLOG("\r\n [ERR]  FMU Connect Lost");
        PRINTLOG("\r\n [ACT]  Reset USART1");
        Mavlink_SendLog(ERR_SYS_SERIAL, logList[ERR_SYS_SERIAL]); 
        msgLostCnt++;
        
        #ifdef  ENABLE_LANGINGGEAR
        // Reset Landing Gear
        sysStatusTemp = LandingGear_Reset();
        if(sysStatusTemp) sysStatus = sysStatusTemp;
        #endif  //ENABLE_LANGINGGEAR
    }
    else if(msgLostCnt==4)
    {
        USART_ReInit();                     // Reset USART
        msgLostCnt = 0;
        PRINTLOG("\r\n [WARN] sysWarning = %d",++sysWarning);
    }

    // Reset System
//    if(sysWarning >= 4)
//    {
//        PRINTLOG("\r\n [ACT]  System: Reset...");
//        Mavlink_SendLog(ERR_SYS_GENERAL, logList[ERR_SYS_GENERAL]);
//        NVIC_SystemReset();
//    }
}


void Mavlink_SendMessage(mavlink_message_t* msg, uint16_t length)
{
    uint8_t buffer[300];                        // Mavlink max length is 263 (v1)
    mavlink_msg_to_send_buffer(buffer, msg);
    Serial_Send(&USART1_Tx, buffer, length);
}


void PRINTLOG(const char *format, ...)
{
    char str[300];
    va_list arg;
    int ret = 0;
    va_start(arg, format);
    ret = vsprintf(str, format, arg);
    va_end(arg);
    
    Serial_Send(&USART3_Tx, (uint8_t*)str, ret);
}

//void DELAY_MS(int32_t nms)
//{  
//    int32_t temp;  
//    SysTick->LOAD = 8000*nms;   // HCLK/8 = 64M/8
//    SysTick->VAL  = 0X00;       // Clear counter  
//    SysTick->CTRL = 0X01;       // Enable
//    do  
//    {  
//        temp=SysTick->CTRL;     // Load
//    }
//    while((temp&0x01)&&(!(temp&(1<<16)))); // Waiting 
//    SysTick->CTRL = 0x00;       // Disable 
//    SysTick->VAL  = 0X00;       // Clear counter  
//} 

/*****END OF FILE****/
