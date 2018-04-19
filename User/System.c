/**
  ******************************************************************************
  * File Name       : System.c
  * Description     : System param for CC_STM32
  *
  * Version         : v0.3.1
  * Created Date    : 2018.02.02
  * Revised Date    : 2018.04.09
  *
  * Author          : Mingye Xie
  ******************************************************************************
  */

#include "System.h"

uint8_t  sysConnect     = 0;    // Flag for system working (Receive first heartbeat from FC)
uint8_t  sysWarning     = 0;    // Counter for fatal error
uint16_t sysTicks       = 0;    // Record system running time
uint8_t  sysArmed       = 0;    // Flag for record whether drone is in the air

mavlink_message_t mavMsgTx;     // Send mavlink massage
uint8_t  msgLostCnt     = 0;    // Mavlink Communication Lost Counter
uint16_t sendCnt        = 0;

QueueType msgQ;                 // Message Queue

extern uint8_t lgAutoReset;
extern SerialType USART1_Tx,USART3_Tx;

// msgList from system.h
char* msgList[48]={
    MSG_00,MSG_01,MSG_02,MSG_03,MSG_04,MSG_05,MSG_06,MSG_XX,MSG_XX,MSG_XX,MSG_XX,MSG_XX,MSG_XX,MSG_XX,MSG_XX,MSG_XX,
    MSG_10,MSG_11,MSG_XX,MSG_XX,MSG_XX,MSG_XX,MSG_XX,MSG_XX,MSG_XX,MSG_XX,MSG_XX,MSG_XX,MSG_XX,MSG_XX,MSG_XX,MSG_XX,
    MSG_20,MSG_21,MSG_22,MSG_23,MSG_24,MSG_25,MSG_26,MSG_27,MSG_28,MSG_29,MSG_2A,MSG_2B,MSG_2C,MSG_2D,MSG_XX,MSG_XX};

char* paramList[4]={PARAM_BATT_0,PARAM_BATT_1,PARAM_BATT_2,PARAM_BATT_3};

void System_Heartbeat(void)
{
    PRINTLOG("\r\n\r\n INFO| System |HrtBeat #%d",++sysTicks);     // Record running time
    sendCnt = mavlink_msg_heartbeat_pack(1, 1, &mavMsgTx, MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_PX4, 81, 1016, MAV_STATE_STANDBY);
    Mavlink_SendMessage(&mavMsgTx, sendCnt);
    srand(sysTicks);
}

void System_MsgReporter(void)
{
    static uint8_t reportTimes = 0;                 // Record report times
    static uint8_t Qcmd = 0;
    static uint8_t Qparam = 0;
    static char msgSend[100]={""};

    if(sysConnect)                                  // Must report after connected with FMU, otherwise it's useless
    {
        if(msgQ.front!=msgQ.rear)                   // MsgQueue have message(s)
        {
            if(!reportTimes)                        // First report, copy message into buffer
            {
                Qcmd    = msgQ.cmd[msgQ.front];
                Qparam  = msgQ.param[msgQ.front];
                reportTimes = 3;                    // Report same error message for 3 times

                memset(msgSend,0,sizeof(msgSend));  // Clear message buffer

                switch(Qcmd>>4)
                {
                    case 0x02:                      // Battery Message
                        if(Qparam<4)    strcpy(msgSend,paramList[Qparam]);
                        break;

                    default:    break;
                }

                strcat(msgSend, msgList[Qcmd]);
                PRINTLOG("\r\n INFO| System |Reporter: 0x%02X,0x%02X,\"%s\"", Qcmd, Qparam, msgSend);
            }

            if(reportTimes > 0)
            {
                sendCnt = mavlink_msg_stm32_f3_command_pack(1, 1, &mavMsgTx, Qcmd, Qparam, msgSend);
                Mavlink_SendMessage(&mavMsgTx, sendCnt);
                reportTimes--;
            }
            if(reportTimes == 0)
            {
                msgQ.front = (++msgQ.front) % 10;
            }
        }
    }
}

void System_ErrorHandler(void)
{
    MsgType syMsg = {0,0};

    #ifndef INGORE_LOSTCOMM
    if(sysConnect)      msgLostCnt++;               // msgLostCnt will reset if receive mavlink msg
    #endif

    // Lost connect with from FMU
    if(msgLostCnt==3)
    {
        sysConnect = 0;
        PRINTLOG("\r\n ERR | System |FMU Connect Lost");
        PRINTLOG("\r\n INFO| System |Reset USART1");
        syMsg.cmd = ERR_SYS_SERIAL;
        ReportMessage(syMsg);
        msgLostCnt++;

        #ifdef  ENABLE_LANGINGGEAR
        lgAutoReset = 1;
        syMsg.cmd = MSG_LG_RESET;
        ReportMessage(syMsg);
        #endif
    }
    else if(msgLostCnt==4)
    {
        USART_ReInit();                     // Reset USART
        msgLostCnt = 0;
        PRINTLOG("\r\n WARN| System |sysWarning: %d",++sysWarning);
    }

    // Reset System
//    if(sysWarning >= 4)
//    {
//        PRINTLOG("\r\n WARN| System |Start CC_STM32 System Reset...");
//        syMsg.cmd = ERR_SYS_GENERAL;
//        ReportMessage(syMsg);
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

void ReportMessage(MsgType msg)
{
    msgQ.cmd[msgQ.rear] = msg.cmd;
    msgQ.param[msgQ.rear] = msg.param;
    msgQ.rear = (++msgQ.rear) % 10;
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
