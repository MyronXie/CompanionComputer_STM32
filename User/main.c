/**
  ******************************************************************************
  * File Name       : main.c
  * Description     : CompanionComputer_STM32 main program
  *
  * Version         : v0.3.1
  * Created Date    : 2017.11.23
  * Revised Date    : 2018.04.18
  *
  * Author          : Mingye Xie
  ******************************************************************************
  */

#include "main.h"

/* Basic Function */
static void SystemClock_Config(void);
static void Error_Handler(void);

/* Serial */
uint8_t recvByte = 0;
uint16_t msgSeqPrev = 0;            // Monitor numbers of Mavlink lost package

/* Mavlink */
mavlink_message_t mavMsgRx;
mavlink_heartbeat_t mavHrt;
mavlink_status_t mavSta;
mavlink_command_long_t mavCmdRx;
mavlink_command_ack_t mavCmdAck;
mavlink_battery_status_t mavBattRx;
mavlink_stm32_f3_command_t mavF3CmdRx;
mavlink_stm32_f3_motor_curr_t mavF3Curr;
void Mavlink_Decode(mavlink_message_t* msg);

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    USART_Init();
    PRINTLOG("\r\n*********** CompanionComputer_STM32 v0.4 ************\r\n");
    LED_Init();

    #ifdef ENABLE_BATTERYMGMT
    PRINTLOG("\r\n INFO|BattMgmt|Init Battery Management System");
    I2C_Init();
    while(battInit==1)     Battery_Init();
    #endif //ENABLE_BATTERYMGMT

    #ifdef ENABLE_LANGINGGEAR
    PRINTLOG("\r\n INFO|LandGear|Init Landing Gear Control System");
    LandingGear_Init();
    #endif //ENABLE_LANGINGGEAR

    #ifdef ENABLE_CURRMONITOR
    PRINTLOG("\r\n INFO|CurrMon |Init Current Monitor System");
    CurrMonitor_Init();
    #endif //ENABLE_CURRMONITOR

    PRINTLOG("\r\n INFO| System |Init CC_STM32 Misc System");
    IWDG_Init();
    TIM_Init();

    #ifdef INGORE_LOSTCOMM
    sysConnect = 1;
    #endif

    while(1)
    {

        /************************* Mavlink Decode Process *************************/
        if(Serial_Mavlink_Available())
        {
            // Get available data from FIFO of Usart
            recvByte = Serial_Mavlink_NextByte();

            if(mavlink_parse_char(MAVLINK_COMM_0, recvByte, &mavMsgRx, &mavSta))
            {
                msgLostCnt = 0;                         // Clear Communication Lost flag
                sysWarning = 0;                         // Clear communication error

                // <Debug> Display all mavlink msg received
                //PRINTLOG("\r\nDEBUG|Mavlink |msgid: %d",mavMsgRx.msgid);

                if(!sysConnect)
                {
                    // Receive first mavlink msg in current process
                    PRINTLOG("\r\n INFO| System |Connected with FMU");
                    sysConnect = 1;
                }
                else
                {
                    // Monitor lost package number of Mavlink
                    if((mavMsgRx.seq-msgSeqPrev!=1)&&(mavMsgRx.seq+256-msgSeqPrev!=1))
                        PRINTLOG("\r\n WARN|Mavlink |Lost %d package(s)",(mavMsgRx.seq>msgSeqPrev)?(mavMsgRx.seq-msgSeqPrev-1):(mavMsgRx.seq+256-msgSeqPrev-1));
                }

                msgSeqPrev=mavMsgRx.seq;                // Record seqid of mavlink msg

                Mavlink_Decode(&mavMsgRx);              // Decode specific mavlink msg
            }
        }

        /************************* Console Recv Process *************************/
        if(Serial_Console_Available())
        {
            recvByte = Serial_Console_NextByte();

            if(recvByte!='\r'&&recvByte!='\n')
            {
                PRINTLOG("\r\n INFO|Console |\"%c\"",recvByte);

                switch(recvByte)
                {
                    case '0': LandingGear_ChangePosition(0);break;
                    case '1': LandingGear_ChangePosition(1);break;
                    case '2': lgAutoReset = 1;break;
                    case 'Q': sysArmed = 1; PRINTLOG("\r\n INFO [System] Drone Armed");break;
                    case 'q': sysArmed = 0; PRINTLOG("\r\n INFO [System] Drone Disarmed");break;
                    case 'R': NVIC_SystemReset();   break;
                    default: Console_BattMgmt(recvByte); break;
                }
            }
        }
    }//while
}//main

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    System_MsgReporter();        // Put here to avoid timing sync bug in the initial part

    if(htim->Instance == TIM2)      // TIM2: System Management (1Hz)
    {
        LED_TOGGLE(LED1);
        System_Heartbeat();
        System_ErrorHandler();
        IWDG_Feed();                    // Feed watchdog
    }

    if(htim->Instance == TIM6)      // TIM6: Landing Gear PWM Adjustment (50Hz)
    {
        LandingGear_Control();
    }

    if(htim->Instance == TIM7)      // TIM7: Read & Send Battery Message (40Hz)
    {
        if(battInit == 1)       Battery_Init();
        else                    Battery_Management();
        if(battPwrOff == 1)     Batt_PowerOff();
    }

    if(htim->Instance == TIM15)     // TIM15: Send ESC Current Message (20Hz)
    {
        CurrMonitor_Capture();
        CurrMonitor_Send();
    }
}

/**
  * @brief  Decode mavlink message
  * @param  msg mavlink message
  * @retval None
  */
void Mavlink_Decode(mavlink_message_t* msg)
{
    switch(msg->msgid)
    {
        /* HEARTBEAT (#0) */
        case MAVLINK_MSG_ID_HEARTBEAT:
            mavlink_msg_heartbeat_decode(msg, &mavHrt);
            PRINTLOG("\r\n INFO|Mavlink |Msg #0  : %d,%d", mavHrt.type, mavHrt.autopilot);
            break;

        /* COMMAND_LONG (#76) */
        case MAVLINK_MSG_ID_COMMAND_LONG:
            mavlink_msg_command_long_decode(msg, &mavCmdRx);
            PRINTLOG("\r\n INFO|Mavlink |Msg #76 : %d,%d,%d", mavCmdRx.command, (int)mavCmdRx.param1, (int)mavCmdRx.param2);
            switch(mavCmdRx.command)
            {
                #ifdef ENABLE_LANGINGGEAR
                case MAV_CMD_AIRFRAME_CONFIGURATION:    // 2520,0x09D8
                    LandingGear_ChangePosition((int)mavCmdRx.param2);
                    break;
                #endif

                default:break;
            }
            break;

        /* COMMAND_ACK (#77) */
        case MAVLINK_MSG_ID_COMMAND_ACK:
            mavlink_msg_command_ack_decode(msg, &mavCmdAck);
            PRINTLOG("\r\n INFO|Mavlink |Msg #77 : %d,%d", mavCmdAck.command, mavCmdAck.result);    // .progess is dummy
            break;

        /* BATTERY_STATUS (#147)*/
        case MAVLINK_MSG_ID_BATTERY_STATUS:
            mavlink_msg_battery_status_decode(msg, &mavBattRx);
            PRINTLOG("\r\n INFO|Mavlink |Msg #147: 0x%02X,%dC,%dmV,%d0mA,%d%%",
                    mavBattRx.id, mavBattRx.temperature, mavBattRx.voltages[0], mavBattRx.current_battery, mavBattRx.battery_remaining);
            break;

        /* STM32_F3_COMMAND (#500)*/
        case MAVLINK_MSG_ID_STM32_F3_COMMAND:
            mavlink_msg_stm32_f3_command_decode(msg, &mavF3CmdRx);
            PRINTLOG("\r\n INFO|Mavlink |Msg #500: 0x%02X,0x%02X,\"%s\"", (uint8_t)mavF3CmdRx.command, (uint8_t)mavF3CmdRx.param, mavF3CmdRx.f3_log);
            switch(mavF3CmdRx.command)
            {
                case CMD_FLY_ARM:
                    if(!sysArmed)       // Only display message when armed status changed
                    {
                        sysArmed = 1;
                        PRINTLOG("\r\n INFO| System |Drone Armed");
                    }
                    break;
                case CMD_FLY_DISARM:
                    if(sysArmed)
                    {
                        sysArmed = 0;
                        PRINTLOG("\r\n INFO| System |Drone Disarmed");
                    }
                    break;
                default:break;
            }
            break;

        /* STM32_F3_MOTOR_CURR (#501)*/
        #ifdef ENABLE_CURRMONITOR
        case MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR:
            mavlink_msg_stm32_f3_motor_curr_decode(msg, &mavF3Curr);
            PRINTLOG("\r\n INFO|Mavlink |Msg #501: %.2f,%.2f,%.2f,%.2f,%.2f,%.2f", mavF3Curr.motor_curr[0],mavF3Curr.motor_curr[1],
                        mavF3Curr.motor_curr[2],mavF3Curr.motor_curr[3],mavF3Curr.motor_curr[4],mavF3Curr.motor_curr[5]);
            break;
        #endif

        default:break;
    }
}


/**
  * @brief  System Clock Configuration
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 64000000
  *            HCLK(Hz)                       = 64000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            PLLMUL                         = RCC_PLL_MUL16 (16)
  *            Flash Latency(WS)              = 2
  */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /* HSI Oscillator already ON after system reset, activate PLL with HSI as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
    PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    /* Configure the Systick interrupt time */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /* Configure the Systick */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

static void Error_Handler(void)
{
    while(1)  {  }
}

/*****END OF FILE****/
