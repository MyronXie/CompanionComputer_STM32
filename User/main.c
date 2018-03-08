/**
  ******************************************************************************
  * File Name       : main.c
  * Description     : CompanionComputer_STM32 main program
  *
  * Version         : v0.3
  * Created Date    : 2017.11.23
  * Revised Date    : 2018.03.08
  *
  * Author          : Mingye Xie
  ******************************************************************************
  */

#include "main.h"

/* Basic Function */
static void SystemClock_Config(void);
static void Error_Handler(void);

/* Extern Variable */
extern TIM_HandleTypeDef htim7;

/* Serial */
uint8_t recvByte = 0;
uint16_t msgSeqPrev = 0;            // Monitor quantity of Mavlink lost package

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
    PRINTLOG("\r\n************* CompanionComputer_STM32 **************\r\n");
    LED_Init();

    #ifdef ENABLE_BATTERYMGMT
    PRINTLOG("\r\n [INFO] Init Battery Management System");
    I2C_Init();
    do{sysStatus = Batt_Init();}  while(sysStatus==MSG_BATT_WAITING);
    #endif //ENABLE_BATTERYMGMT

    #ifdef ENABLE_LANGINGGEAR
    PRINTLOG("\r\n [INFO] Init Landing Gear Control System");
    LandingGear_Init();
    #endif //ENABLE_LANGINGGEAR

    #ifdef ENABLE_CURRMONITOR
    PRINTLOG("\r\n [INFO] Init Current Monitor System");
    CurrMonitor_Init();
    #endif //ENABLE_CURRMONITOR

    PRINTLOG("\r\n [INFO] Init CC_STM32 Misc System");
    IWDG_Init();
    TIM_Init();

    while(1)
    {
        /************************* Mavlink Decode Process *************************/
        if(Serial_Rx_Available())
        {
            // Get available data from FIFO of Usart
            recvByte = Serial_Rx_NextByte();

            if(mavlink_parse_char(MAVLINK_COMM_0, recvByte, &mavMsgRx, &mavSta))
            {
                msgLostCnt = 0;                         // Clear Communication Lost flag
                sysWarning = 0;                         // Clear communication error

                // <Dev> Display all mavlink msg received
                //PRINTLOG("\r\n [Info] Mav: %d",mavMsgRx.msgid);

                if(!sysConnect)
                {
                    // Receive first mavlink msg in this process
                    PRINTLOG("\r\n [INFO] Connected with FMU");
                    sysConnect = 1;
                }
                else
                {
                    // Monitor lost package number of Mavlink
                    if((mavMsgRx.seq-msgSeqPrev!=1)&&(mavMsgRx.seq+256-msgSeqPrev!=1))
                    {
                        PRINTLOG("\r\n [WARN] Mavlink lost: %d", (mavMsgRx.seq>msgSeqPrev)?(mavMsgRx.seq-msgSeqPrev-1):(mavMsgRx.seq+256-msgSeqPrev-1));
                    }
                }
                msgSeqPrev=mavMsgRx.seq;

                Mavlink_Decode(&mavMsgRx);
            }
        }

        /********** LandingGear: Auto Reset Process  **********/
        if(lgAutoReset) LG_Reset();

        /********** BattMgmt: Power Off Process **********/
        if(battPwrOff == 1)
        {
            sysStatusTemp = Batt_PowerOff();
            if(sysStatusTemp != MSG_BATT_WAITING)
            {
                sysStatus = sysStatusTemp;
            }
        }

        /********** BattMgmt: ReInit Process **********/
        if(battReinit)
        {
            sysStatusTemp = Batt_Init();
            if(sysStatusTemp != MSG_BATT_WAITING)
            {
                battReinit = 0;
                HAL_TIM_Base_Start_IT(&htim7);
                sysStatus = sysStatusTemp;
            }
        }

        System_StatusReporter();
    }//while
}//main

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2)      // TIM2: System Management (1Hz)
    {
        System_Heartbeat();
        System_ErrorHandler();
        IWDG_Feed();                // Feed watchdog
    }

    if(htim->Instance == TIM6)      // TIM6: Landing Gear PWM Adjustment (100Hz)
    {
        LandingGear_Adjustment();
    }

    if(htim->Instance == TIM7)      // TIM7: Read & Send Battery Message (40Hz)
    {
       sysStatusTemp = Battery_Management();
       if(sysStatusTemp) sysStatus = sysStatusTemp;
    }

    if(htim->Instance == TIM15)     // TIM15: Send ESC Current Message (20Hz)
    {
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
            PRINTLOG("\r\n [FMU]  #0  : %d,%d", mavHrt.type, mavHrt.autopilot);
            break;

        /* COMMAND_LONG (#76) */
        case MAVLINK_MSG_ID_COMMAND_LONG:
            mavlink_msg_command_long_decode(msg, &mavCmdRx);
            PRINTLOG("\r\n [FMU]  #76 : %4d,%d,%d", mavCmdRx.command, (int)mavCmdRx.param1, (int)mavCmdRx.param2);    // Only use 2 params at present
            switch(mavCmdRx.command)
            {
                #ifdef ENABLE_LANGINGGEAR
                case MAV_CMD_AIRFRAME_CONFIGURATION:    // 2520,0x09D8
                    LandingGear_Control(&mavCmdRx);
                    break;
                #endif

                default:break;
            }
            break;

        /* COMMAND_ACK (#77) */
        case MAVLINK_MSG_ID_COMMAND_ACK:
            mavlink_msg_command_ack_decode(msg, &mavCmdAck);
            PRINTLOG("\r\n [FMU]  #77 : %4d,%d", mavCmdAck.command, mavCmdAck.result);    // .progess is dummy
            break;

        /* BATTERY_STATUS (#147)*/
        case MAVLINK_MSG_ID_BATTERY_STATUS:
            mavlink_msg_battery_status_decode(msg, &mavBattRx);
            PRINTLOG("\r\n [FMU]  #147: 0x%02X,%dC,%dmV,%d0mA,%d%%",
                    mavBattRx.id, mavBattRx.temperature, mavBattRx.voltages[0], mavBattRx.current_battery, mavBattRx.battery_remaining);
            break;

        /* STM32_F3_COMMAND (#500)*/
        case MAVLINK_MSG_ID_STM32_F3_COMMAND:
            mavlink_msg_stm32_f3_command_decode(msg, &mavF3CmdRx);
            PRINTLOG("\r\n [FMU]  #500: 0x%02X,\"%s\"", (uint8_t)mavF3CmdRx.command, mavF3CmdRx.f3_log);
            switch(mavF3CmdRx.command)
            {
                case CMD_FLY_ARM:
                    if(!sysArmed)
                    {
                        sysArmed = 1;
                        PRINTLOG("\r\n [INFO] Drone Armed");
                    }
                    break;
                case CMD_FLY_DISARM:
                    if(sysArmed)
                    {
                        sysArmed = 0;
                        PRINTLOG("\r\n [INFO] Drone Disarmed");
                    }
                    break;
                default:break;
            }
            break;

        /* STM32_F3_MOTOR_CURR (#501)*/
        #ifdef ENABLE_CURRMONITOR
        case MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR:
            mavlink_msg_stm32_f3_motor_curr_decode(msg, &mavF3Curr);
            PRINTLOG("\r\n [FMU]  #501: %.2f,%.2f,%.2f,%.2f,%.2f,%.2f", mavF3Curr.motor_curr[0],mavF3Curr.motor_curr[1],
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
