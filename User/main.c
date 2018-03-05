/**
  ******************************************************************************
  * File Name       : main.c
  * Description     : CompanionComputer_STM32 main program
  *
  * Version         : v0.2
  * Created Date    : 2017.11.23
  * Revised Date    : 2018.03.02
  *
  * Author          : Mingye Xie
  ******************************************************************************
  */

#include "main.h"

/* Basic Function */
static void SystemClock_Config(void);
static void Error_Handler(void);

/* Serial & Mavlink */
uint8_t recvByte = 0;
uint16_t msgSeqPrev = 0;            // Monitor quantity of Mavlink lost package 

uint32_t ADC_Value[60]={0};
extern ADC_HandleTypeDef hadc1;

mavlink_message_t mavMsgRx;
mavlink_heartbeat_t mavHrt;
mavlink_status_t mavSta;
mavlink_command_long_t mavCmdRx;
mavlink_command_ack_t mavCmdAck;
mavlink_battery_status_t mavBattRx;
mavlink_stm32_f3_command_t mavF3CmdRx;
void Mavlink_Decode(mavlink_message_t* msg);

extern uint8_t battReinit;
extern TIM_HandleTypeDef htim7;
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

    PRINTLOG("\r\n [INFO] Init: I2C");
    I2C_Init();

    PRINTLOG("\r\n [INFO] Init: Battery");
    do{sysStatus = Batt_Init();}  while(sysStatus==0x10);

    #ifdef ENABLE_LANGINGGEAR
    PRINTLOG("\r\n [INFO] Init: LandingGear");
    LandingGear_Init();
    #endif //ENABLE_LANGINGGEAR

    PRINTLOG("\r\n [INFO] Init: Watchdog");
    IWDG_Init();

    PRINTLOG("\r\n [INFO] Init: ADC");
    DMA_Init();
    ADC_Init();
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_Value, 60);    
    
    PRINTLOG("\r\n [INFO] Init: Timer");
    TIM_Init();
    TIM_Start();

    while(1)
    {	
        /************************* Mavlink Receive Process *************************/
        if(Serial_Rx_Available())
        {
            recvByte = Serial_Rx_NextByte();

            if(mavlink_parse_char(MAVLINK_COMM_0, recvByte, &mavMsgRx, &mavSta))
            {
                msgLostCnt = 0;                         // Clear Communication Lost flag
                sysWarning = 0;                         // Clear communication error

                if(!sysConnect)                         // Receive first mavlink msg, start system
                {
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
        
        /***** Landing Gear Auto Reset Process  *****/
        if(lgAutoReset) LG_Reset();      
        
        /***** Battery ReInit Process *****/
        if(battReinit)
        {
            sysStatusTemp = Batt_Init();
            if(sysStatusTemp != 0x10) 
            {
                battReinit = 0;
                HAL_TIM_Base_Start_IT(&htim7);
                sysStatus = sysStatusTemp;
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
    if(htim->Instance == TIM2)      // TIM2: System Management (1Hz)
    {
        System_Heartbeat();
        System_StatusReporter();
        System_ErrorHandler();
        IWDG_Feed();                // Feed watchdog	
        PRINTLOG("\r\n [INFO] ADC_Value:%d,%d,%d,%d,%d,%d",ADC_Value[0],ADC_Value[1],ADC_Value[2],ADC_Value[3],ADC_Value[4],ADC_Value[5]);
    }

    #ifdef ENABLE_LANGINGGEAR
    if(htim->Instance == TIM6)      // TIM6: Landing Gear PWM Adjustment (100Hz)
    {
        LandingGear_Adjustment();
    }
    #endif //ENABLE_LANGINGGEAR

    if(htim->Instance == TIM7)      // TIM7: Read & Send Battery Message (40Hz)
    {
       sysStatusTemp = Battery_Management();
       if(sysStatusTemp) sysStatus = sysStatusTemp;
    }
}


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
                case MAV_CMD_AIRFRAME_CONFIGURATION:    // 2520,0x09D8
                    LandingGear_Control(&mavCmdRx);
                    break;

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
            PRINTLOG("\r\n [FMU]  #147: 0x%02X,%dC,%dV,%dA,%d%%", 
                    mavBattRx.id, mavBattRx.temperature, mavBattRx.voltages[0], mavBattRx.current_battery, mavBattRx.battery_remaining);
            break;
        
        /* STM32_F3_COMMAND (#500)*/
        case MAVLINK_MSG_ID_STM32_F3_COMMAND:
            mavlink_msg_stm32_f3_command_decode(msg, &mavF3CmdRx);
            PRINTLOG("\r\n [FMU]  #500: 0x%02X,%s", mavF3CmdRx.command, mavF3CmdRx.f3_log);
            switch(mavF3CmdRx.command)
            {
                case CMD_FLY_ARM:
                    sysFlying = 1;
                    PRINTLOG("\r\n [INFO] Drone is flying");
                    break;
                case CMD_FLY_DISARM:
                    sysFlying = 0;
                    PRINTLOG("\r\n [INFO] Drone is landing");
                    break;
                default:break;
            }
            break;
        
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
