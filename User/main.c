/**
  ******************************************************************************
  * File Name		: main.c
  * Description		: CompanionComputer_STM32 main program
  *
  * Version			: v0.2
  * Created	Date	: 2017.11.23
  * Revised	Date	: 2018.02.02
  *
  * Author			: Mingye Xie
  ******************************************************************************
  */

#include "main.h"

/* Basic Function */
static void SystemClock_Config(void);
static void Error_Handler(void);

/* Serial & Mavlink */
uint8_t recvByte = 0;

uint16_t msgSeqPrev = 0;				// Monitor lost package number of Mavlink
mavlink_message_t mavMsgRx;
mavlink_heartbeat_t mavHrt;
mavlink_status_t mavSta;
mavlink_command_long_t mavCmdRx;
mavlink_command_ack_t mavCmdAck;
mavlink_battery_status_t mavBattRx;
mavlink_stm32_f3_command_t mavF3CmdRx;
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
	printf("\r\n**************************************************");
	printf("\r\n*             CompanionComputer_STM32            *");
	printf("\r\n**************************************************\r\n");
	
	printf("\r\n [INFO] Init: I2C");
	I2C_Init();
	
	printf("\r\n [INFO] Init: Battery");
	sysStatus = Batt_Init();
	if(sysStatus) sysReport = 1;
	
	#ifdef ENABLE_LANGINGGEAR
	printf("\r\n [INFO] Init: LandingGear");
	LandingGear_Init();
	#endif //ENABLE_LANGINGGEAR
	
	printf("\r\n [INFO] Init: Watchdog");
	IWDG_Init();

	printf("\r\n [INFO] Init: Timer");
	TIM_Init();

	printf("\r\n [SYS]  Connecting...");
	TIM_Start();
	
	while(1)
	{	
		/************************* Mavlink Receive Process *************************/
		if(Serial_Available())
		{
			recvByte = Serial_GetNextByte();
			
			if(mavlink_parse_char(MAVLINK_COMM_0, recvByte, &mavMsgRx, &mavSta))
			{
				msgLostCnt = 0;							// Clear Communication Lost flag
				sysWarning = 0;							// Current for communication error
				
				if(!sysConnect)							// Receive first mavlink msg, start system
				{
					printf("\r\n [SYS]  Connected with FMU");
					sysConnect = 1;					
				}
//				else
//				{
//					// <Dev> Monitor lost package number of Mavlink
//					if((mavMsgRx.seq-msgSeqPrev!=1)&&(mavMsgRx.seq+256-msgSeqPrev!=1))
//					{
//						printf("\r\n [WARN] Mavkink lost: %d", (mavMsgRx.seq>msgSeqPrev)?(mavMsgRx.seq-msgSeqPrev-1):(mavMsgRx.seq+256-msgSeqPrev-1));
//					}
//				}
//				msgSeqPrev=mavMsgRx.seq;
				
				Mavlink_Decode(&mavMsgRx);
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
	if(htim->Instance == TIM2)		// TIM2: System Management (1Hz)
	{
		System_Heartbeat();
		System_StatusReporter();
		System_ErrorHandler();
		IWDG_Feed();				// Feed watchdog	
	}
	
	#ifdef ENABLE_LANGINGGEAR
	if(htim->Instance == TIM6)		// TIM6: Landing Gear PWM Adjustment (100Hz)
	{
		LandingGear_Adjustment();
	}
	#endif //ENABLE_LANGINGGEAR

	if(htim->Instance == TIM7)		// TIM7: Read & Send Battery Message (40Hz)
	{
		sysStatus = Battery_Management();
		if(sysStatus) sysReport = 1;
	}
}


void Mavlink_Decode(mavlink_message_t* msg)
{
	switch(msg->msgid)
	{
		/* HEARTBEAT (#0) */
		case MAVLINK_MSG_ID_HEARTBEAT:
			mavlink_msg_heartbeat_decode(msg, &mavHrt);
			printf("\r\n [FMU]  #0  : %d,%d", mavHrt.type, mavHrt.autopilot);
			break;

		/* COMMAND_LONG (#76) */
		case MAVLINK_MSG_ID_COMMAND_LONG: 
			mavlink_msg_command_long_decode(msg, &mavCmdRx);
			printf("\r\n [FMU]  #76 : %4d,%d,%d", mavCmdRx.command, (int)mavCmdRx.param1, (int)mavCmdRx.param2);		// Only use 2 params at present
			switch(mavCmdRx.command)
			{
				case MAV_CMD_AIRFRAME_CONFIGURATION:			// 2520,0x09D8
					LandingGear_Control(&mavCmdRx);
					break;//MAV_CMD_AIRFRAME_CONFIGURATION

				default:break;
			}//switch(mavCmdRx.command)
			break;

		/* COMMAND_ACK (#77) */
		case MAVLINK_MSG_ID_COMMAND_ACK:
			mavlink_msg_command_ack_decode(msg, &mavCmdAck);
			printf("\r\n [FMU]  #77 : %4d,%d", mavCmdAck.command, mavCmdAck.result);	// .progess is dummy
			break;

		/* BATTERY_STATUS (#147)*/
		case MAVLINK_MSG_ID_BATTERY_STATUS:
			mavlink_msg_battery_status_decode(msg, &mavBattRx);
			printf("\r\n [FMU]  #147: 0x%02X,%dC,%dV,%dA,%d%%", mavBattRx.id, mavBattRx.temperature, mavBattRx.voltages[0], mavBattRx.current_battery, mavBattRx.battery_remaining);
			break;
		
		/* STM32_F3_COMMAND (#500)*/
		case MAVLINK_MSG_ID_STM32_F3_COMMAND:
			mavlink_msg_stm32_f3_command_decode(msg, &mavF3CmdRx);
			printf("\r\n [FMU]  #500: 0x%02X,%s", mavF3CmdRx.command, mavF3CmdRx.f3_log);
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
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	/* HSI Oscillator already ON after system reset, activate PLL with HSI as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
	{
		Error_Handler();
	}

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2)!= HAL_OK)
	{
		Error_Handler();
	}
}

static void Error_Handler(void)
{
	while(1)  {  }
}

/*****END OF FILE****/
