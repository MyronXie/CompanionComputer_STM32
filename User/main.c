/**
  ******************************************************************************
  * File Name		: main.c
  * Description		: CompanionComputer_STM32 main program
  *
  * Version			: v0.1.1
  * Created	Date	: 2017.11.23
  * Revised	Date	: 2017.12.26
  *
  * Author			: Mingye Xie
  ******************************************************************************
  */

#include "main.h"

/* System Function */
static void SystemClock_Config(void);
static void Error_Handler(void);

/* Extern parameter*/
extern UART_HandleTypeDef huart1,huart3;
extern uint8_t aRxBuffer;
extern TIM_HandleTypeDef htim6,htim7;

IWDG_HandleTypeDef hiwdg;
void IWDG_Init(void);

/* Parameter for Mavlink */
mavlink_message_t msgRx,msgTx;
mavlink_heartbeat_t hrtRx;
mavlink_status_t sta;
mavlink_command_long_t cmdRx,cmdTx;
mavlink_battery_status_t batt1,batt2;

uint8_t msgOK = 0;			//Mavlink Receive flag
uint8_t msgLostCnt=0;		//Mavlink Lost Counter
uint8_t bufferTx[263];		//Mavlink max length is 263
uint16_t sendBytes=0; 

/* Parameter for Landing Gear */
uint8_t lgPositionRecv=0;
uint8_t lgPositionCurr=0;			//Position of Landing Gear [Down: 0, Up: 1]
uint8_t lgPositionPrev=0;
uint8_t lgChangeStatus=0;			//Change Status of Landing Gear
uint8_t lgChangeStatusPrev=0;
uint8_t lgChangeProgress=50;		//Change Progress of Landing Gear

/* Parameter for Battery Control */
uint16_t regData;

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
	printf("\r\n*****CompanionComputer_STM32*****\r\n");

	printf("\r\n# Init Landing Gear");
	LG_Init();

	IWDG_Init();
	
	printf("\r\n# Init Timer");
	TIM_Init();
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	
//	printf("\r\n# Init Battery\r\n");
//	Batt_Init();	
//	batt1.id				= 0x16;
//	batt1.current_consumed	= 100;
//	batt1.temperature		= 2760;
//	batt1.voltages[0]		= 24360;
//	batt1.current_battery	= -1602;
//	batt1.battery_function	= MAV_BATTERY_FUNCTION_ALL;
//	batt1.type				= MAV_BATTERY_TYPE_LIPO;
//	batt1.battery_remaining	= 99;

	printf("\r\n> System Running!\r\n");

	while(1)
	{	
		/************************* Mavlink Decode Process *************************/
		if(msgOK)
		{
			msgOK=0;		//Clear Mavlink Receive flag
			msgLostCnt=0;
			printf("\r\n[MSG]%3d,%3d,%d,%d,%3d;", msgRx.len, msgRx.seq, msgRx.sysid, msgRx.compid, msgRx.msgid);
			switch(msgRx.msgid)
			{
				/* #0: Heartbeat */
				case MAVLINK_MSG_ID_HEARTBEAT:
					mavlink_msg_heartbeat_decode(&msgRx,&hrtRx);
					printf(" {HRTBEAT} %d,%d,%d,%d,%d,%d",hrtRx.type,hrtRx.autopilot,hrtRx.base_mode,hrtRx.custom_mode,hrtRx.system_status,hrtRx.mavlink_version);
					break;
				
				/* #76: Command_Long */
				case MAVLINK_MSG_ID_COMMAND_LONG: 
					mavlink_msg_command_long_decode(&msgRx, &cmdRx);
					printf(" {CMDLONG} %4d,%d,%d",cmdRx.command,(int)cmdRx.param1,(int)cmdRx.param2);
					
					switch(cmdRx.command)
					{
						/********** Landing Gear Control **********/
						case MAV_CMD_AIRFRAME_CONFIGURATION:
							printf("\r\n[CMD]LandingGear");
							lgPositionRecv=(int)cmdRx.param2;
							if(lgPositionRecv!=0&&lgPositionRecv!=1)	//Parameter Check
							{
								printf("%s",":Wrong Parameter!");
								break;
							}
							else	printf("(%s)",lgPositionRecv?"UP":"DOWN");

							/* Landing Gear is changing */
							if(lgChangeStatus)
							{
								if(lgPositionRecv!=lgPositionCurr)
								{
									printf(":%s","Ingore!");					//Ignore too fast command
									sendBytes=mavlink_msg_command_ack_pack(1, 1, &msgTx, MAV_CMD_AIRFRAME_CONFIGURATION, MAV_RESULT_TEMPORARILY_REJECTED, 0);
									mavlink_msg_to_send_buffer(bufferTx, &msgTx);
									HAL_UART_Transmit_IT(&huart1,bufferTx,sendBytes);
								}
								else
								{
									printf(":%s","Changing!");
									sendBytes=mavlink_msg_command_ack_pack(1, 1, &msgTx, MAV_CMD_AIRFRAME_CONFIGURATION, MAV_RESULT_IN_PROGRESS, lgChangeProgress);
									mavlink_msg_to_send_buffer(bufferTx, &msgTx);
									HAL_UART_Transmit_IT(&huart1,bufferTx,sendBytes);
								}

							}
							/* Landing Gear is standby */
							else
							{
								lgPositionCurr=lgPositionRecv;
								/* Adjust position of Landing Gear */
								if(lgPositionPrev!=lgPositionCurr)
								{
									lgChangeStatus=1;
									lgPositionPrev=lgPositionCurr;
								}
								else
								{
									printf("%s","Already changed!");
								}
							}

							break;//MAV_CMD_AIRFRAME_CONFIGURATION
						
						default:break;
					}//switch(cmdRx.command)

					break;

				/* #147: Battery Status */
				case MAVLINK_MSG_ID_BATTERY_STATUS: //for test, noused now
					mavlink_msg_battery_status_decode(&msgRx,&batt2);
					printf(" {BATTERY} ID:%d,V:%.2f,I:%.2f,T:%.2f,R:%d",batt2.id,batt2.voltages[0]/1000.0,batt2.current_battery/100.0,batt2.temperature/100.0,batt2.battery_remaining);
					break;
				
				default:break;
			}
		}

		/* Lost communication with FC */
		if(msgLostCnt==3)
		{
			printf("\r\n# Reset Landing Gear");
			HAL_TIM_Base_Stop_IT(&htim6);
			LG_Reset();
			lgPositionCurr=0;
			lgPositionPrev=0;
			HAL_TIM_Base_Start_IT(&htim6);
		}
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)
	{
		/* Receive MavLink massage */
		if(mavlink_frame_char(MAVLINK_COMM_0, aRxBuffer, &msgRx, &sta) != MAVLINK_FRAMING_INCOMPLETE)
		{
			msgOK =1;
		}	
		HAL_UART_Receive_IT(&huart1,&aRxBuffer,1); 	//Restart usart1's IT for next receive process
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* TIM6: Landing Gear PWM Adjustment (100Hz) */
	if(htim->Instance==TIM6)
	{
		if(lgChangeStatusPrev!=lgChangeStatus)	//Status change msg
		{
			lgChangeStatusPrev=lgChangeStatus;
			if(lgChangeStatus)
			{
				printf("\r\n# TIM6: Start(%s)",lgPositionCurr?"UP":"DOWN");
				sendBytes=mavlink_msg_command_ack_pack(1, 1, &msgTx, MAV_CMD_AIRFRAME_CONFIGURATION, MAV_RESULT_ACCEPTED, 1);
				mavlink_msg_to_send_buffer(bufferTx, &msgTx);
				HAL_UART_Transmit_IT(&huart1,bufferTx,sendBytes);
			}
			else
			{
				printf("\r\n# TIM6: Stop(%s)",lgPositionCurr?"UP":"DOWN");
				sendBytes=mavlink_msg_command_ack_pack(1, 1, &msgTx, MAV_CMD_AIRFRAME_CONFIGURATION, MAV_RESULT_ACCEPTED, 100);
				mavlink_msg_to_send_buffer(bufferTx, &msgTx);
				HAL_UART_Transmit_IT(&huart1,bufferTx,sendBytes);
			}
		}
		/* Landing Gear changing process */
		if(lgChangeStatus)
		{
			Relay_ON();						//Turn on relay to power on steers
			lgChangeStatus=LG_Control(lgPositionCurr,&lgChangeProgress);	//If changing process finished, lgChangeStatus turns 0
		}
		else Relay_OFF();					//Turn off relay to power off steers
	}
	
	/* TIM7: HeartBeat (1Hz) */
	if(htim->Instance==TIM7)
	{
		msgLostCnt++;
		printf("\r\n> Heartbeat");
		mavlink_msg_heartbeat_pack(1, 1, &msgTx, MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_PX4, 81, 65536, MAV_STATE_STANDBY);
		HAL_IWDG_Refresh(&hiwdg);
	}
}

void IWDG_Init(void)
{
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
	hiwdg.Init.Window = 4095;
	hiwdg.Init.Reload = 2000;
	HAL_IWDG_Init(&hiwdg);
}

static void Error_Handler(void)
{
  while(1)  {  }
}

/*****END OF FILE****/
