/**
  ******************************************************************************
  * File Name		: main.c
  * Description		: CompanionComputer_STM32 main program
  *
  * Version			: v0.1.1
  * Created	Date	: 2017.11.23
  * Revised	Date	: 2018.01.03
  *
  * Author			: Mingye Xie
  ******************************************************************************
  */

#include "main.h"

/* System Function */
static void SystemClock_Config(void);
static void Error_Handler(void);

/* Extern parameter */
extern UART_HandleTypeDef huart1,huart3;
extern uint8_t aRxBuffer;
extern TIM_HandleTypeDef htim6,htim7;

/* Watchdog */
IWDG_HandleTypeDef hiwdg;
void IWDG_Init(void);

/* Flash */
const uint32_t flashAddr = 0x0800F800;	// Page 31: 0x0800F800-0x0800FFFF (2K)
FLASH_EraseInitTypeDef flashErase;
uint32_t PageError = 0;
void FLASH_SaveLGStatus(uint8_t pos, uint8_t cng);
void FLASH_LoadLGStatus(uint8_t* pos, uint8_t* cng);

/* Mavlink */
mavlink_message_t mavMsgTx,mavMsgRx;
mavlink_heartbeat_t mavHrt;
mavlink_status_t mavSta;
mavlink_command_long_t mavCmdTx,mavCmdRx;
mavlink_command_ack_t mavCmdAck;
mavlink_battery_status_t batt1,batt2;

uint8_t msgRecvFin = 0;					// Mavlink Receive flag
uint8_t msgLostCnt = 0;					// Mavlink Communication Lost Counter
uint8_t bufferTx[263];					// Mavlink max length is 263
uint16_t sendBytes = 0; 				// Length of Mavlink package

/* Landing Gear */
uint8_t lgPositionRecv = 0;				//Position of Landing Gear [Down: 0, Up: 1]
uint8_t lgPositionCurr = 0;
uint8_t lgPositionPrev = 0;
uint8_t lgChangeStatus = 0;				//Change Status of Landing Gear [Standby: 0, Changing: 1]
uint8_t lgChangeStatusPrev = 0;
uint8_t lgChangeProgress = 50;			//Change Progress of Landing Gear [0-100(%)]

/* Battery Control */
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
	
	printf("\r\n# LandingGear: Init");
	LandingGear_Init();
	printf("\r\n# Flash: Load status");
	FLASH_LoadLGStatus(&lgPositionCurr,&lgChangeStatus);
	/* Landing Gear not reset in last process, Reset Landing Gear */
	if(lgPositionCurr!=0 || lgChangeStatus!=0)
	{
		printf("\r\n# LandingGear: Reset...");
		LandingGear_Reset();
		lgPositionCurr = 0, lgChangeStatus = 0;
		FLASH_SaveLGStatus(lgPositionCurr,lgChangeStatus);
	}
	
	IWDG_Init();
	
	printf("\r\n# Timer: Init");
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
		if(msgRecvFin)
		{
			msgRecvFin = 0;							//Clear Mavlink Receive flag
			msgLostCnt = 0;							//Clear Communication Lost flag
			//printf("\r\n[MSG]%3d,%3d;", mavMsgRx.seq, mavMsgRx.msgid);
			switch(mavMsgRx.msgid)
			{
				/* #0: Heartbeat */
				case MAVLINK_MSG_ID_HEARTBEAT:
					mavlink_msg_heartbeat_decode(&mavMsgRx, &mavHrt);
					printf("\r\n");
					printf(" {HRTBEAT} %d,%d,%d,%d,%d,%d", mavHrt.type, mavHrt.autopilot, mavHrt.base_mode, mavHrt.custom_mode, mavHrt.system_status, mavHrt.mavlink_version);
					break;
				
				/* #76: Command_Long */
				case MAVLINK_MSG_ID_COMMAND_LONG: 
					mavlink_msg_command_long_decode(&mavMsgRx, &mavCmdRx);
					printf("\r\n");
					printf(" {CMDLONG} %4d,%d,%d", mavCmdRx.command, (int)mavCmdRx.param1, (int)mavCmdRx.param2);		// Only use 2 params at present
					
					switch(mavCmdRx.command)
					{
						/********** Landing Gear Control **********/
						case MAV_CMD_AIRFRAME_CONFIGURATION:			//.command == 2520
							printf("\r\n> [CMD]LandingGear");
							lgPositionRecv = (int)mavCmdRx.param2;		// param2 for landing gear position
							if(lgPositionRecv!=0 && lgPositionRecv!=1)	// Param Check
							{
								printf(": Wrong Parameter!");
								break;
							}
							else	printf("(%s)", lgPositionRecv?"UP":"DOWN");

							/* Landing Gear is changing, ignore recv command */
							if(lgChangeStatus)
							{
								if(lgPositionRecv!=lgPositionCurr)	// Opposite direction, send reject message to FC
								{
									printf(": Reject!");
									sendBytes = mavlink_msg_command_ack_pack(1, 1, &mavMsgTx, MAV_CMD_AIRFRAME_CONFIGURATION, MAV_RESULT_TEMPORARILY_REJECTED, 0, 0, 1, 1);
									mavlink_msg_to_send_buffer(bufferTx, &mavMsgTx);
									HAL_UART_Transmit_IT(&huart1, bufferTx, sendBytes);
								}
								else								// Same direction, send ignore message to FC
								{
									printf(": Igrore!");
									sendBytes = mavlink_msg_command_ack_pack(1, 1, &mavMsgTx, MAV_CMD_AIRFRAME_CONFIGURATION, MAV_RESULT_IN_PROGRESS, 0, 0, 1, 1);
									mavlink_msg_to_send_buffer(bufferTx, &mavMsgTx);
									HAL_UART_Transmit_IT(&huart1, bufferTx, sendBytes);
								}

							}
							/* Landing Gear is standby, respond recv command */
							else
							{
								printf(": Respond!");
								lgPositionCurr = lgPositionRecv;
								if(lgPositionPrev != lgPositionCurr)	// Opposite direction, set change status of Landing Gear
								{
									lgChangeStatus = 1;
									lgPositionPrev = lgPositionCurr;
								}
								else									// Same direction, ignore command
								{
									printf(": Already changed!");
								}
							}

							break;//MAV_CMD_AIRFRAME_CONFIGURATION
						
						default:break;
					}//switch(mavCmdRx.command)

					break;

				/* #77: Command_Ack */
				case MAVLINK_MSG_ID_COMMAND_ACK:
					mavlink_msg_command_ack_decode(&mavMsgRx, &mavCmdAck);
					printf("\r\n");
					printf(" {CMD_ACK} %4d,%d", mavCmdAck.command, mavCmdAck.result);	// Param progess is dummy
					break;

				/* #147: Battery Status */
				case MAVLINK_MSG_ID_BATTERY_STATUS:
					mavlink_msg_battery_status_decode(&mavMsgRx, &batt2);
					printf("\r\n");
					printf(" {BATTERY} ID:%d,V:%.2f,I:%.2f,T:%.2f,R:%d", batt2.id, batt2.voltages[0]/1000.0, batt2.current_battery/100.0, batt2.temperature/100.0, batt2.battery_remaining);
					break;
				
				default:break;
			}
		}

		/* Lost communication with FC, Reset Landing Gear */
		if(msgLostCnt == 3)
		{
			msgLostCnt++;							// Prevent step into this process too many times
			printf("\r\n> Communication Lost!");
			if(lgPositionCurr == 1)					// Changing Landing Gear cost 1s approx., no need to judge change status
			{
				HAL_TIM_Base_Stop_IT(&htim6);		// Stop general changing process temp.
				printf("\r\n# LandingGear: Reset...");
				LandingGear_Reset();
				lgPositionCurr = 0, lgPositionPrev = 0;
				FLASH_SaveLGStatus(lgPositionCurr,lgChangeStatus);
				HAL_TIM_Base_Start_IT(&htim6);		// Restart general changing process
			}
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
	if(huart->Instance == USART1)
	{
		/* Receive MavLink massage */
		if(mavlink_frame_char(MAVLINK_COMM_0, aRxBuffer, &mavMsgRx, &mavSta) != MAVLINK_FRAMING_INCOMPLETE)
		{
			msgRecvFin = 1;							// Receive one mavlink message (decode sucuess)
		}	
		HAL_UART_Receive_IT(&huart1, &aRxBuffer, 1); 	// Restart usart1's IT for next receive process
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* TIM6: Landing Gear PWM Adjustment (100Hz) */
	if(htim->Instance == TIM6)
	{
		if(lgChangeStatusPrev != lgChangeStatus)	// Change status message
		{
			lgChangeStatusPrev = lgChangeStatus;
			if(lgChangeStatus)						// Changing Process Start
			{
				printf("\r\n# LandingGear: Start Changing(%s)",lgPositionCurr?"UP":"DOWN");
				sendBytes = mavlink_msg_command_ack_pack(1, 1, &mavMsgTx, MAV_CMD_AIRFRAME_CONFIGURATION, MAV_RESULT_IN_PROGRESS, 0, 0, 1, 1);
				mavlink_msg_to_send_buffer(bufferTx, &mavMsgTx);
				HAL_UART_Transmit_IT(&huart1, bufferTx, sendBytes);
				printf("\r\n# Flash: Save status (%s,%s)",lgPositionCurr?"UP":"DOWN",lgChangeStatus?"Changing":"Standby");
				FLASH_SaveLGStatus(lgPositionCurr,lgChangeStatus);
			}
			else									// Changing Process Finish
			{
				printf("\r\n# LandingGear: Stop Changing(%s)",lgPositionCurr?"UP":"DOWN");
				sendBytes = mavlink_msg_command_ack_pack(1, 1, &mavMsgTx, MAV_CMD_AIRFRAME_CONFIGURATION, MAV_RESULT_ACCEPTED, 100, 0, 1, 1);
				mavlink_msg_to_send_buffer(bufferTx, &mavMsgTx);
				HAL_UART_Transmit_IT(&huart1, bufferTx, sendBytes);
				printf("\r\n# Flash: Save status (%s,%s)",lgPositionCurr?"UP":"DOWN",lgChangeStatus?"Changing":"Standby");
				FLASH_SaveLGStatus(lgPositionCurr,lgChangeStatus);
			}
		}
		/* Landing Gear changing process */
		if(lgChangeStatus)
		{
			Relay_ON();								// Turn on relay to power on steers
			// If changing process finished, lgChangeStatus turns 0
			lgChangeStatus = LandingGear_Control(lgPositionCurr,&lgChangeProgress);
		}
		else Relay_OFF();							// Turn off relay to power off steers
	}
	
	/* TIM7: HeartBeat (1Hz) */
	if(htim->Instance == TIM7)
	{
		msgLostCnt++;
		printf("\r\n> Heartbeat");
		sendBytes = mavlink_msg_heartbeat_pack(1, 1, &mavMsgTx, MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_PX4, 81, 1016, MAV_STATE_STANDBY);
		mavlink_msg_to_send_buffer(bufferTx, &mavMsgTx);
		HAL_UART_Transmit_IT(&huart1,bufferTx,sendBytes);
		//if(msgLostCnt<10)
		{
			HAL_IWDG_Refresh(&hiwdg);				// Feed watchdog
		}
		//else
		//{
		//	printf("\r\n> Ready to reboot system");
		//}
	}
}

void IWDG_Init(void)
{
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
	hiwdg.Init.Window = 4095;
	hiwdg.Init.Reload = 2000;						// Reboot system if not refresh iwdg in 2s
	HAL_IWDG_Init(&hiwdg);
}

/**
  * @brief  Save Landing Gear status on flash
  * @param  pos position of Landing Gear
  *         cng change status of Landing Gear
  * @retval None
  */
void FLASH_SaveLGStatus(uint8_t pos, uint8_t cng)
{
	HAL_FLASH_Unlock();
	
	flashErase.TypeErase = FLASH_TYPEERASE_PAGES;
	flashErase.PageAddress = flashAddr;
	flashErase.NbPages = 1;
	HAL_FLASHEx_Erase(&flashErase, &PageError);
	
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flashAddr, (uint32_t)pos);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flashAddr+4, (uint32_t)cng);
	
	HAL_FLASH_Lock();
}

/**
  * @brief  Load Landing Gear status on flash
  * @param  pos position of Landing Gear
  *         cng change status of Landing Gear
  * @retval None
  */
void FLASH_LoadLGStatus(uint8_t* pos, uint8_t* cng)
{
	*pos = *(__IO uint32_t*)(flashAddr);
	*cng = *(__IO uint32_t*)(flashAddr+4);
}

static void Error_Handler(void)
{
  while(1)  {  }
}

/*****END OF FILE****/
