/**
  ******************************************************************************
  * File Name		: main.c
  * Description		: CompanionComputer_STM32 main program
  *
  * Version			: v0.2
  * Created	Date	: 2017.11.23
  * Revised	Date	: 2018.01.23
  *
  * Author			: Mingye Xie
  ******************************************************************************
  */

#include "main.h"

/* Basic Function */
static void SystemClock_Config(void);
static void Error_Handler(void);

/* Extern parameter */
extern UART_HandleTypeDef huart1,huart3;
extern uint8_t aRxBuffer;
extern TIM_HandleTypeDef htim2,htim6,htim7;

/* System */
uint8_t sysRunning = 0;					// Flag for system working (Receive first heartbeat from FC)
uint8_t sysError = 0;					// Counter for fatal error
uint8_t sysBattery = 0;					// Flag for battery , 0 for no problem

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
mavlink_battery_status_t mavBatt1,mavBattTx,mavBattRx;

uint16_t msgSeqPrev = 0;				// Monitor lost package number of Mavlink
uint8_t msgRecvFin = 0;					// Mavlink Receive flag
uint8_t msgLostCnt = 0;					// Mavlink Communication Lost Counter
uint8_t bufferTx[263];					// Mavlink max length is 263
uint16_t sendBytes = 0; 				// Length of Mavlink package

/* Landing Gear */
uint8_t lgPositionRecv = 0;				// Position of Landing Gear [Down: 0, Up: 1]
uint8_t lgPositionCurr = 0;
uint8_t lgPositionPrev = 0;
uint8_t lgChangeDelayCnt = 0;			// Delay for prevent too fast changing
uint8_t lgChangeStatusCurr = 0;			// Change Status of Landing Gear [Standby: 0, Changing: 1]
uint8_t lgChangeStatusPrev = 0;
uint8_t lgChangeProgress = 50;			// Change Progress of Landing Gear [0-100(%)], no use now

/* Battery Control */
extern BattMsg battA,battB;
uint8_t battNum	= 0;					// Flag for send battery msg alternately
uint8_t regSta;
uint8_t regData;
uint16_t regVal,regVal2;
void Batt_MavlinkPack(mavlink_battery_status_t* mav, BattMsg batt);

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
	
	printf("\r\n# [Init] I2C");
	I2C_Init();
	
	printf("\r\n# [Init] Battery");
	sysBattery = Batt_Init();
	
	mavBattTx.id				= 0x06;
	mavBattTx.battery_function	= MAV_BATTERY_FUNCTION_ALL;
	mavBattTx.type				= MAV_BATTERY_TYPE_LIPO;
	mavBattTx.temperature		= 2018;			// in centi-degrees celsius
	mavBattTx.voltages[0]		= 0;			// in mV
	mavBattTx.current_battery	= 1516;			// in 10mA
	mavBattTx.current_consumed	= 0;			// in mAh, -1: does not provide mAh consumption estimate
	mavBattTx.energy_consumed	= -1;			// -1: does not provide energy consumption estimate
	mavBattTx.battery_remaining	= 99;			// 0%: 0, 100%: 100
	
	printf("\r\n# [Init] LandingGear");
	LandingGear_Init();
	
	FLASH_LoadLGStatus(&lgPositionCurr,&lgChangeStatusCurr);
	printf("\r\n#   Load Flash: %s,%s",lgPositionCurr?"UP":"DOWN",lgChangeStatusCurr?"Changing":"Standby");
	/* Landing Gear not reset in last process, Reset Landing Gear */
	if(!lgPositionCurr||!lgChangeStatusCurr)
	{
		printf("\r\n#   LandingGear: Reset...");
		LandingGear_Reset();
		lgPositionCurr = 0, lgChangeStatusCurr = 0;
		FLASH_SaveLGStatus(lgPositionCurr,lgChangeStatusCurr);
	}
	
	printf("\r\n# [Init] Watchdog");
	IWDG_Init();

	printf("\r\n# [Init] Timer");
	TIM_Init();

	printf("\r\n> System: Waiting for msg from FC...");

	while(1)
	{	
		/************************* Mavlink Decode Process *************************/
		if(msgRecvFin)
		{
			msgRecvFin = 0;							// Clear Mavlink Receive flag
			msgLostCnt = 0;							// Clear Communication Lost flag
			sysError = 0;							// Current for communication error
			
			if(!sysRunning)							// Receive first mavlink msg, start system
			{
				printf("\r\n# System: Running!\r\n");
				sysRunning = 1;
			}
			else
			{
				//<Dev> Monitor lost percentage of Mavlink
				//printf("\r\n[MSG]%3d,%3d;", mavMsgRx.seq, mavMsgRx.msgid);		// Monitor Mavlink msg
				//if((mavMsgRx.seq-msgSeqPrev!=1)&&(mavMsgRx.seq+256-msgSeqPrev!=1))
				//{
				//	printf("\r\n\t\t\t\t [MSG] Lost %d msg.", (mavMsgRx.seq>msgSeqPrev)?(mavMsgRx.seq-msgSeqPrev-1):(mavMsgRx.seq+256-msgSeqPrev-1));
				//}
				//msgSeqPrev=mavMsgRx.seq;
			}
	
			switch(mavMsgRx.msgid)
			{
				/* #0: Heartbeat */
				case MAVLINK_MSG_ID_HEARTBEAT:
					mavlink_msg_heartbeat_decode(&mavMsgRx, &mavHrt);
					printf("\r\n\t\t\t\t");
					printf(" {HRTBEAT} %d,%d", mavHrt.type, mavHrt.autopilot);
					break;

				/* #76: Command_Long */
				case MAVLINK_MSG_ID_COMMAND_LONG: 
					mavlink_msg_command_long_decode(&mavMsgRx, &mavCmdRx);
					printf("\r\n\t\t\t\t");
					printf(" {CMDLONG} %4d,%d,%d", mavCmdRx.command, (int)mavCmdRx.param1, (int)mavCmdRx.param2);		// Only use 2 params at present

					switch(mavCmdRx.command)
					{
						/*************** Landing Gear Control ***************/
						case MAV_CMD_AIRFRAME_CONFIGURATION:			// 2520,0x09D8
							printf("\r\n> LandingGear");
							lgPositionRecv = (int)mavCmdRx.param2;		// param2 for landing gear position
							if(lgPositionRecv!=0 && lgPositionRecv!=1)	// Param check, ignore wrong param
							{
								break;
							}
							else	printf("(%s)", lgPositionRecv?"UP":"DOWN");

							/* Landing Gear is changing or in delay time, ignore recv command */
							if(lgChangeStatusCurr||lgChangeDelayCnt)
							{
								if(lgPositionRecv!=lgPositionCurr)	// Opposite direction, send reject message to FC
								{
									printf(": Reject!");
									sendBytes = mavlink_msg_command_ack_pack(1, 1, &mavMsgTx, MAV_CMD_AIRFRAME_CONFIGURATION, MAV_RESULT_TEMPORARILY_REJECTED, 0, 0, 1, 1);
								}
								else								// Same direction, send ignore message to FC
								{
									printf(": Igrore!");
									sendBytes = mavlink_msg_command_ack_pack(1, 1, &mavMsgTx, MAV_CMD_AIRFRAME_CONFIGURATION, MAV_RESULT_IN_PROGRESS, 0, 0, 1, 1);
								}
								mavlink_msg_to_send_buffer(bufferTx, &mavMsgTx);
								HAL_UART_Transmit_IT(&huart1, bufferTx, sendBytes);
							}
							/* Landing Gear is standby, respond recv command */
							else
							{
								lgPositionCurr = lgPositionRecv;
								if(lgPositionPrev != lgPositionCurr)	// Opposite direction, set change status of Landing Gear
								{
									printf(": Respond.");
									lgChangeStatusCurr = 1;
									lgPositionPrev = lgPositionCurr;
								}
								else									// Same direction, ignore command
								{
									printf(": Changed!");
								}
							}

							break;//MAV_CMD_AIRFRAME_CONFIGURATION

						default:break;
					}//switch(mavCmdRx.command)

					break;

				/* #77: Command_Ack */
				case MAVLINK_MSG_ID_COMMAND_ACK:
					mavlink_msg_command_ack_decode(&mavMsgRx, &mavCmdAck);
					printf("\r\n\t\t\t\t");
					printf(" {CMD_ACK} %4d,%d", mavCmdAck.command, mavCmdAck.result);	// Param progess is dummy
					break;

				/* #147: Battery Status */
				case MAVLINK_MSG_ID_BATTERY_STATUS:
					mavlink_msg_battery_status_decode(&mavMsgRx, &mavBattRx);
					printf("\r\n\t\t\t\t");
					printf(" {BATTERY} 0x%02X,%.2fC,%.2fV,%.2fA,%d%%", mavBattRx.id, mavBattRx.temperature/100.0, mavBattRx.voltages[0]/1000.0, mavBattRx.current_battery/100.0, mavBattRx.battery_remaining);
					break;
				
				default:break;
			}
		}//Mavlink Decode Process

		/************************* Lost comm from FC *************************/
		if(msgLostCnt == 2)
		{
			// Add SysError
			msgLostCnt = 0;							// For next count cycle
			sysError++;
			printf("\r\n> Comm Lost! SysError: %d",sysError);
			
			// Reset USART
			printf("\r\n# USART: Reset");
			USART_DeInit();
			USART_Init();
		}
		
		/************************* Reset System *************************/
		if(sysError == 3)
		{
			// Reset Landing Gear
			if(lgPositionCurr)						// Changing Landing Gear cost 1~2s approx., no need to judge change status
			{
				HAL_TIM_Base_Stop_IT(&htim6);		// Stop general changing process temp.
				printf("\r\n# LandingGear: Reset...");
				LandingGear_Reset();
				//******************************
				// <WIP> Need sending F3 log to FC via Mavlink
				//******************************
				lgPositionCurr = 0, lgPositionPrev = 0;
				HAL_TIM_Base_Start_IT(&htim6);		// Restart general changing process
			}
			sysError++;								// Prevent frequent reset
		}
		
		if(sysError >= 7)
		{
			printf("\r\n! System: Reset...");
			NVIC_SystemReset();
		}
			
	}//while 
}//main  
  
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
		if(mavlink_parse_char(MAVLINK_COMM_0, aRxBuffer, &mavMsgRx, &mavSta))
		{
			msgRecvFin = 1;								// Receive one mavlink message (decode sucuess)
		}	
		HAL_UART_Receive_IT(&huart1, &aRxBuffer, 1); 	// Restart usart1's IT for next receive process
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* TIM2: HeartBeat (1Hz) */
	if(htim->Instance == TIM2)
	{
		//<Dev> Temp. disable msgLostCnt for dev
		//if(sysRunning)			msgLostCnt++;		// will turn 0 if recv mavlink msg
		if(lgChangeDelayCnt)	lgChangeDelayCnt--;

		sendBytes = mavlink_msg_heartbeat_pack(1, 1, &mavMsgTx, MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_PX4, 81, 1016, MAV_STATE_STANDBY);
		mavlink_msg_to_send_buffer(bufferTx, &mavMsgTx);
		HAL_UART_Transmit_IT(&huart1,bufferTx,sendBytes);
		printf("\r\n> [Hrt] #%d",mavMsgTx.seq);
		
		HAL_IWDG_Refresh(&hiwdg);						// Feed watchdog
	}
	
	/* TIM6: Landing Gear PWM Adjustment (100Hz) */
	if(htim->Instance == TIM6)
	{
		if(lgChangeStatusPrev != lgChangeStatusCurr)	// Change status message
		{
			lgChangeStatusPrev = lgChangeStatusCurr;
			if(lgChangeStatusCurr)						// Changing Process Start
			{
				printf("\r\n# LandingGear: Start(%s)",lgPositionCurr?"UP":"DOWN");
				sendBytes = mavlink_msg_command_ack_pack(1, 1, &mavMsgTx, MAV_CMD_AIRFRAME_CONFIGURATION, MAV_RESULT_IN_PROGRESS, 0, 0, 1, 1);
			}
			else										// Changing Process Finish
			{
				printf("\r\n# LandingGear: Stop (%s)",lgPositionCurr?"UP":"DOWN");
				sendBytes = mavlink_msg_command_ack_pack(1, 1, &mavMsgTx, MAV_CMD_AIRFRAME_CONFIGURATION, MAV_RESULT_ACCEPTED, 100, 0, 1, 1);
			}
			mavlink_msg_to_send_buffer(bufferTx, &mavMsgTx);
			HAL_UART_Transmit_IT(&huart1, bufferTx, sendBytes);
			FLASH_SaveLGStatus(lgPositionCurr, lgChangeStatusCurr);
		}
		/* Landing Gear changing process */
		if(lgChangeStatusCurr)
		{
			Relay_ON();								// Turn on relay to power on steers
			// If changing process finished, lgChangeStatus turns 0
			lgChangeStatusCurr = LandingGear_Control(lgPositionCurr, &lgChangeProgress);
			lgChangeDelayCnt = 2;					// Ignore cmd for 2s (guard time)
		}
		else Relay_OFF();							// Turn off relay to power off steers
	}

	/* TIM7: Send Battery Message (1Hz) */
	if(htim->Instance == TIM7)
	{
		if(!sysBattery)								// Send battery msg when sysBattery enabled
		{	
			if(!battNum)							// Send battery msg alternately
			{
				regSta = Batt_Measure(&battA);
				if(!regSta) 
				{
					printf("\r\n> [Batt]0x%02x:0x%02x,%d,%d,%d,%d", battA.id, battA.status, battA.temperature, battA.voltage, battA.current, battA.soc);		
					Batt_MavlinkPack(&mavBattTx, battA);
				}
				else printf("\r\n> battA not found!");
			}
			else
			{
				regSta = Batt_Measure(&battB);
				if(!regSta)
				{
					printf("\r\n> [Batt]0x%02x:0x%02x,%d,%d,%d,%d", battB.id, battB.status, battB.temperature, battB.voltage, battB.current, battB.soc);		
					Batt_MavlinkPack(&mavBattTx, battB);
				}
				else printf("\r\n> battB not found!");
			}
			
			sendBytes = mavlink_msg_battery_status_pack(1, 1, &mavMsgTx, mavBattTx.id, mavBattTx.battery_function, mavBattTx.type, mavBattTx.temperature, mavBattTx.voltages, mavBattTx.current_battery, mavBattTx.current_consumed, mavBattTx.energy_consumed, mavBattTx.battery_remaining);
			mavlink_msg_to_send_buffer(bufferTx, &mavMsgTx);
			HAL_UART_Transmit_IT(&huart1, bufferTx, sendBytes);		
			
			battNum=!battNum;
		}
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
	HAL_FLASH_Unlock();								// Unlock Flash
	
	flashErase.TypeErase = FLASH_TYPEERASE_PAGES;
	flashErase.PageAddress = flashAddr;
	flashErase.NbPages = 1;
	HAL_FLASHEx_Erase(&flashErase, &PageError);		// Erase Flash
	
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flashAddr, (uint32_t)pos);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flashAddr+4, (uint32_t)cng);
	
	HAL_FLASH_Lock();								// Lock Flash
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

void Batt_MavlinkPack(mavlink_battery_status_t* mav, BattMsg batt)
{
	mav->id 				= batt.id;
	mav->battery_function	= MAV_BATTERY_FUNCTION_ALL;
	mav->type				= MAV_BATTERY_TYPE_LIPO;
	mav->temperature		= batt.temperature;
	mav->voltages[0]		= batt.voltage;
	mav->current_battery	= batt.current;
	mav->current_consumed	= batt.fullChargeCapacity-batt.remainingCapacity;
	mav->energy_consumed	= -1;
	mav->battery_remaining	= batt.soc;
}

static void Error_Handler(void)
{
  while(1)  {  }
}

/*****END OF FILE****/
