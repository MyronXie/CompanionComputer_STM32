/**
  ******************************************************************************
  * File Name		: main.c
  * Description		: CompanionComputer_STM32 main program
  *
  * Version			: v0.2
  * Created	Date	: 2017.11.23
  * Revised	Date	: 2018.02.01
  *
  * Author			: Mingye Xie
  ******************************************************************************
  */

#include "main.h"

/* Basic Function */
static void SystemClock_Config(void);
static void Error_Handler(void);

/* Extern parameter */
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim6;

/* System */
uint8_t sysConnect = 0;					// Flag for system working (Receive first heartbeat from FC)
uint8_t sysError = 0;					// Counter for fatal error
uint8_t sysBattery = 0;					// Flag for battery , 0 for no problem
uint16_t sysTicks = 0;					// Record system running time
void System_Heartbeat(void);
void System_ErrorHandler(void);

/* Flash */
uint32_t flashParam[FLASHSIZE];

/* Serial & Mavlink */
uint8_t recvByte = 0;
uint16_t sendByteCnt = 0; 				// Length of Mavlink package
uint8_t msgLostCnt = 0;					// Mavlink Communication Lost Counter
uint16_t msgSeqPrev = 0;				// Monitor lost package number of Mavlink
mavlink_message_t mavMsgTx,mavMsgRx;
mavlink_heartbeat_t mavHrt;
mavlink_status_t mavSta;
mavlink_command_long_t mavCmdRx;
mavlink_command_ack_t mavCmdAck;
mavlink_battery_status_t mavBattTx,mavBattRx;
mavlink_stm32_f3_command_t mavF3Cmd;
void Mavlink_Decode(mavlink_message_t* msg);
void Mavlink_SendMessage(mavlink_message_t* msg, uint16_t length);

/* Landing Gear */
uint8_t lgPositionRecv = 0;				// Position of Landing Gear [Down: 0, Up: 1]
uint8_t lgPositionCurr = 0;
uint8_t lgPositionPrev = 0;
uint8_t lgChangeDelayCnt = 0;			// Delay for prevent too fast changing
uint8_t lgChangeStatusCurr = 0;			// Change Status of Landing Gear [Standby: 0, Changing: 1]
uint8_t lgChangeStatusPrev = 0;
uint8_t lgChangeProgress = 50;			// Change Progress of Landing Gear [0-100(%)], not used now
void LandingGear_Init(void);
void LandingGear_Control(void);
void LandingGear_Adjustment(void);

/* Battery Mgmt */
extern BattMsg battA,battB,battO;
BattMsg* battX;
uint8_t battCycleCnt = 0;				// Counter for dispatch command for battmgmt system
uint8_t battAutoOff = 0;				// Flag for enable Auto Power Off Function
void Battery_Management(void);
void Batt_MavlinkPack(mavlink_battery_status_t* mav);

/* Log list */
char F3LOG[100]={""};
char LOG_0x01[15]={"System Reboot"};
char LOG_0x02[15]={"Serial Reset"};
char LOG_0x11[20]={"Battery Offboard"};
char LOG_0x12[15]={"Vdiff>100mV"};
char LOG_0x16[25]={"Power Off Fail"};
char LOG_0x17[25]={"Power Off Success"};
char LOG_0x21[25]={"Landing Gear Auto Reset"};

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
	sysBattery = Batt_Init();
	
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
				sysError = 0;							// Current for communication error
				
				if(!sysConnect)							// Receive first mavlink msg, start system
				{
					printf("\r\n [SYS]  Connected with FMU");
					sysConnect = 1;					
				}
				else
				{
					// Monitor lost package number of Mavlink
					if((mavMsgRx.seq-msgSeqPrev!=1)&&(mavMsgRx.seq+256-msgSeqPrev!=1))
					{
						printf("\r\n [FMU]  LOST: %d", (mavMsgRx.seq>msgSeqPrev)?(mavMsgRx.seq-msgSeqPrev-1):(mavMsgRx.seq+256-msgSeqPrev-1));
					}
				}

				msgSeqPrev=mavMsgRx.seq;
				
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
	if(htim->Instance == TIM2)		// TIM2: HeartBeat (1Hz)
	{
		System_Heartbeat();
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
		Battery_Management();
	}
}


void System_Heartbeat(void)
{	
	printf("\r\n\r\n [INFO] Hrt#%d",++sysTicks);			// Record running time
	sendByteCnt = mavlink_msg_heartbeat_pack(1, 1, &mavMsgTx, MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_PX4, 81, 1016, MAV_STATE_STANDBY);
	Mavlink_SendMessage(&mavMsgTx, sendByteCnt);
	
	// <Dev> for Monitor
	if(!(sysTicks%10))
	{		
		sprintf(F3LOG,"Heartbeat #%d",sysTicks);
		sendByteCnt = mavlink_msg_stm32_f3_command_pack(1, 1, &mavMsgTx, 0x00, F3LOG);
		Mavlink_SendMessage(&mavMsgTx, sendByteCnt);
	}	
}


void System_ErrorHandler(void)
{
	#ifndef INGORE_LOSTCOMM
	if(sysConnect)		msgLostCnt++;			// msgLostCnt will reset if receive mavlink msg
	#endif

	// Lost connect with from FMU
	if(msgLostCnt>=3)
	{
		printf("\r\n [ERR]  Connect Lost: %d",msgLostCnt);
		
		if(!(msgLostCnt%3)) 
		{
			sysError++;
			printf("\r\n [ACT]  USART1: Reset");
			USART_ReInit();						// Reset USART
			
			sendByteCnt = mavlink_msg_stm32_f3_command_pack(1, 1, &mavMsgTx, 0x02, LOG_0x02);
			Mavlink_SendMessage(&mavMsgTx, sendByteCnt);
		}
	}
	
	#ifdef ENABLE_LANGINGGEAR
	// Reset Landing Gear
	if(sysError == 2)
	{
		if(lgPositionCurr)						// Changing Landing Gear cost 1~2s approx., no need to judge change status
		{
			printf("\r\n [ACT]  LandingGear: Reset...");
			HAL_TIM_Base_Stop_IT(&htim6);		// Stop general changing process temp.
			LG_Reset();
			lgPositionCurr = 0, lgChangeStatusCurr = 0;
			flashParam[0] = lgPositionCurr;		flashParam[1] = lgChangeStatusCurr;
			FLASH_SaveParam(flashParam,2);
			HAL_TIM_Base_Start_IT(&htim6);		// Restart general changing process

			// Send log to FC
			sendByteCnt = mavlink_msg_stm32_f3_command_pack(1, 1, &mavMsgTx, 0x21, LOG_0x21);
			Mavlink_SendMessage(&mavMsgTx, sendByteCnt);
		}
	}
	#endif //ENABLE_LANGINGGEAR
	
	// Reset System
	if(sysError >= 4)
	{
		printf("\r\n [ACT]  System: Reset...");
		sendByteCnt = mavlink_msg_stm32_f3_command_pack(1, 1, &mavMsgTx, 0x01, LOG_0x01);
		Mavlink_SendMessage(&mavMsgTx, sendByteCnt);
		NVIC_SystemReset();
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
					LandingGear_Control();
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
			mavlink_msg_stm32_f3_command_decode(msg, &mavF3Cmd);
			printf("\r\n [FMU]  #500: 0x%02X,%s", mavF3Cmd.command, mavF3Cmd.f3_log);
			break;
		
		default:break;
	}
}


void Mavlink_SendMessage(mavlink_message_t* msg, uint16_t length)
{
	uint8_t buffer[263];								// Mavlink max length is 263
	mavlink_msg_to_send_buffer(buffer, msg);
	HAL_UART_Transmit(&huart1, buffer, length, 1);	
	//HAL_UART_Transmit_IT(&huart1, buffer, length);	
}

void LandingGear_Init(void)
{
	LG_Init();											// Low-layer init
	
	FLASH_LoadParam(flashParam,2);
	lgPositionCurr = flashParam[0];		lgChangeStatusCurr = flashParam[1];
	printf(": %s,%s",lgPositionCurr?"UP":"DOWN",lgChangeStatusCurr?"Changing":"Standby");
	/* Landing Gear not reset in last process, Reset Landing Gear */
	if(lgPositionCurr||lgChangeStatusCurr)
	{
		printf("\r\n [ACT]  Reset Landing Gear");
		LG_Reset();
		lgPositionCurr = 0, lgChangeStatusCurr = 0;
		flashParam[0] = lgPositionCurr;		flashParam[1] = lgChangeStatusCurr;
		FLASH_SaveParam(flashParam,2);
	}
}

void LandingGear_Control(void)
{
	lgPositionRecv = (int)mavCmdRx.param2;		// .param2 for landing gear position
	if(lgPositionRecv==0||lgPositionRecv==1)	// Param check
	{
		printf("\r\n [INFO] Landing Gear: %s", lgPositionRecv?"UP":"DOWN");
		/* Landing Gear is changing or in delay time, ignore recv command */
		if(lgChangeStatusCurr||lgChangeDelayCnt)
		{
			if(lgPositionRecv!=lgPositionCurr)	// Opposite direction, send reject message to FMU
			{
				printf(", Reject");
				sendByteCnt = mavlink_msg_command_ack_pack(1, 1, &mavMsgTx, MAV_CMD_AIRFRAME_CONFIGURATION, MAV_RESULT_TEMPORARILY_REJECTED, 0, 0, 1, 1);
			}
			else								// Same direction, send ignore message to FMU
			{
				printf(", Igrore");
				sendByteCnt = mavlink_msg_command_ack_pack(1, 1, &mavMsgTx, MAV_CMD_AIRFRAME_CONFIGURATION, MAV_RESULT_IN_PROGRESS, 0, 0, 1, 1);
			}
			Mavlink_SendMessage(&mavMsgTx, sendByteCnt);
		}
		/* Landing Gear is standby, respond recv command */
		else
		{
			lgPositionCurr = lgPositionRecv;
			if(lgPositionPrev != lgPositionCurr)	// Opposite direction, set change status of Landing Gear
			{
				printf(", Respond");
				lgChangeStatusCurr = 1;
				lgPositionPrev = lgPositionCurr;
			}
			else									// Same direction, ignore command
			{
				printf(", Igrore");
			}
		}						
	}
}

void LandingGear_Adjustment(void)
{
	// Decrease lgChangeDelayCnt
	if(lgChangeDelayCnt)	lgChangeDelayCnt--;

	if(lgChangeStatusPrev != lgChangeStatusCurr)	// Change status message
	{
		lgChangeStatusPrev = lgChangeStatusCurr;
		if(lgChangeStatusCurr)						// Changing Process Start
		{
			printf("\r\n [ACT]  Landing Gear: Start(%s)",lgPositionCurr?"UP":"DOWN");
			sendByteCnt = mavlink_msg_command_ack_pack(1, 1, &mavMsgTx, MAV_CMD_AIRFRAME_CONFIGURATION, MAV_RESULT_IN_PROGRESS, 0, 0, 1, 1);
		}
		else										// Changing Process Finish
		{
			printf("\r\n [ACT]  Landing Gear: Stop (%s)",lgPositionCurr?"UP":"DOWN");
			sendByteCnt = mavlink_msg_command_ack_pack(1, 1, &mavMsgTx, MAV_CMD_AIRFRAME_CONFIGURATION, MAV_RESULT_ACCEPTED, 100, 0, 1, 1);
		}
		Mavlink_SendMessage(&mavMsgTx, sendByteCnt);
		flashParam[0] = lgPositionCurr;		flashParam[1] = lgChangeStatusCurr;
		FLASH_SaveParam(flashParam,2);
	}
	/* Landing Gear changing process */
	if(lgChangeStatusCurr)
	{
		Relay_ON();								// Turn on relay to power on steers
		// If changing process finished, lgChangeStatus turns 0
		lgChangeStatusCurr = LG_Control(lgPositionCurr, &lgChangeProgress);
		lgChangeDelayCnt = 200;					// Ignore cmd for 2s (guard time)
	}
	else Relay_OFF();							// Turn off relay to power off steers
}

// battCycleCnt(0x00~0x27)
// 
// Measure : 000X 0NNN
// Send    : 000X 1NNN
//  X=0 battA, X=1 battB
//  N=0~7 type
// Judge   : 0010 0NNN
//  N=0~7 judge type
void Battery_Management(void)
{
	/********** Measure & Send Process **********/
	if(!(battCycleCnt&BATT_SYS_JUDGE))
	{
		// Select Battery
		if(!(battCycleCnt&BATT_SYS_BATTB))		battX = &battA;
		else									battX = &battB;
		
		// Measure Process
		if(!(battCycleCnt&BATT_SYS_SEND))
		{
			if(battX->status&BATT_INUSE) Batt_Measure(battX, battCycleCnt&BATT_SYS_MASK_CMD);
		}
		// Send Process
		else
		{
			switch(battCycleCnt&BATT_SYS_MASK_CMD)
			{
				// Printf data
				case 0x01:
					if(battX->status&BATT_INUSE)
					{
						if(battX->status&BATT_ONBOARD)
						{
							printf("\r\n [INFO] Batt_0x%02x:0x%02x%02x,%d,%d,%d,%d,%d,%d,%d", battX->id, battX->status, battX->fet, battX->temperature, battX->voltage, battX->current, battX->soc, battX->remainingCapacity, battX->fullChargeCapacity, battX->designCapacity);
							battX->lostCnt = 0;
						}
						else
						{
							battX->lostCnt++;
							if(battX->lostCnt<=3) printf("\r\n [INFO] Batt_0x%02x:Lost#%d!", battX->id, battX->lostCnt);
						}
					}
					break;

				default: break;
			}
		}
	}
	/********** Judge Process **********/
	else
	{
		switch(battCycleCnt&BATT_SYS_MASK_CMD)
		{

			// Pack & Send message
			case 0x01:
				#ifdef SINGLE_BATTERY
				if(!(battX->status&BATT_INUSE))
				{
					if(battA.status&BATT_INUSE)	battX = &battA;
					if(battB.status&BATT_INUSE)	battX = &battB;
				}
				#endif
				Batt_MavlinkPack(&mavBattTx);
				sendByteCnt = mavlink_msg_battery_status_pack(1, 1, &mavMsgTx, mavBattTx.id, mavBattTx.battery_function, mavBattTx.type, mavBattTx.temperature, mavBattTx.voltages, mavBattTx.current_battery, mavBattTx.current_consumed, mavBattTx.energy_consumed, mavBattTx.battery_remaining);
				Mavlink_SendMessage(&mavMsgTx, sendByteCnt);
				break;

			// Battery Link Lost
			case 0x02:
				if((battA.lostCnt>=3)||(battB.lostCnt>=3))
				{
					printf("\r\n [ERR]  Batt: Connect lost");
					sysBattery = 0x11;
					sendByteCnt = mavlink_msg_stm32_f3_command_pack(1, 1, &mavMsgTx, 0x11, LOG_0x11);
					Mavlink_SendMessage(&mavMsgTx, sendByteCnt);
				}
				break;
			
			#ifndef SINGLE_BATTERY
			#ifdef AUTO_POWEROFF
			// Judge Auto power off
			case 0x03:
				if(!battAutoOff)
				{
					if(((battA.fet&PWR_ON)&&(!(battB.fet&PWR_ON)))||((battB.fet&PWR_ON)&&(!(battA.fet&PWR_ON))))
					{
						battAutoOff = 1;	
					}
				}
				break;
				
			// Auto Power Off Process
			case 0x04:
				if(battAutoOff)
				{
					// Attempt every 2s
					if((battAutoOff+1)%2)
					{
						printf("\r\n [ACT]  Batt: Auto-Off Attempt#%d",(battAutoOff+1)/2);
						if(battA.fet&PWR_ON) Batt_WriteWord(battA.id, BATT_PowerControl, BATT_POWEROFF);
						if(battB.fet&PWR_ON) Batt_WriteWord(battB.id, BATT_PowerControl, BATT_POWEROFF);
					
						Batt_ReadFET(&battA);
						Batt_ReadFET(&battB);
						printf(": A-0x%02x, B-0x%02x",battA.fet,battB.fet);
					}
					
					// All Batteries have powered off
					if(!((battA.fet&PWR_ON)||(battB.fet&PWR_ON)))
					{
						printf("\r\n [INFO] Batt: Auto Power Off Success");
						battAutoOff = 0;
						battA.status &= ~BATT_INUSE;
						battB.status &= ~BATT_INUSE;
						sysBattery = 0x17;
						sendByteCnt = mavlink_msg_stm32_f3_command_pack(1, 1, &mavMsgTx, 0x17, LOG_0x17);
						Mavlink_SendMessage(&mavMsgTx, sendByteCnt);
					}
					else if(++battAutoOff>=7)
					{
						printf("\r\n [ERR]  Batt: Auto Power Off Fail");
						battAutoOff = 0;
						sysBattery = 0x16;
						sendByteCnt = mavlink_msg_stm32_f3_command_pack(1, 1, &mavMsgTx, 0x16, LOG_0x16);
						Mavlink_SendMessage(&mavMsgTx, sendByteCnt);
					}
				}
				break;
			#endif //AUTO_POWEROFF
			#endif //SINGLE_BATTERY
		}
	}		
	
	// Increase battCycleCnt
	battCycleCnt = (battCycleCnt+1)%40;
}


void Batt_MavlinkPack(mavlink_battery_status_t* mav)
{
	#ifndef SINGLE_BATTERY
	mav->id 				= 0x36;
	mav->battery_function	= ((battA.status<<4)&0xF0)+(battB.status&0x0F);	// Redefine this param
	mav->type				= sysBattery;									// Redefine this param
	mav->temperature		= (battA.temperature + battB.temperature)/2;	// in centi-degrees celsius
	mav->voltages[0]		= (battA.voltage + battB.voltage)/2;			// in mV
	mav->current_battery	= battA.current + battB.current;				// in 10mA
	mav->current_consumed	= (battA.fullChargeCapacity - battA.remainingCapacity)+(battB.fullChargeCapacity - battB.remainingCapacity);	// in mAh
	mav->energy_consumed	= -1;											// -1: does not provide
	mav->battery_remaining	= (battA.soc + battB.soc)/2;					// 0%: 0, 100%: 100

// <Dev> Test Data	
//	mav->id 				= 0x36;
//	mav->battery_function	= 4;	// Redefine this param
//	mav->type				= 3;									// Redefine this param
//	mav->temperature		= 1234;	// in centi-degrees celsius
//	mav->voltages[0]		= 23456;			// in mV
//	mav->current_battery	= 987;				// in 10mA
//	mav->current_consumed	= 345;	// in mAh
//	mav->energy_consumed	= -1;											// -1: does not provide
//	mav->battery_remaining	= 89;						// 0%: 0, 100%: 100
	#else
	mav->id 				= battX->id;
	mav->battery_function	= battX->status;									// Redefine this param
	mav->type				= sysBattery;
	mav->temperature		= battX->temperature;
	mav->voltages[0]		= battX->voltage;
	mav->current_battery	= battX->current;
	mav->current_consumed	= battX->fullChargeCapacity - battX->remainingCapacity;
	mav->energy_consumed	= -1;
	mav->battery_remaining	= battX->soc;
	#endif
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
