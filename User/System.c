/**
  ******************************************************************************
  * File Name		: System.c
  * Description		: System param for CC_STM32
  *
  * Version			: v0.2
  * Created	Date	: 2018.02.02
  * Revised	Date	: 2018.02.02
  *
  * Author			: Mingye Xie
  ******************************************************************************
  */

#include "System.h"

extern UART_HandleTypeDef huart1;

uint8_t sysConnect = 0;					// Flag for system working (Receive first heartbeat from FC)
uint8_t sysWarning = 0;					// Counter for fatal error
uint8_t sysStatus = 0;					// Flag for battery , 0 for no problem
uint8_t sysReport = 0;					// Flag for report error msg
uint16_t sysTicks = 0;					// Record system running time

uint8_t msgLostCnt = 0;					// Mavlink Communication Lost Counter

mavlink_message_t mavMsgTx;
uint16_t sendByteCnt = 0;

char* logList[64]={
	LOG_00,LOG_01,LOG_02,"","","","","","","","","","","","","",
	LOG_10,LOG_11,LOG_12,LOG_13,LOG_14,LOG_15,LOG_16,"","","","","","","","","",
	LOG_20,LOG_21};

extern uint8_t LandingGear_Reset(void);

void System_Heartbeat(void)
{	
	printf("\r\n\r\n [HRT]  #%d",++sysTicks);			// Record running time
	sendByteCnt = mavlink_msg_heartbeat_pack(1, 1, &mavMsgTx, MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_PX4, 81, 1016, MAV_STATE_STANDBY);
	Mavlink_SendMessage(&mavMsgTx, sendByteCnt);
}

void System_StatusReporter(void)
{
	if(sysConnect&&sysStatus&&sysReport)			// Must report after connected with FMU, otherwise it's dummy
	{
		sysReport = 0;
		//Mavlink_SendLog(sysStatus, logList[sysStatus]);
		sendByteCnt = mavlink_msg_stm32_f3_command_pack(1, 1, &mavMsgTx, sysStatus, logList[sysStatus]);
		Mavlink_SendMessage(&mavMsgTx, sendByteCnt);
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
	if(sysConnect)		msgLostCnt++;			// msgLostCnt will reset if receive mavlink msg
	#endif

	// Lost connect with from FMU
	if(msgLostCnt>=2)
	{
		printf("\r\n [ERR]  Connect Lost: %d",msgLostCnt);
		
		if(!(msgLostCnt%3)) sysWarning++;
		
		if(msgLostCnt==3||msgLostCnt==6||msgLostCnt==10) 
		{
			printf("\r\n [ACT]  USART1: Reset");
			USART_ReInit();						// Reset USART
			
			//Mavlink_SendLog(ERR_SYS_SERIAL, logList[ERR_SYS_SERIAL]);
			sendByteCnt = mavlink_msg_stm32_f3_command_pack(1, 1, &mavMsgTx, ERR_SYS_SERIAL, logList[ERR_SYS_SERIAL]);
			Mavlink_SendMessage(&mavMsgTx, sendByteCnt);
		}
	}
	
	#ifdef ENABLE_LANGINGGEAR
	// Reset Landing Gear
	if(sysWarning == 2)
	{
		sysStatus = LandingGear_Reset();
		if(sysStatus) sysReport = 1;
	}
	#endif //ENABLE_LANGINGGEAR
	
	// Reset System
	if(sysWarning >= 4)
	{
		printf("\r\n [ACT]  System: Reset...");
		//Mavlink_SendLog(ERR_SYS_GENERAL, logList[ERR_SYS_GENERAL]);
		sendByteCnt = mavlink_msg_stm32_f3_command_pack(1, 1, &mavMsgTx, ERR_SYS_GENERAL, logList[ERR_SYS_GENERAL]);
		Mavlink_SendMessage(&mavMsgTx, sendByteCnt);
		NVIC_SystemReset();
	}
}


void Mavlink_SendMessage(mavlink_message_t* msg, uint16_t length)
{
	uint8_t buffer[263];								// Mavlink max length is 263
	mavlink_msg_to_send_buffer(buffer, msg);
	HAL_UART_Transmit(&huart1, buffer, length, 1);	
	//HAL_UART_Transmit_IT(&huart1, buffer, length);	
}


/*****END OF FILE****/
