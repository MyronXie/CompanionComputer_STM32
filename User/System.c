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

void Mavlink_SendMessage(mavlink_message_t* msg, uint16_t length)
{
	uint8_t buffer[263];								// Mavlink max length is 263
	mavlink_msg_to_send_buffer(buffer, msg);
	HAL_UART_Transmit(&huart1, buffer, length, 1);	
	//HAL_UART_Transmit_IT(&huart1, buffer, length);	
}


/*****END OF FILE****/
