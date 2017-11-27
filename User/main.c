/**
  ******************************************************************************
  * File Name		: main.c
  * Description		: CompanionComputer_STM32 main program
  *
  * Version			: v0.1
  * Created	Date	: 2017.11.23
  * Revised	Date	: 2017.11.27
  *
  * Author			: Mingye Xie
  ******************************************************************************
  */

#include "main.h"

extern UART_HandleTypeDef huart1,huart3;
extern uint8_t aRxBuffer;
extern TIM_HandleTypeDef htim6,htim7;

mavlink_message_t msgRx,msgTx;
mavlink_heartbeat_t hrtRx;
mavlink_status_t sta;
mavlink_command_long_t cmdRx,cmdTx;
mavlink_channel_t chan = MAVLINK_COMM_0;
mavlink_battery_status_t batt1,batt2;
int msgOK = 0;
uint8_t bufferTx[255];

static void SystemClock_Config(void);
static void Error_Handler(void);

extern uint8_t lgPosition;			//Down: 0, Up: 1
extern uint8_t lgChanged;			//Changed: 1
uint8_t lgChangedPrev=0;			//Changed: 1

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	
	HAL_Init();
	SystemClock_Config();
	
	TIM_Init();
	USART_Init();
	
	LG_Init();
	Batt_Init();
	
	HAL_UART_Receive_IT(&huart1,&aRxBuffer,1);
	HAL_UART_Receive_IT(&huart3,&aRxBuffer,1);	

	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	
	batt1.current_consumed=100;
	batt1.temperature=2760;
	batt1.voltages[0]=24360;
	batt1.current_battery=-1602;
	batt1.id=0x16;
	batt1.battery_function=MAV_BATTERY_FUNCTION_ALL;
	batt1.type=MAV_BATTERY_TYPE_LIPO;
	batt1.battery_remaining=99;

	
	while(1)
	{	
		if(msgOK)
		{
			msgOK=0;
			printf("\r\n[MSG] ID:%d,Seq:%d,Comp:%d,Sys:%d,Len:%d", msgRx.msgid, msgRx.seq, msgRx.compid, msgRx.sysid, msgRx.len);
			switch(msgRx.msgid)
			{
				case MAVLINK_MSG_ID_HEARTBEAT:
					mavlink_msg_heartbeat_decode(&msgRx,&hrtRx);
					printf(" (HEARTBEAT)Type:%d",hrtRx.type);
					break;
				
				case MAVLINK_MSG_ID_COMMAND_LONG: 
					mavlink_msg_command_long_decode(&msgRx, &cmdRx);
					printf(" (CMD_LONG)Cmd:%d,P1:%d,P2:%d",cmdRx.command,(int)cmdRx.param1,(int)cmdRx.param2);
					if(cmdRx.command==MAV_CMD_AIRFRAME_CONFIGURATION)	//Landing Gear Control Command
					{
						lgChanged=1;
						lgPosition=(int)cmdRx.param2;
						printf("\r\n[INFO]Receive Landing Gear Command.");
					}						
					break;
				
				case MAVLINK_MSG_ID_BATTERY_STATUS: //for test, noused
					mavlink_msg_battery_status_decode(&msgRx,&batt2);
					printf(" (BATTERY)ID:%d,V:%.2f,I:%.2f,T:%.2f,R:%d",batt2.id,batt2.voltages[0]/1000.0,batt2.current_battery/100.0,batt2.temperature/100.0,batt2.battery_remaining); 
					break;
				
				default:
					printf(" (ERROR) Can't decode message.");
					break;
			}
					
		}
	
	}

}  
  
//	cmdTx.param1=0;
//	cmdTx.param2=1;
//	cmdTx.command=2520;
//			mavlink_msg_command_long_encode(1, 1, &msgTx, &cmdTx);
//			mavlink_msg_to_send_buffer(bufferTx, &msgTx);
//			HAL_UART_Transmit(&huart1,bufferTx,41,1000);

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

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
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
		if(mavlink_frame_char(chan, aRxBuffer, &msgRx, &sta) != MAVLINK_FRAMING_INCOMPLETE)
		{
			msgOK =1;	//mavlink receive success
		}	
		HAL_UART_Receive_IT(&huart1,&aRxBuffer,1); //restart usart1 IT for next receive process
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//100Hz, Landing Gear PWM Adjustment
	if(htim->Instance==TIM6)
	{
		if(lgChangedPrev!=lgChanged)
		{
			lgChangedPrev=lgChanged;
			if(lgChanged) printf("\r\n[TIM6]Start PWM adjustment (%d)",lgPosition);
			else printf("\r\n[TIM6]Stop PWM adjustment (%d)",lgPosition);
		}
		
		if(lgChanged)
		{
			Relay_ON();
			lgChanged=LG_Control(lgPosition);
		}
		else Relay_OFF();
	}
	
	//1Hz, Send Battery data to Pixhawk
	if(htim->Instance==TIM7)
	{
		mavlink_msg_battery_status_encode(1, 1, &msgTx, &batt1);
		mavlink_msg_to_send_buffer(bufferTx, &msgTx);
		HAL_UART_Transmit(&huart1,bufferTx,44,1000);
		printf("\r\n[TIM7]Send battery Message.");
	}
}

static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}

/*****END OF FILE****/
