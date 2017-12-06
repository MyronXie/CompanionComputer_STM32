/**
  ******************************************************************************
  * File Name		: main.c
  * Description		: CompanionComputer_STM32 main program
  *
  * Version			: v0.1
  * Created	Date	: 2017.11.23
  * Revised	Date	: 2017.12.06
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
uint8_t bufferTx[263];	//Mavlink max length is 263.

static void SystemClock_Config(void);
static void Error_Handler(void);

uint8_t lgPos=0;			//Down: 0, Up: 1
uint8_t lgPosPrev=0;
uint8_t lgChanged=0;		//Changed: 1
uint8_t lgChangedPrev=0;

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

	TIM_Init();
	USART_Init();

	LG_Init();
	Batt_Init();

	HAL_UART_Receive_IT(&huart1,&aRxBuffer,1);
	HAL_UART_Receive_IT(&huart3,&aRxBuffer,1);	

	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);

	batt1.id				= 0x16;
	batt1.current_consumed	= 100;
	batt1.temperature		= 2760;
	batt1.voltages[0]		= 24360;
	batt1.current_battery	= -1602;
	batt1.battery_function	= MAV_BATTERY_FUNCTION_ALL;
	batt1.type				= MAV_BATTERY_TYPE_LIPO;
	batt1.battery_remaining	= 99;
	
	while(1)
	{	
		/* Mavlink Decode Process */
		if(msgOK)
		{
			msgOK=0;		//Clear flag
			printf("\r\n[MSG]%3d,%3d,%d,%d,%3d;", msgRx.len, msgRx.seq, msgRx.sysid, msgRx.compid, msgRx.msgid);
			switch(msgRx.msgid)
			{
				/* #0:Heartbeat */
				case MAVLINK_MSG_ID_HEARTBEAT:
					mavlink_msg_heartbeat_decode(&msgRx,&hrtRx);
					printf(" {HRTBEAT} Type:%d",hrtRx.type);
					break;
				
				/* #76:Command_Long */
				case MAVLINK_MSG_ID_COMMAND_LONG: 
					mavlink_msg_command_long_decode(&msgRx, &cmdRx);
					printf(" {CMDLONG} Cmd:%d,P1:%d,P2:%d",cmdRx.command,(int)cmdRx.param1,(int)cmdRx.param2);
					
					switch(cmdRx.command)
					{
						/* Landing Gear Control */
						case MAV_CMD_AIRFRAME_CONFIGURATION:
							printf("\r\n[CMD]Landing Gear ");	
							lgPos=(int)cmdRx.param2;
							printf("%s",lgPos?"UP":"DOWN");
							if(lgPosPrev!=lgPos)					//Ignore same direction command
							{
								lgChanged=1;
								lgPosPrev=lgPos;
							}
							break;
						
						default:break;
					}
					break;
				
				/* #147:Battery Status */
				case MAVLINK_MSG_ID_BATTERY_STATUS: //for test, noused now
					mavlink_msg_battery_status_decode(&msgRx,&batt2);
					printf(" {BATTERY} ID:%d,V:%.2f,I:%.2f,T:%.2f,R:%d",batt2.id,batt2.voltages[0]/1000.0,batt2.current_battery/100.0,batt2.temperature/100.0,batt2.battery_remaining); 
					break;
				
				default:break;
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
	if(huart->Instance==USART1)
	{
		if(mavlink_frame_char(chan, aRxBuffer, &msgRx, &sta) != MAVLINK_FRAMING_INCOMPLETE)
		{
			msgOK =1;	//Receive MavLink massage
		}	
		HAL_UART_Receive_IT(&huart1,&aRxBuffer,1); 	//Restart usart1's IT for next receive process
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* TIM6:Landing Gear PWM Adjustment (100Hz) */
	if(htim->Instance==TIM6)
	{
		if(lgChangedPrev!=lgChanged)	//Status changed msg
		{
			lgChangedPrev=lgChanged;
			if(lgChanged) 	printf("\r\n[CMD]TIM6:Start(%s)",lgPos?"UP":"DOWN");
			else 			printf("\r\n[CMD]TIM6:Stop(%s)",lgPos?"UP":"DOWN");
		}
		
		if(lgChanged)						//Changing process
		{
			Relay_ON();						//Turn on relay to power on steers
			lgChanged=LG_Control(lgPos);	//If changing process finished, lgChanged turns 0
		}
		else Relay_OFF();					//Turn off relay to power off steers
	}
	
	/* TIM7:Send Battery data to Pixhawk (1Hz) */
	if(htim->Instance==TIM7)
	{

//		Batt_ReadWord(BATT_A, BATT_Voltage, &regData);			batt1.voltages[0]		= regData;
//		Batt_ReadWord(BATT_A, BATT_Current, &regData);  		batt1.current_battery	= regData;
//		Batt_ReadWord(BATT_A, BATT_Temperature, &regData);		batt1.temperature		= regData;
//		Batt_ReadWord(BATT_A, BATT_RelativeSOC, &regData);		batt1.battery_remaining	= regData;
		
//		mavlink_msg_battery_status_encode(1, 1, &msgTx, &batt1);
//		mavlink_msg_to_send_buffer(bufferTx, &msgTx);
//		HAL_UART_Transmit(&huart1,bufferTx,44,1000);
//		printf("\r\n[TIM7]Send battery Message.");
	}
}

static void Error_Handler(void)
{
  while(1)  {  }
}

/*****END OF FILE****/
