/**
  ******************************************************************************
  * File Name		: LandingGear.c
  * Description		: Landing Gear Drivers
  *
  * Version			: v0.1
  * Created	Date	: 2017.09.25
  * Revised	Date	: 2017.11.27
  *
  * Author			: Mingye Xie
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "LandingGear.h"

uint8_t lgPosition=0;			//Down: 0, Up: 1
uint8_t lgChanged=0;			//Changed: 1
uint16_t lgPulseL=PUL_LEFT_DOWN;
uint16_t lgPulseR=PUL_RIGHT_DOWN;

TIM_HandleTypeDef htim2;
TIM_OC_InitTypeDef hocl,hocr;

void LG_Init()
{
	LG_TIM_Init();
	LG_Relay_Init();
}

void LG_TIM_Init(void)
{
	//TIM2 for PWM control (100Hz)
	htim2.Instance=TIM2;
	htim2.Init.Prescaler=64;
	htim2.Init.CounterMode=TIM_COUNTERMODE_UP;
	htim2.Init.Period=10000-1;
	htim2.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_PWM_Init(&htim2);
	
	hocl.OCMode=TIM_OCMODE_PWM1;
	hocl.OCPolarity=TIM_OCPOLARITY_HIGH;
	hocl.Pulse=PUL_LEFT_DOWN;
	HAL_TIM_PWM_ConfigChannel(&htim2,&hocl,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	
	hocr.OCMode=TIM_OCMODE_PWM1;
	hocr.OCPolarity=TIM_OCPOLARITY_HIGH;
	hocr.Pulse=PUL_RIGHT_DOWN;
	HAL_TIM_PWM_ConfigChannel(&htim2,&hocr,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	__HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_TIM2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

uint8_t LG_Control(uint8_t status)
{
	uint8_t changed=1;
	
	if(status)	//Landing gear Up
	{
		lgPulseL+=PUL_LEFT_Range*PUL_SCALE;
		lgPulseR+=PUL_RIGHT_Range*PUL_SCALE;
		if(lgPulseL>PUL_LEFT_UP||lgPulseR>PUL_RIGHT_UP)
		{
			lgPulseL=PUL_LEFT_UP;
			lgPulseR=PUL_RIGHT_UP;
			changed=0; 
		}
	}
	else		//Landing gear Down
	{
		lgPulseL-=PUL_LEFT_Range*PUL_SCALE;
		lgPulseR-=PUL_RIGHT_Range*PUL_SCALE;
		if(lgPulseL<PUL_LEFT_DOWN||lgPulseR<PUL_RIGHT_DOWN)
		{
			lgPulseL=PUL_LEFT_DOWN;
			lgPulseR=PUL_RIGHT_DOWN;
			changed=0; 
		}
	}

	hocl.Pulse=lgPulseL;	
	HAL_TIM_PWM_ConfigChannel(&htim2,&hocl,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	
	hocr.Pulse=lgPulseR;	
	HAL_TIM_PWM_ConfigChannel(&htim2,&hocr,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);

	return changed;
}


void LG_Relay_Init(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;

	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Pin = GPIO_PIN_11;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/******************************END OF FILE******************************/
