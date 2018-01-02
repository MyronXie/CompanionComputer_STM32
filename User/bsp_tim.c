/**
  ******************************************************************************
  * File Name		: bsp_tim.c
  * Description		: Drivers for usart (based on HAL)
  *
  * Version			: v0.1
  * Created	Date	: 2017.10.18
  * Revised	Date	: 2017.12.26
  *
  * Author			: Mingye Xie
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_tim.h"

TIM_HandleTypeDef htim6,htim7;
TIM_MasterConfigTypeDef sMasterConfig;

void TIM_Init(void)
{
	/* TIM6: Landing Gear PWM Adjustment (100Hz) */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 64000-1;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 10-1;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_Base_Init(&htim6);
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);

	/* TIM7: HeartBeat (1Hz) */
	htim7.Instance = TIM7;
	htim7.Init.Prescaler = 64000-1;
	htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim7.Init.Period = 1000-1;
	htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_Base_Init(&htim7);
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
    __HAL_RCC_TIM6_CLK_ENABLE();
	HAL_NVIC_SetPriority(TIM6_DAC1_IRQn, 0, 3);
    HAL_NVIC_EnableIRQ(TIM6_DAC1_IRQn);

    __HAL_RCC_TIM7_CLK_ENABLE();
	HAL_NVIC_SetPriority(TIM7_DAC2_IRQn, 0, 3);
    HAL_NVIC_EnableIRQ(TIM7_DAC2_IRQn);
}

/******************************END OF FILE******************************/
