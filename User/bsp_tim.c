/**
  ******************************************************************************
  * File Name       : bsp_tim.c
  * Description     : Drivers for usart (based on HAL)
  *
  * Version         : v0.3.1
  * Created Date    : 2017.10.18
  * Revised Date    : 2018.04.18
  *
  * Author          : Mingye Xie
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_tim.h"

TIM_HandleTypeDef htim2,htim6,htim7,htim15;
TIM_MasterConfigTypeDef sMasterConfig;

void TIM_Init(void)
{
    /* TIM2: System Management (1Hz) */
    htim2.Instance          = TIM2;
    htim2.Init.Prescaler    = 64000-1;
    htim2.Init.CounterMode  = TIM_COUNTERMODE_UP;
    htim2.Init.Period       = 1000-1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim2);
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

    /* TIM6: Landing Gear PWM Adjustment (50Hz) */
    #ifdef ENABLE_LANGINGGEAR
    htim6.Instance          = TIM6;
    htim6.Init.Prescaler    = 64000-1;
    htim6.Init.CounterMode  = TIM_COUNTERMODE_UP;
    htim6.Init.Period       = 20-1;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim6);
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);
    #endif //ENABLE_LANGINGGEAR

    /* TIM7: Read & Send Battery Message (40Hz) */
    #ifdef ENABLE_BATTERYMGMT
    htim7.Instance          = TIM7;
    htim7.Init.Prescaler    = 64000-1;
    htim7.Init.CounterMode  = TIM_COUNTERMODE_UP;
    htim7.Init.Period       = 25-1;
    htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim7);
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig);
    #endif //ENABLE_BATTERYMGMT

    /* TIM15: Send ESC Current Message (20Hz) */
    #ifdef ENABLE_CURRMONITOR
    htim15.Instance          = TIM15;
    htim15.Init.Prescaler    = 64000-1;
    htim15.Init.CounterMode  = TIM_COUNTERMODE_UP;
    htim15.Init.Period       = 50-1;
    htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim15);
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig);
    #endif  //ENABLE_CURRMONITOR
}


void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
    if(htim_base->Instance == TIM2)
    {
        __HAL_RCC_TIM2_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
        HAL_TIM_Base_Start_IT(&htim2);
    }

    if(htim_base->Instance == TIM6)
    {
        __HAL_RCC_TIM6_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM6_DAC1_IRQn, 1, 1);
        HAL_NVIC_EnableIRQ(TIM6_DAC1_IRQn);
        HAL_TIM_Base_Start_IT(&htim6);
    }

    if(htim_base->Instance == TIM7)
    {
        __HAL_RCC_TIM7_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM7_IRQn, 1, 1);
        HAL_NVIC_EnableIRQ(TIM7_IRQn);
        HAL_TIM_Base_Start_IT(&htim7);
    }

    if(htim_base->Instance == TIM15)
    {
        __HAL_RCC_TIM15_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM15_IRQn, 1, 1);
        HAL_NVIC_EnableIRQ(TIM15_IRQn);
        HAL_TIM_Base_Start_IT(&htim15);
    }
}

/******************************END OF FILE******************************/
