/**
  ******************************************************************************
  * File Name       : CurrMonitor.c
  * Description     : ESC Current Monitor
  *
  * Version         : v0.3
  * Created Date    : 2017.03.05
  * Revised Date    : 2018.03.05
  *
  * Author          : Mingye Xie
  ******************************************************************************
  */

#include "CurrMonitor.h"

extern ADC_HandleTypeDef hadc1;

uint32_t ADC_RawData[60]={0};
float Curr_Value[10]={0};

void CurrMonitor_Init(void)
{
    DMA_Init();
    ADC_Init();
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_RawData, 60);
}

void CurrMonitor_Send(void)
{
    PRINTLOG("\r\n [INFO] ADC_Value:");
    for(int i = 0; i < 6; i++)
    {
        Curr_Value[i] = 25.0*ADC_RawData[i]/1024 - 50; //100*x/4096-50
        PRINTLOG("%.2f,",Curr_Value[i]);
    }
//    sendCnt = mavlink_msg_stm32_f3_motor_curr_pack(1, 1, &mavMsgTx, Curr_Value);
//    Mavlink_SendMessage(&mavMsgTx, sendCnt);
}

/******************************END OF FILE******************************/
