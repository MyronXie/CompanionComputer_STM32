/**
  ******************************************************************************
  * File Name       : CurrMonitor.c
  * Description     : ESC Current Monitor
  *
  * Version         : v0.3
  * Created Date    : 2017.03.05
  * Revised Date    : 2018.04.11
  *
  * Author          : Mingye Xie
  ******************************************************************************
  */

#include "CurrMonitor.h"

extern ADC_HandleTypeDef hadc1;

uint32_t ADC_RawData[ESC_ARRAY_SIZE]={0};
uint32_t ADC_AvgData[ESC_NUM]={0};
float Curr_Value[10]={0};

void CurrMonitor_Init(void)
{
    DMA_Init();
    ADC_Init();
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_RawData, ESC_ARRAY_SIZE);
}

void CurrMonitor_Capture(void)
{
    for(int i = 0; i < ESC_NUM; i++)
    {
        ADC_AvgData[i] = 0;

        for(int j = 0; j < ESC_SAMPLE_TIMES; j++)
            ADC_AvgData[i] += ADC_RawData[ESC_NUM*j+i];

        ADC_AvgData[i] = ADC_AvgData[i]/ESC_SAMPLE_TIMES;
        Curr_Value[i] = 25.0*ADC_AvgData[i]/1024 - 50; //100*x/4096-50
    }
}

void CurrMonitor_Send(void)
{
    PRINTLOG("\r\n INFO|CurrMon |ADC_Value:");
    for(int i = 0; i < ESC_NUM; i++)        PRINTLOG("%.2f,",Curr_Value[i]);

    sendCnt = mavlink_msg_stm32_f3_motor_curr_pack(1, 1, &mavMsgTx, Curr_Value);
    Mavlink_SendMessage(&mavMsgTx, sendCnt);
}

/******************************END OF FILE******************************/
