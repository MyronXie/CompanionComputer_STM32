/**
  ******************************************************************************
  * File Name       : bsp_usart.c
  * Description     : Drivers for usart (based on HAL)
  *
  * Version         : v0.2
  * Created Date    : 2017.10.18
  * Revised Date    : 2018.01.31
  *
  * Author          : Mingye Xie
  ******************************************************************************
  */

#include "bsp_usart.h"

UART_HandleTypeDef huart1,huart3;
uint8_t aRxBuffer[BUFFSIZE];
uint8_t *rxBufFront,*rxBufRear;

uint16_t bufCnt = 0;
uint16_t bufCntLst = 0;
uint16_t bufCntFlag = 0;


void USART_Init(void)
{
    // USART1 for Pixhawk
    huart1.Instance         = USART1;
    huart1.Init.BaudRate    = 57600;
    huart1.Init.WordLength  = UART_WORDLENGTH_8B;
    huart1.Init.StopBits    = UART_STOPBITS_1;
    huart1.Init.Parity      = UART_PARITY_NONE;
    huart1.Init.HwFlowCtl   = UART_HWCONTROL_NONE;
    huart1.Init.Mode        = UART_MODE_TX_RX;
    HAL_UART_Init(&huart1);

    HAL_NVIC_SetPriority(USART1_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    // USART3 for debug
    huart3.Instance         = USART3;
    huart3.Init.BaudRate    = 115200;
    huart3.Init.WordLength  = UART_WORDLENGTH_8B;
    huart3.Init.StopBits    = UART_STOPBITS_1;
    huart3.Init.Parity      = UART_PARITY_NONE;
    huart3.Init.HwFlowCtl   = UART_HWCONTROL_NONE;
    huart3.Init.Mode        = UART_MODE_TX_RX;
    HAL_UART_Init(&huart3);

    HAL_NVIC_SetPriority(USART3_IRQn, 0, 3);
    HAL_NVIC_EnableIRQ(USART3_IRQn);

    rxBufFront  = aRxBuffer;
    rxBufRear   = aRxBuffer;
    HAL_UART_Receive_IT(&huart1,aRxBuffer,1);
}

void USART_DeInit(void)
{
    rxBufFront  = aRxBuffer;
    rxBufRear   = aRxBuffer;
    HAL_UART_DeInit(&huart1);
    HAL_UART_DeInit(&huart3);
}

void USART_ReInit(void)
{
    USART_DeInit();
    USART_Init();
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitS;

    if(huart->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        
        GPIO_InitS.Pin          = GPIO_PIN_6|GPIO_PIN_7;
        GPIO_InitS.Mode         = GPIO_MODE_AF_PP;
        GPIO_InitS.Pull         = GPIO_PULLUP;
        GPIO_InitS.Speed        = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitS.Alternate	= GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOB,&GPIO_InitS);
    }
    
    if(huart->Instance == USART3)
    {
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        
        GPIO_InitS.Pin          = GPIO_PIN_10|GPIO_PIN_11;
        GPIO_InitS.Mode         = GPIO_MODE_AF_PP;
        GPIO_InitS.Pull         = GPIO_PULLUP;
        GPIO_InitS.Speed        = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitS.Alternate	= GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOB,&GPIO_InitS);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);
        HAL_NVIC_DisableIRQ(USART1_IRQn); 
    }
    
    if(huart->Instance == USART3)
    {
        __HAL_RCC_USART3_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);
        HAL_NVIC_DisableIRQ(USART3_IRQn); 
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    { 
        if(++rxBufRear >= (aRxBuffer + BUFFSIZE))   rxBufRear = aRxBuffer;  
        
        HAL_UART_Receive_IT(&huart1, rxBufRear, 1);     // Restart usart1's IT for next receive process
    }
}

uint8_t Serial_Available(void)
{
    if(rxBufFront != rxBufRear)	return 1;
    else return 0;
}

uint8_t Serial_GetNextByte(void)  
{  
    uint8_t tmp;
    
    tmp = (uint8_t)*(rxBufFront);
    
    rxBufFront++;  

    if (rxBufFront >= (aRxBuffer+BUFFSIZE))  
        rxBufFront = aRxBuffer;  
    
    return tmp; 
}

int fputc(int ch, FILE *f)          //->printf()
{
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 1);
    return ch;
}

int fgetc(FILE * f)                 //->scanf()
{
    uint8_t ch = 0;
    HAL_UART_Receive(&huart3, (uint8_t *)&ch, 1, 1);
    return ch;
}
/******************************END OF FILE******************************/
