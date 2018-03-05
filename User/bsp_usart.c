/**
  ******************************************************************************
  * File Name       : bsp_usart.c
  * Description     : Drivers for usart (based on HAL)
  *
  * Version         : v0.3
  * Created Date    : 2017.10.18
  * Revised Date    : 2018.02.28
  *
  * Author          : Mingye Xie
  ******************************************************************************
  */

#include "bsp_usart.h"

UART_HandleTypeDef huart1,huart3;
SerialType USART1_Tx,USART1_Rx,USART3_Tx;

void USART_Config_Init(void)
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
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    // USART3 for debug
    huart3.Instance         = USART3;
    huart3.Init.BaudRate    = 256000;
    huart3.Init.WordLength  = UART_WORDLENGTH_8B;
    huart3.Init.StopBits    = UART_STOPBITS_1;
    huart3.Init.Parity      = UART_PARITY_NONE;
    huart3.Init.HwFlowCtl   = UART_HWCONTROL_NONE;
    huart3.Init.Mode        = UART_MODE_TX_RX;
    HAL_UART_Init(&huart3);
    HAL_NVIC_SetPriority(USART3_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}

void USART_Buffer_Init(void)
{
    USART1_Tx.handle = &huart1;
    USART1_Tx.length = BUFFSIZE;
    USART1_Tx.front  = USART1_Tx.buffer;
    USART1_Tx.rear   = USART1_Tx.buffer;
    USART1_Tx.flag   = 0;
    
    USART1_Rx.handle = &huart1;
    USART1_Rx.length = BUFFSIZE;
    USART1_Rx.front  = USART1_Rx.buffer;
    USART1_Rx.rear   = USART1_Rx.buffer;
    USART1_Rx.flag   = 0;
    
    USART3_Tx.handle = &huart3;
    USART3_Tx.length = BUFFSIZE;
    USART3_Tx.front  = USART3_Tx.buffer;
    USART3_Tx.rear   = USART3_Tx.buffer;
    USART3_Tx.flag   = 0;

    HAL_UART_Receive_IT(USART1_Rx.handle,USART1_Rx.rear,1);
}

void USART_Init(void)
{
    USART_Config_Init();
    USART_Buffer_Init();
}

void USART_ReInit(void)
{
    HAL_UART_DeInit(&huart1);
    HAL_UART_DeInit(&huart3);
    USART_Config_Init();
    HAL_UART_Receive_IT(USART1_Rx.handle,USART1_Rx.rear,1);
    if(USART1_Tx.front!=USART1_Tx.rear) HAL_UART_Transmit_IT(USART1_Tx.handle,USART1_Tx.front,1);
    if(USART3_Tx.front!=USART3_Tx.rear) HAL_UART_Transmit_IT(USART3_Tx.handle,USART3_Tx.front,1);
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
        if(++USART1_Rx.rear >= (USART1_Rx.buffer + USART1_Rx.length))   USART1_Rx.rear = USART1_Rx.buffer;
        HAL_UART_Receive_IT(USART1_Rx.handle, USART1_Rx.rear, 1);     // Restart usart1's IT for next receive process
    }
}

uint8_t Serial_Rx_Available(void)
{
    if(USART1_Rx.front != USART1_Rx.rear)	return 1;
    else return 0;
}

uint8_t Serial_Rx_NextByte(void)  
{  
    uint8_t tmp;
    
    tmp = (uint8_t)(*USART1_Rx.front);
    if (++USART1_Rx.front >= (USART1_Rx.buffer+USART1_Rx.length)) USART1_Rx.front = USART1_Rx.buffer;  
    
    return tmp; 
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{   
    if(huart->Instance == USART1)
    { 
        if(USART1_Tx.front != USART1_Tx.rear)
        {           
            HAL_UART_Transmit_IT(USART1_Tx.handle, USART1_Tx.front, 1);
            if (++USART1_Tx.front >= (USART1_Tx.buffer+USART1_Tx.length)) USART1_Tx.front = USART1_Tx.buffer;
        }
        else USART1_Tx.flag = 0;
    }
    
    if(huart->Instance == USART3)
    { 
        if(USART3_Tx.front != USART3_Tx.rear)
        {           
            HAL_UART_Transmit_IT(USART3_Tx.handle, USART3_Tx.front, 1);
            if (++USART3_Tx.front >= (USART3_Tx.buffer + USART3_Tx.length)) USART3_Tx.front = USART3_Tx.buffer;
        }
        else USART3_Tx.flag = 0;
    }
}

void Serial_Send(SerialType* serial, uint8_t* buf, uint16_t len)
{
    uint16_t lentmp;
    
    // Package process
    if((serial->rear + len) < (serial->buffer + serial->length))    // Judge whether exceed end of buffer
    {
        memcpy(serial->rear, buf, len);
        serial->rear += len;
    }
    else    // Exceed end of buffer, divide into two parts 
    {
        lentmp = (serial->buffer + serial->length) - serial->rear;
        memcpy(serial->rear, buf, lentmp);
        memcpy(serial->buffer, buf+lentmp, len-lentmp);
        serial->rear = serial->buffer + (len-lentmp);
    }
    
    // Start sending process
    if(!serial->flag)
    {
        serial->flag = 1;
        HAL_UART_Transmit_IT(serial->handle, serial->front++, 1);
    }
}

// These function is not used now.

//int fputc(int ch, FILE *f)          //->printf()
//{
//    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 1);
//    return ch;
//}

//int fgetc(FILE * f)                 //->scanf()
//{
//    uint8_t ch = 0;
//    HAL_UART_Receive(&huart3, (uint8_t *)&ch, 1, 1);
//    return ch;
//}
/******************************END OF FILE******************************/
