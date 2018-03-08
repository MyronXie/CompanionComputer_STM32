/**
  ******************************************************************************
  * File Name       : bsp_i2c.c
  * Description     : Drivers for I2C (based on HAL)
  *
  * Version         : v0.2
  * Created Date    : 2018.01.22
  * Revised Date    : 2018.01.25
  *
  * Author          : Mingye Xie
  ******************************************************************************
  */

#include "bsp_i2c.h"

I2C_HandleTypeDef I2cHandle;

/**
  * @brief  I2C Configuration (using I2C_Timing_Configuration_V1.0.1.xls)
  *            Device Mode                    = Master
  *            I2C Speed Mode                 = Standard Mode
  *            I2C Speed Frequency(KHz)       = 40
  *            I2C Clock Source Frequency(KHz)= 64000
  *            Analog Filter Delay            = ON
  *            Coefficient of Digital Filter  = 0
  *            Rise Time(ns)                  = 1000
  *            Fall Time(ns)                  = 300
  */
void I2C_Init(void)
{
    I2cHandle.Instance              = I2C1;
    I2cHandle.Init.Timing           = 0x20E22EEA;//0x40F39BFB;//0x50D329D0;//0x20E22EEA;
    I2cHandle.Init.OwnAddress1      = 0x00;//Dummy
    I2cHandle.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    I2cHandle.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    I2cHandle.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    I2cHandle.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

    HAL_I2C_Init(&I2cHandle);
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;

    /*##-1- Configure the I2C clock source. The clock is derived from the SYSCLK #*/
    RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    RCC_PeriphCLKInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

    /*##-2- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    /*##-3- Configure peripheral GPIO ##########################################*/
    /* I2C TX GPIO pin configuration  */
    GPIO_InitStruct.Pin         = GPIO_PIN_8;
    GPIO_InitStruct.Mode        = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull        = GPIO_PULLUP;
    GPIO_InitStruct.Speed       = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate   = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C RX GPIO pin configuration  */
    GPIO_InitStruct.Pin         = GPIO_PIN_9;
    GPIO_InitStruct.Alternate   = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}


uint8_t I2C_ReadWord(uint8_t _addr, uint8_t _reg, uint16_t* _data)
{
    uint8_t msg[2],status=0;

    status += HAL_I2C_Master_Transmit(&I2cHandle, _addr, &_reg, 1, 1);
    status += HAL_I2C_Master_Receive(&I2cHandle, _addr+1, msg, 2, 1);

    *_data=(uint16_t)((msg[1]<<8)+msg[0]);

    return status;
}

uint8_t I2C_WriteWord(uint8_t _addr, uint8_t _reg, uint16_t _data)
{
    uint8_t msg[3],status=0;

    msg[0]=_reg;
    msg[1]=(uint8_t)(_data&0xFF);
    msg[2]=(uint8_t)(_data>>8);
    status += HAL_I2C_Master_Transmit(&I2cHandle, _addr, msg, 3, 1);

    return status;
}

uint8_t I2C_WriteByte(uint8_t _addr, uint8_t _reg, uint8_t _data)
{
    uint8_t msg[2],status=0;

    msg[0]=_reg;
    msg[1]=_data;
    status += HAL_I2C_Master_Transmit(&I2cHandle, _addr, msg, 2, 1);

    return status;
}

uint8_t I2C_ReadByte(uint8_t _addr, uint8_t _reg, uint8_t* _data)
{
    uint8_t msg[1],status=0;

    status += HAL_I2C_Master_Transmit(&I2cHandle, _addr, &_reg, 1, 1);
    status += HAL_I2C_Master_Receive(&I2cHandle, _addr+1, msg, 1, 1);

    *_data=msg[0];

    return status;
}
/******************************END OF FILE******************************/
