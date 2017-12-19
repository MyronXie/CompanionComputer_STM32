/**
  ******************************************************************************
  * File Name		: BattMgmt.c
  * Description		: Battery Management Drivers (through I2C)
  *
  * Version			: v0.1
  * Created	Date	: 2017.09.25
  * Revised	Date	: 2017.11.27
  *
  * Author			: Mingye Xie
  ******************************************************************************
  */

#include "BattMgmt.h"

I2C_HandleTypeDef I2cHandle;

void Batt_Init(void)
{
	
	I2cHandle.Instance             = I2C1;
	I2cHandle.Init.Timing          = 0x20E22EEA;//0x1042C3C7;//0x60305B85//0x00201881
	I2cHandle.Init.OwnAddress1     = 0x00;//Dummy
	I2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
	I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;  
	
	if(HAL_I2C_Init(&I2cHandle) != HAL_OK)
	{
		/* Initialization Error */
		while(1);    
	}
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
	GPIO_InitStruct.Pin       = GPIO_PIN_8;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull      = GPIO_PULLUP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* I2C RX GPIO pin configuration  */
	GPIO_InitStruct.Pin       = GPIO_PIN_9;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}


uint8_t Batt_ReadWord(uint8_t _addr, uint8_t _reg, uint16_t* _data)
{
	uint8_t msg[2],status=0;
	
	while(HAL_I2C_Master_Transmit(&I2cHandle, _addr, &_reg, 1, 1000)!=HAL_OK){}
	while(HAL_I2C_Master_Receive(&I2cHandle, _addr+1, msg, 2, 1000)!=HAL_OK){}
	
	*_data=(uint16_t)((msg[1]<<8)+msg[0]);
	
	return status;
}

uint8_t Batt_WriteWord(uint8_t _addr, uint8_t _reg, uint16_t _data)
{
	uint8_t msg[3],status=0;
	
	msg[0]=_reg;
	msg[1]=(uint8_t)(_data>>8);
	msg[2]=(uint8_t)(_data&0xFF);
	while(HAL_I2C_Master_Transmit(&I2cHandle, _addr, msg, 3, 1000)!=HAL_OK)	{	}
		
	
	return status;
}

uint8_t Batt_WriteByte(uint8_t _addr, uint8_t _reg, uint8_t _data)
{
	uint8_t msg[2],status=0;
	
	msg[0]=_reg;
	msg[1]=_data;
	while(HAL_I2C_Master_Transmit(&I2cHandle, _addr, msg, 2, 1000)!=HAL_OK)	{	}
		
	
	return status;
}

uint8_t Batt_ReadByte(uint8_t _addr, uint8_t _reg, uint8_t* _data)
{
	uint8_t msg[1],status=0;
	
	while(HAL_I2C_Master_Transmit(&I2cHandle, _addr, &_reg, 1, 1000)!=HAL_OK){}
	while(HAL_I2C_Master_Receive(&I2cHandle, _addr+1, msg, 1, 1000)!=HAL_OK){}
	
	*_data=msg[0];
	
	return status;
}
/******************************END OF FILE******************************/