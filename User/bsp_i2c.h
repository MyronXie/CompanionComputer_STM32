/**
  ******************************************************************************
  * File Name		: bsp_i2c.h
  * Description		: Drivers for I2C (based on HAL)
  *
  * Version			: v0.2
  * Created	Date	: 2018.01.22
  * Revised	Date	: 2018.01.22
  *
  * Author			: Mingye Xie
  ******************************************************************************
  */

#ifndef __BSP_I2C_H
#define __BSP_I2C_H


#include "stm32f3xx_hal.h"


void I2C_Init(void);

uint8_t I2C_WriteWord(uint8_t _addr, uint8_t _reg, uint16_t _data);
uint8_t I2C_ReadWord(uint8_t _addr, uint8_t _reg, uint16_t* _data);
uint8_t I2C_WriteByte(uint8_t _addr, uint8_t _reg, uint8_t _data);
uint8_t I2C_ReadByte(uint8_t _addr, uint8_t _reg, uint8_t* _data);


#endif /* __BSP_I2C_H */

/******************************END OF FILE******************************/
