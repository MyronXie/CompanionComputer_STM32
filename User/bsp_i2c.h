/**
  ******************************************************************************
  * File Name       : bsp_i2c.h
  * Description     : Drivers for I2C (based on HAL)
  *
  * Version         : v0.3.1
  * Created Date    : 2018.01.22
  * Revised Date    : 2018.03.13
  *
  * Author          : Mingye Xie
  ******************************************************************************
  */

#ifndef __BSP_I2C_H
#define __BSP_I2C_H


#include "stm32f3xx_hal.h"
#include "string.h"

void I2C_Init(void);

uint8_t I2C_ReadWord(uint8_t _addr, uint8_t _reg, uint16_t* _data);
uint8_t I2C_ReadByte(uint8_t _addr, uint8_t _reg, uint8_t* _data);
uint8_t I2C_ReadBlock(uint8_t _addr, uint8_t _reg, uint8_t* _data, uint8_t _num);

uint8_t I2C_WriteByte(uint8_t _addr, uint8_t _reg, uint8_t _data);
uint8_t I2C_WriteWord(uint8_t _addr, uint8_t _reg, uint16_t _data);
uint8_t I2C_WriteBlock(uint8_t _addr, uint8_t _reg, uint8_t* _data, uint8_t _num);


#endif /* __BSP_I2C_H */

/******************************END OF FILE******************************/
