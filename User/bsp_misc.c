/**
  ******************************************************************************
  * File Name       : bsp_misc.c
  * Description     : Drivers for LED, Flash, IWDG (based on HAL)
  *
  * Version         : v0.3
  * Created Date    : 2018.02.01
  * Revised Date    : 2018.03.07
  *
  * Author          : Mingye Xie
  ******************************************************************************
  */

#include "bsp_misc.h"

FLASH_EraseInitTypeDef flashErase;
uint32_t PageError = 0;

IWDG_HandleTypeDef hiwdg;

/**
  * @brief  Save Landing Gear status on flash
  * @param  pos position of Landing Gear
  *         cng change status of Landing Gear
  * @retval None
  */
void FLASH_SaveParam(uint32_t* param, uint8_t size)
{
    uint8_t cnt = 0;

    // Unlock
    HAL_FLASH_Unlock();

    // Erase
    flashErase.TypeErase = FLASH_TYPEERASE_PAGES;
    flashErase.PageAddress = FLASHADDR;
    flashErase.NbPages = 1;
    HAL_FLASHEx_Erase(&flashErase, &PageError);

    // Write
    for(cnt=0;cnt<size;cnt++)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASHADDR+4*(cnt), (uint32_t)param[cnt]);
    }

    // Lock
    HAL_FLASH_Lock();
}

/**
  * @brief  Load Landing Gear status on flash
  * @param  pos position of Landing Gear
  *         cng change status of Landing Gear
  * @retval None
  */
uint32_t FLASH_LoadParam(uint8_t id)
{
    return *(__IO uint32_t*)(FLASHADDR+4*id);
}

void IWDG_Init(void)
{
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
    hiwdg.Init.Window = 4095;
    hiwdg.Init.Reload = 2000;                   // Reboot system if not refresh iwdg in 2s
    HAL_IWDG_Init(&hiwdg);
}

void IWDG_Feed(void)
{
    HAL_IWDG_Refresh(&hiwdg);
}

void LED_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    GPIO_InitStruct.Pin         = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_1;
    GPIO_InitStruct.Mode        = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull        = GPIO_PULLUP;
    GPIO_InitStruct.Speed       = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate   = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    LED_OFF(LED1);
    LED_OFF(LED2);
    LED_OFF(LED3);
    LED_OFF(LED4);
}

/******************************END OF FILE******************************/
