/**
  ******************************************************************************
  * File Name       : LandingGear.c
  * Description     : Landing Gear Drivers
  *
  * Version         : v0.3.1
  * Created Date    : 2017.09.25
  * Revised Date    : 2018.04.16
  *
  * Author          : Mingye Xie
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "LandingGear.h"

/* Landing Gear */
uint8_t lgPositionCurr = 0;         // Position of Landing Gear [Down: 0, Up: 1]
uint8_t lgPositionPrev = 0;
uint8_t lgDelayCnt = 0;             // Prevent too fast changing
uint8_t lgChangeCurr = 0;           // Change Status of Landing Gear [Standby: 0, Changing: 1]
uint8_t lgChangePrev = 0;           // To indicate begin or end of changing status
uint8_t lgAutoReset = 1;

uint16_t lgPulseL=PUL_LEFT_DOWN;
uint16_t lgPulseR=PUL_RIGHT_DOWN;

TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
TIM_OC_InitTypeDef hocl,hocr;

void LandingGear_Init(void)
{
    LG_Relay_Init();                        // Low-layer init
    LG_TIM_Init();

    PRINTLOG("\r\n [ACT]  Reset Landing Gear");
    while(lgAutoReset) LG_Reset();
}

void LandingGear_Control(uint8_t param)
{
    if(param==0||param==1)                  // Param check
    {
        PRINTLOG("\r\n [INFO] Landing Gear: %s", param?"UP":"DOWN");
        /* Landing Gear is changing or in delay time, ignore recv command */
        if(lgChangeCurr||lgDelayCnt)
        {
            if(param!=lgPositionCurr)       // Opposite direction, send reject message to FMU
            {
                PRINTLOG(", Reject");
                sendCnt = mavlink_msg_command_ack_pack(1, 1, &mavMsgTx, MAV_CMD_AIRFRAME_CONFIGURATION, MAV_RESULT_TEMPORARILY_REJECTED, 0, 0, 1, 1);
            }
            else                            // Same direction, send ignore message to FMU
            {
                PRINTLOG(", Igrore");
                sendCnt = mavlink_msg_command_ack_pack(1, 1, &mavMsgTx, MAV_CMD_AIRFRAME_CONFIGURATION, MAV_RESULT_IN_PROGRESS, 0, 0, 1, 1);
            }
            Mavlink_SendMessage(&mavMsgTx, sendCnt);
        }
        /* Landing Gear is standby, respond recv command */
        else
        {
            lgPositionCurr = param;
            if(lgPositionPrev != lgPositionCurr)    // Opposite direction, set change status of Landing Gear
            {
                PRINTLOG(", Respond");
                lgChangeCurr = 1;
                lgPositionPrev = lgPositionCurr;
            }
            else                                    // Same direction, ignore command
            {
                PRINTLOG(", Igrore");
            }
        }
    }
}

void LandingGear_Adjustment(void)
{
    // Decrease lgDelayCnt
    if(lgDelayCnt)    lgDelayCnt--;

    if(lgChangePrev != lgChangeCurr)            // Change status message
    {
        lgChangePrev = lgChangeCurr;
        if(lgChangeCurr)                        // Changing Process Start
        {
            LED_ON(LED2);
            PRINTLOG("\r\n [ACT]  Landing Gear: Start(%s)",lgPositionCurr?"UP":"DOWN");
            sendCnt = mavlink_msg_command_ack_pack(1, 1, &mavMsgTx, MAV_CMD_AIRFRAME_CONFIGURATION, MAV_RESULT_IN_PROGRESS, 0, 0, 1, 1);
        }
        else                                    // Changing Process Finish
        {
            LED_OFF(LED2);
            PRINTLOG("\r\n [ACT]  Landing Gear: Stop (%s)",lgPositionCurr?"UP":"DOWN");
            sendCnt = mavlink_msg_command_ack_pack(1, 1, &mavMsgTx, MAV_CMD_AIRFRAME_CONFIGURATION, MAV_RESULT_ACCEPTED, 100, 0, 1, 1);
        }
        Mavlink_SendMessage(&mavMsgTx, sendCnt);
    }
    /* Landing Gear changing process */
    if(lgChangeCurr)
    {
        Relay_ON();                             // Turn on relay to power on steers
        // If changing process finished, lgChange turns 0
        lgChangeCurr = LG_Control(lgPositionCurr);
        lgDelayCnt = LG_CHANGE_DELAY;           // Ignore cmd for 2s (guard time)
    }
    else Relay_OFF();                           // Turn off relay to power off steers
}

void LandingGear_Reset(void)
{
    MsgType lgMsg = {0,0};

    if(lgPositionCurr)                          // Changing Landing Gear cost 1~2s approx., no need to judge change status
    {
        PRINTLOG("\r\n [ACT]  LandingGear: Reset...");
        lgAutoReset = 1;
        lgMsg.cmd = MSG_LG_RESET;
        ReportMessage(lgMsg);
    }
}

void LG_TIM_Init(void)
{
    /* TIM3 for PWM control (50Hz) */
    htim3.Instance          = TIM3;
    htim3.Init.Prescaler    = 128;
    htim3.Init.CounterMode  = TIM_COUNTERMODE_UP;
    htim3.Init.Period       = 10000-1;
    htim3.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim3);

    hocl.OCMode         = TIM_OCMODE_PWM1;
    hocl.OCPolarity     = TIM_OCPOLARITY_HIGH;
    hocl.Pulse          = PUL_LEFT_DOWN;
    HAL_TIM_PWM_ConfigChannel(&htim3,&hocl,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);

    hocr.OCMode         = TIM_OCMODE_PWM1;
    hocr.OCPolarity     = TIM_OCPOLARITY_HIGH;
    hocr.Pulse          = PUL_RIGHT_DOWN;
    HAL_TIM_PWM_ConfigChannel(&htim3,&hocr,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin     = GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode    = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull    = GPIO_NOPULL;
    GPIO_InitStruct.Speed   = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
  * @brief  Landing Gear Control function.
  * @param  pos current position of landing gear
  * @retval new langding gear changing status
  */
uint8_t LG_Control(uint8_t pos)
{
    uint8_t changeStatus = 1;

    if(pos==1)          // Landing Gear Up(1)
    {
        lgPulseL+=PUL_LEFT_Range*PUL_SCALE_UP;
        lgPulseR+=PUL_RIGHT_Range*PUL_SCALE_UP;
        if(lgPulseL>PUL_LEFT_UP||lgPulseR>PUL_RIGHT_UP)
        {
            lgPulseL=PUL_LEFT_UP;
            lgPulseR=PUL_RIGHT_UP;          // Ensure safe
            changeStatus=0;                 // Finish process
        }
    }
    else if(pos==0)     // Landing Gear Down(0)
    {
        lgPulseL-=PUL_LEFT_Range*PUL_SCALE_DOWN;
        lgPulseR-=PUL_RIGHT_Range*PUL_SCALE_DOWN;
        if(lgPulseL<PUL_LEFT_DOWN||lgPulseR<PUL_RIGHT_DOWN)
        {
            lgPulseL=PUL_LEFT_DOWN;
            lgPulseR=PUL_RIGHT_DOWN;
            changeStatus=0;
        }
    }

    hocl.Pulse=lgPulseL;
    HAL_TIM_PWM_ConfigChannel(&htim3,&hocl,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);

    hocr.Pulse=lgPulseR;
    HAL_TIM_PWM_ConfigChannel(&htim3,&hocr,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);

    return changeStatus;
}

void LG_Reset(void)
{
    static uint8_t stage = 0;
    static uint32_t timeTick = 0;

    switch(stage)
    {
        case 0x00:
            Relay_ON();

            lgPulseL=PUL_LEFT_DOWN;
            lgPulseR=PUL_RIGHT_DOWN;

            hocl.Pulse=lgPulseL;
            HAL_TIM_PWM_ConfigChannel(&htim3,&hocl,TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);

            hocr.Pulse=lgPulseR;
            HAL_TIM_PWM_ConfigChannel(&htim3,&hocr,TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);

            timeTick = HAL_GetTick();
            stage = 0x01;

        case 0x01:
            if(HAL_GetTick() - timeTick > 1000)
            {
                Relay_OFF();
                lgPositionCurr = 0, lgChangeCurr = 0;
                lgAutoReset = 0;
                stage = 0x00;
            }

        default: break;
    }
}

void LG_Relay_Init(void)
{
    GPIO_InitTypeDef   GPIO_InitStructure;

    #ifdef BOARD_REV1
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Pin  = GPIO_PIN_11;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
    #else
    #ifdef BOARD_REV2
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Pin  = GPIO_PIN_13;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
    #endif
    #endif

    Relay_OFF();
}

/******************************END OF FILE******************************/
