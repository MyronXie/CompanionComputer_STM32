/**
  ******************************************************************************
  * File Name       : LandingGear.c
  * Description     : Landing Gear Drivers
  *
  * Version         : v0.2
  * Created Date    : 2017.09.25
  * Revised Date    : 2018.03.07
  *
  * Author          : Mingye Xie
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "LandingGear.h"

/* Landing Gear */
uint8_t lgPositionRecv = 0;             // Position of Landing Gear [Down: 0, Up: 1]
uint8_t lgPositionCurr = 0;
uint8_t lgPositionPrev = 0;
uint8_t lgChangeDelayCnt = 0;           // Delay for prevent too fast changing
uint8_t lgChangeStatusCurr = 0;         // Change Status of Landing Gear [Standby: 0, Changing: 1]
uint8_t lgChangeStatusPrev = 0;
uint8_t lgChangeProgress = 50;          // Change Progress of Landing Gear [0-100(%)], not used now
uint8_t lgAutoReset = 0;

uint32_t flashParam[FLASHSIZE];

uint16_t lgPulseL=PUL_LEFT_DOWN;
uint16_t lgPulseR=PUL_RIGHT_DOWN;

TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
TIM_OC_InitTypeDef hocl,hocr;



void LandingGear_Init(void)
{
    LG_Relay_Init();                    // Low-layer init
    LG_TIM_Init();
    Relay_OFF();

    lgPositionCurr = FLASH_LoadParam(0);
    lgChangeStatusCurr = FLASH_LoadParam(1);
    PRINTLOG(": %s,%s",lgPositionCurr?"UP":"DOWN",lgChangeStatusCurr?"Changing":"Standby");
    /* Landing Gear not reset in last process, Reset Landing Gear */
    if(lgPositionCurr||lgChangeStatusCurr)
    {
        PRINTLOG("\r\n [ACT]  Reset Landing Gear");
        LG_Reset();
        lgPositionCurr = 0, lgChangeStatusCurr = 0;
        flashParam[0] = lgPositionCurr;     flashParam[1] = lgChangeStatusCurr;
        FLASH_SaveParam(flashParam,2);
    }
}

void LandingGear_Control(mavlink_command_long_t* cmd)
{
    lgPositionRecv = (int)cmd->param2;          // .param2 for landing gear position
    if(lgPositionRecv==0||lgPositionRecv==1)    // Param check
    {
        PRINTLOG("\r\n [INFO] Landing Gear: %s", lgPositionRecv?"UP":"DOWN");
        /* Landing Gear is changing or in delay time, ignore recv command */
        if(lgChangeStatusCurr||lgChangeDelayCnt)
        {
            if(lgPositionRecv!=lgPositionCurr)  // Opposite direction, send reject message to FMU
            {
                PRINTLOG(", Reject");
                sendCnt = mavlink_msg_command_ack_pack(1, 1, &mavMsgTx, MAV_CMD_AIRFRAME_CONFIGURATION, MAV_RESULT_TEMPORARILY_REJECTED, 0, 0, 1, 1);
            }
            else                                // Same direction, send ignore message to FMU
            {
                PRINTLOG(", Igrore");
                sendCnt = mavlink_msg_command_ack_pack(1, 1, &mavMsgTx, MAV_CMD_AIRFRAME_CONFIGURATION, MAV_RESULT_IN_PROGRESS, 0, 0, 1, 1);
            }
            Mavlink_SendMessage(&mavMsgTx, sendCnt);
        }
        /* Landing Gear is standby, respond recv command */
        else
        {
            lgPositionCurr = lgPositionRecv;
            if(lgPositionPrev != lgPositionCurr)    // Opposite direction, set change status of Landing Gear
            {
                PRINTLOG(", Respond");
                lgChangeStatusCurr = 1;
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
    // Decrease lgChangeDelayCnt
    if(lgChangeDelayCnt)    lgChangeDelayCnt--;

    if(lgChangeStatusPrev != lgChangeStatusCurr)    // Change status message
    {
        lgChangeStatusPrev = lgChangeStatusCurr;
        if(lgChangeStatusCurr)                      // Changing Process Start
        {
            PRINTLOG("\r\n [ACT]  Landing Gear: Start(%s)",lgPositionCurr?"UP":"DOWN");
            sendCnt = mavlink_msg_command_ack_pack(1, 1, &mavMsgTx, MAV_CMD_AIRFRAME_CONFIGURATION, MAV_RESULT_IN_PROGRESS, 0, 0, 1, 1);
        }
        else                                        // Changing Process Finish
        {
            PRINTLOG("\r\n [ACT]  Landing Gear: Stop (%s)",lgPositionCurr?"UP":"DOWN");
            sendCnt = mavlink_msg_command_ack_pack(1, 1, &mavMsgTx, MAV_CMD_AIRFRAME_CONFIGURATION, MAV_RESULT_ACCEPTED, 100, 0, 1, 1);
        }
        Mavlink_SendMessage(&mavMsgTx, sendCnt);
        flashParam[0] = lgPositionCurr;     flashParam[1] = lgChangeStatusCurr;
        FLASH_SaveParam(flashParam,2);
    }
    /* Landing Gear changing process */
    if(lgChangeStatusCurr)
    {
        Relay_ON();                                 // Turn on relay to power on steers
        // If changing process finished, lgChangeStatus turns 0
        lgChangeStatusCurr = LG_Control(lgPositionCurr, &lgChangeProgress);
        lgChangeDelayCnt = 200;                     // Ignore cmd for 2s (guard time)
    }
    else Relay_OFF();                               // Turn off relay to power off steers
}

void LandingGear_Reset(void)
{
    MsgType lgMsg = {0,0};

    if(lgPositionCurr)                      // Changing Landing Gear cost 1~2s approx., no need to judge change status
    {
        PRINTLOG("\r\n [ACT]  LandingGear: Reset...");
        //HAL_TIM_Base_Stop_IT(&htim6);       // Stop general changing process temp.
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
  * @param  prog precentage of changing progress
  * @retval new langding gear changing status
  */
uint8_t LG_Control(uint8_t pos, uint8_t* prog)
{
    uint8_t changeStatus = 1;

    if(pos==1)          // Landing Gear Up(1)
    {
        lgPulseL+=PUL_LEFT_Range*PUL_SCALE_UP;
        lgPulseR+=PUL_RIGHT_Range*PUL_SCALE_UP;
        if(lgPulseL>PUL_LEFT_UP||lgPulseR>PUL_RIGHT_UP)
        {
            lgPulseL=PUL_LEFT_UP;
            lgPulseR=PUL_RIGHT_UP;          //Ensure safe
            changeStatus=0;                 //Finish process
        }
        *prog=(uint8_t)((lgPulseL-PUL_LEFT_DOWN)*100.0/PUL_LEFT_Range);
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
        *prog=(uint8_t)((PUL_LEFT_UP-lgPulseL)*100.0/PUL_LEFT_Range);
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
            if(HAL_GetTick() - timeTick < 1500)
            {
                Relay_OFF();
                lgPositionCurr = 0, lgChangeStatusCurr = 0;
                flashParam[0] = lgPositionCurr;     flashParam[1] = lgChangeStatusCurr;
                FLASH_SaveParam(flashParam,2);
                //HAL_TIM_Base_Start_IT(&htim6);      // Restart general changing process
                lgAutoReset = 0;
                stage = 0x00;
            }
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
    #elif BOARD_REV2
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Pin  = GPIO_PIN_13;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
    #endif
}

/******************************END OF FILE******************************/
