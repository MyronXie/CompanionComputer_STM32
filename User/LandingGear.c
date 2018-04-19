/**
  ******************************************************************************
  * File Name       : LandingGear.c
  * Description     : Landing Gear Drivers
  *
  * Version         : v0.4
  * Created Date    : 2017.09.25
  * Revised Date    : 2018.04.19
  *
  * Author          : Mingye Xie
  ******************************************************************************
  */

#include "LandingGear.h"

TIM_HandleTypeDef htim3;
TIM_OC_InitTypeDef hocl,hocr;

uint8_t lgPosNext   = 0;            // Position of Landing Gear [Down: 0, Up: 1]
uint8_t lgPosCurr   = 0;
uint8_t lgMode      = 0;            // Change Status of Landing Gear [Standby: 0, Changing: 1]
uint8_t lgAutoReset = 1;

uint16_t lgPulseL   = PUL_LEFT_DOWN;
uint16_t lgPulseR   = PUL_RIGHT_DOWN;
uint8_t  lgProgress = 0;

float stepListNormal[11]={0.025,0.007,0.004,0.004,0.007,0.014,0.020,0.050,0.050,0.012,0.006};
//float stepListNormal[11]={0.004,0.004,0.004,0.004,0.004,0.004,0.004,0.004,0.004,0.004,0.004};
float stepListReset[11]={0.024,0.010,0.010,0.010,0.010,0.024,0.036,0.048,0.048,0.024,0.016};

void LandingGear_Init(void)
{
    LG_Relay_Init();                        // Low-layer init
    LG_TIM_Init();

    PRINTLOG("\r\n INFO|LandGear|Reset Landing Gear");
    while(lgAutoReset) LG_Reset();
}

void LandingGear_ChangePosition(uint8_t pos)
{
    if(pos==LG_POS_DOWN||pos==LG_POS_UP)  // Param check
    {
        PRINTLOG("\r\n INFO|LandGear|RecvCmd: %s", pos?"UP":"DOWN");
        /* Landing Gear is changing or in delay time, ignore recv command */
        if(lgMode!=LG_MODE_STANDBY)
        {
            if(pos!=lgPosNext)              // Opposite direction, send reject message to FMU
            {
                PRINTLOG(", Reject");
                sendCnt = mavlink_msg_command_ack_pack(1, 1, &mavMsgTx, MAV_CMD_AIRFRAME_CONFIGURATION, MAV_RESULT_TEMPORARILY_REJECTED, 0, 0, 1, 1);
            }
            else                            // Same direction, send ignore message to FMU
            {
                PRINTLOG(", Igrore");
                //sendCnt = mavlink_msg_command_ack_pack(1, 1, &mavMsgTx, MAV_CMD_AIRFRAME_CONFIGURATION, MAV_RESULT_IN_PROGRESS, 0, 0, 1, 1);
            }
            Mavlink_SendMessage(&mavMsgTx, sendCnt);
        }
        /* Landing Gear is standby, respond recv command */
        else
        {
            lgPosNext = pos;
            if(lgPosCurr != lgPosNext)    // Opposite direction, set change status of Landing Gear
            {
                PRINTLOG(", Respond!");
                lgMode = LG_MODE_CHANGING;
            }
            else                                    // Same direction, ignore command
            {
                PRINTLOG(", Igrore");
            }
        }
    }
}

void LandingGear_Control(void)
{
    switch(lgMode)
    {
        case LG_MODE_CHANGING:
            LG_Control();
            break;

        case LG_MODE_STANDBY:
            if(lgAutoReset)     lgMode = LG_MODE_RESETING;
            break;

        case LG_MODE_RESETING:
            LG_Reset();
            break;

        default:    break;
    }
}

void LG_TIM_Init(void)
{
    /* TIM3 for PWM control (50Hz) */
    htim3.Instance          = TIM3;
    htim3.Init.Prescaler    = 128-1;
    htim3.Init.CounterMode  = TIM_COUNTERMODE_UP;
    htim3.Init.Period       = 10000-1;
    htim3.Init.ClockDivision= TIM_CLOCKDIVISION_DIV1;
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
    GPIO_InitStruct.Speed   = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

int16_t LG_PulseStep(uint8_t pos,uint8_t type)
{
    int16_t step,range;

    if(type == LG_STEER_LEFT)   range = PUL_LEFT_RANGE;
    else                        range = PUL_RIGHT_RANGE;

    if(pos == LG_POS_UP)                step = range*stepListNormal[lgProgress/10];     // Resolution:10%
    else if(lgMode == LG_MODE_CHANGING) step = -range*stepListNormal[(100-lgProgress)/10];     // Resolution:10%
    else                                step = -range*stepListReset[(100-lgProgress)/10];

    //PRINTLOG("\r\nDEBUG|LandGear|#%d,%d,%d,%d",pos,type,prog,step);       // <Dev>
    return step;
}

uint8_t LG_AdjustPulse(void)
{
    uint8_t status = 0;

    lgPulseL += LG_PulseStep(lgPosNext,LG_STEER_LEFT);
    lgPulseR += LG_PulseStep(lgPosNext,LG_STEER_RIGHT);
    if(lgPosNext == LG_POS_UP)
        lgProgress = ((lgPulseL-PUL_LEFT_DOWN)*100/PUL_LEFT_RANGE+(lgPulseR-PUL_RIGHT_DOWN)*100/PUL_RIGHT_RANGE)/2;
    else
        lgProgress = ((PUL_LEFT_UP-lgPulseL)*100/PUL_LEFT_RANGE+(PUL_RIGHT_UP-lgPulseR)*100/PUL_RIGHT_RANGE)/2;

    if(lgPulseL>PUL_LEFT_MAX||lgPulseL<PUL_LEFT_MIN||lgPulseR>PUL_RIGHT_MAX||lgPulseR<PUL_RIGHT_MIN)
    {
        lgProgress  = 100;
        if(lgPosNext == LG_POS_UP)
        {
            lgPulseL    = PUL_LEFT_UP;       // Safe Ensurence
            lgPulseR    = PUL_RIGHT_UP;
        }
        else   // LG_POS_DOWN
        {
            lgPulseL    = PUL_LEFT_DOWN;
            lgPulseR    = PUL_RIGHT_DOWN;
        }
        status = 1;
    }

    // Adjust PWM duty cycle
    hocl.Pulse=lgPulseL;
    HAL_TIM_PWM_ConfigChannel(&htim3,&hocl,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
    hocr.Pulse=lgPulseR;
    HAL_TIM_PWM_ConfigChannel(&htim3,&hocr,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);

    return status;
}

/**
  * @brief  Landing Gear Control function.
  * @param  pos current position of landing gear
  * @retval new langding gear changing status
  */
void LG_Control(void)
{
    static uint8_t stage = 0;
    static uint32_t tmTickRelay = 0, tmTickCmd = 0;
    static uint8_t lgCycleCnt = 0;

    lgCycleCnt = (lgCycleCnt+1)%25;

    // Judge
    switch(stage)
    {
        // Change pulse
        case 0x00:
            PRINTLOG("\r\n INFO|LandGear|Start Changing %s",lgPosNext?"UP":"DOWN");
            sendCnt = mavlink_msg_command_ack_pack(1, 1, &mavMsgTx, MAV_CMD_AIRFRAME_CONFIGURATION, MAV_RESULT_IN_PROGRESS, 0, 0, 1, 1);
            Mavlink_SendMessage(&mavMsgTx, sendCnt);

            Relay_ON();                     // Turn on relay to power on steers
            tmTickCmd = HAL_GetTick();
            stage = 0x01;

        case 0x01:
            if(LG_AdjustPulse())
            {
                tmTickRelay = HAL_GetTick();
                stage = 0x02;
            }

            //PRINTLOG("\r\nDEBUG|LandGear|$%4d,%4d",lgPulseL,lgPulseR);  // <Dev> Watch lgPulseL/lgPulseR

            if(!lgCycleCnt)
            {
                PRINTLOG("\r\n INFO|LandGear|Changing:%3d%%",lgProgress);
                sendCnt = mavlink_msg_command_ack_pack(1, 1, &mavMsgTx, MAV_CMD_AIRFRAME_CONFIGURATION, MAV_RESULT_IN_PROGRESS, lgProgress, 0, 1, 1);
                Mavlink_SendMessage(&mavMsgTx, sendCnt);
            }


            break;

        // Waiting for relay finish
        case 0x02:
            if(HAL_GetTick() - tmTickRelay > LG_RELAY_DELAY)
            {
                Relay_OFF();                            // Turn off relay to power off steers
                stage = 0x03;
                lgProgress = 0;
                if(lgPosNext == LG_POS_UP)  LED_ON(LED2);
                else                        LED_OFF(LED2);
            }
            break;

        case 0x03:
            if(HAL_GetTick() - tmTickCmd > LG_CMD_DELAY)
            {
                PRINTLOG("\r\n INFO|LandGear|Stop Changing %s",lgPosNext?"UP":"DOWN");
                sendCnt = mavlink_msg_command_ack_pack(1, 1, &mavMsgTx, MAV_CMD_AIRFRAME_CONFIGURATION, MAV_RESULT_ACCEPTED, 100, 0, 1, 1);
                Mavlink_SendMessage(&mavMsgTx, sendCnt);

                stage = 0x00;
                lgPosCurr = lgPosNext;
                lgMode = LG_MODE_STANDBY;
            }
            break;

        default: break;
    }
}

void LG_Reset(void)
{
    static uint8_t stage = 0;
    static uint32_t timeTick = 0;

    switch(stage)
    {
        case 0x00:
            PRINTLOG("\r\n INFO|LandGear|Start Reset Process");

            lgPosNext   = LG_POS_DOWN;
            Relay_ON();

            if(lgPosCurr == LG_POS_DOWN)
            {
                lgPulseL    = PUL_LEFT_DOWN;
                lgPulseR    = PUL_RIGHT_DOWN;

                hocl.Pulse=lgPulseL;
                HAL_TIM_PWM_ConfigChannel(&htim3,&hocl,TIM_CHANNEL_1);
                HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
                hocr.Pulse=lgPulseR;
                HAL_TIM_PWM_ConfigChannel(&htim3,&hocr,TIM_CHANNEL_2);
                HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);

                timeTick = HAL_GetTick();
                stage = 0x02;
            }
            else
            {
                lgPulseL    = PUL_LEFT_UP;
                lgPulseR    = PUL_RIGHT_UP;
                stage = 0x01;
            }

        case 0x01:
            if(LG_AdjustPulse())
            {
                timeTick = HAL_GetTick();
                stage = 0x02;
            }
            break;

        case 0x02:
            if(HAL_GetTick() - timeTick > LG_RELAY_DELAY)
            {
                PRINTLOG("\r\n INFO|LandGear|Stop Reset Process");
                Relay_OFF();
                lgPosCurr = LG_POS_DOWN;
                lgMode = LG_MODE_STANDBY;
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
