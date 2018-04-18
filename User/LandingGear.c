/**
  ******************************************************************************
  * File Name       : LandingGear.c
  * Description     : Landing Gear Drivers
  *
  * Version         : v0.4
  * Created Date    : 2017.09.25
  * Revised Date    : 2018.04.18
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
uint8_t lgModeCurr = 0;             // Change Status of Landing Gear [Standby: 0, Changing: 1]
uint8_t lgModePrev = 0;             // To indicate begin or end of changing status
uint8_t lgAutoReset = 1;

uint16_t lgPulseL=PUL_LEFT_MIN;
//uint16_t lgPulseR=PUL_RIGHT_MIN;
uint16_t lgPulseR=PUL_RIGHT_MAX;
uint8_t  lgProgress=0;

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

void LandingGear_Control(uint8_t posi)
{
    if(posi==LG_POS_DOWN||posi==LG_POS_UP)  // Param check
    {
        PRINTLOG("\r\n [INFO] Landing Gear: %s", posi?"UP":"DOWN");
        /* Landing Gear is changing or in delay time, ignore recv command */
        if(lgModeCurr||lgDelayCnt)
        {
            if(posi!=lgPositionCurr)        // Opposite direction, send reject message to FMU
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
            lgPositionCurr = posi;
            if(lgPositionPrev != lgPositionCurr)    // Opposite direction, set change status of Landing Gear
            {
                PRINTLOG(", Respond");
                lgModeCurr = LG_MODE_CHANGING;
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
    static uint8_t lgCycleCnt = 50-1;

    lgCycleCnt = (lgCycleCnt+1)%50;

    // Decrease lgDelayCnt
    if(lgDelayCnt)    lgDelayCnt--;

    if(lgModePrev != lgModeCurr)                // Change status message
    {
        lgModePrev = lgModeCurr;
        if(lgModeCurr)                          // Changing Process Start
        {
            PRINTLOG("\r\n [ACT]  Landing Gear: Start(%s)",lgPositionCurr?"UP":"DOWN");
            sendCnt = mavlink_msg_command_ack_pack(1, 1, &mavMsgTx, MAV_CMD_AIRFRAME_CONFIGURATION, MAV_RESULT_IN_PROGRESS, 0, 0, 1, 1);
        }
        else                                    // Changing Process Finish
        {
            PRINTLOG("\r\n [ACT]  Landing Gear: Stop (%s)",lgPositionCurr?"UP":"DOWN");
            sendCnt = mavlink_msg_command_ack_pack(1, 1, &mavMsgTx, MAV_CMD_AIRFRAME_CONFIGURATION, MAV_RESULT_ACCEPTED, 100, 0, 1, 1);
        }
        Mavlink_SendMessage(&mavMsgTx, sendCnt);
    }
    /* Landing Gear changing process */
    if(lgModeCurr==LG_MODE_CHANGING)
    {
        Relay_ON();                             // Turn on relay to power on steers
        lgModeCurr = LG_Control(lgPositionCurr);// If changing process finished, lgMode turns 0
        lgDelayCnt = LG_CHANGE_DELAY;           // Ignore cmd for 2s (guard time)
        if(!lgCycleCnt)    PRINTLOG("\r\n [INFO] Landing Gear: Moving %3d%%",lgProgress);
    }
    else Relay_OFF();                           // Turn off relay to power off steers
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

int8_t LG_Step(uint8_t pos,uint8_t type)
{
    int8_t temp;
    uint8_t prog;
    int16_t range;
    if(pos == LG_POS_UP)    prog=lgProgress;
    else                    prog=100-lgProgress;
    
    if(type==LG_STEER_LEFT) range=PUL_LEFT_RANGE;
    else                    range=PUL_RIGHT_RANGE;
    
    switch(prog/10)
    {
        case 0:     temp=range*0.018;   break;
        case 1:     temp=range*0.012;   break;
        case 2:     temp=range*0.008;   break;
        case 3:     temp=range*0.008;   break;
        case 4:     temp=range*0.008;   break;
        case 5:     temp=range*0.010;   break;
        case 6:     temp=range*0.016;   break;
        case 7:     temp=range*0.020;   break;
        case 8:     temp=range*0.018;   break;
        case 9:     temp=range*0.012;   break;
        case 10:    temp=range*0.006;   break; 
        default:    temp=0;
    }
    
    PRINTLOG("\r\n #%d,%d,%d,%d",pos,type,_prog,temp);
    return temp;
}

/**
  * @brief  Landing Gear Control function.
  * @param  pos current position of landing gear
  * @retval new langding gear changing status
  */
uint8_t LG_Control(uint8_t pos)
{
    static uint8_t stage = 0;
    static uint32_t timeTick = 0;

    // Judge
    switch(stage)
    {
        // Change pulse
        case 0x00:
            if(pos == LG_POS_UP)            // Landing Gear Up(1)
            {
                lgPulseL+=LG_Step(pos,LG_STEER_LEFT);
                lgPulseR+=LG_Step(pos,LG_STEER_RIGHT);
                //lgPulseL+=(int8_t)(PUL_LEFT_RANGE*PUL_SCALE_UP);
                //lgPulseR+=(int8_t)(PUL_RIGHT_RANGE*PUL_SCALE_UP);
                lgProgress=(lgPulseL-PUL_LEFT_DOWN)*100/PUL_LEFT_RANGE;
                if(lgPulseL>PUL_LEFT_MAX||lgPulseL<PUL_LEFT_MIN||lgPulseR>PUL_RIGHT_MAX||lgPulseR<PUL_RIGHT_MIN)
                {
                    lgPulseL=PUL_LEFT_UP;       // Ensure safe
                    lgPulseR=PUL_RIGHT_UP;
                    timeTick = HAL_GetTick();
                    stage = 0x01;
                }
            }
            else if(pos == LG_POS_DOWN)     // Landing Gear Down(0)
            {
                lgPulseL-=LG_Step(pos,LG_STEER_LEFT);
                lgPulseR-=LG_Step(pos,LG_STEER_RIGHT);
                //lgPulseL-=(int8_t)(PUL_LEFT_RANGE*PUL_SCALE_DOWN);
                //lgPulseR-=(int8_t)(PUL_RIGHT_RANGE*PUL_SCALE_DOWN);
                lgProgress=(PUL_LEFT_UP-lgPulseL)*100/PUL_LEFT_RANGE;
                if(lgPulseL>PUL_LEFT_MAX||lgPulseL<PUL_LEFT_MIN||lgPulseR>PUL_RIGHT_MAX||lgPulseR<PUL_RIGHT_MIN)
                {
                    lgPulseL=PUL_LEFT_DOWN;
                    lgPulseR=PUL_RIGHT_DOWN;
                    timeTick = HAL_GetTick();
                    stage = 0x01;
                }
            }
            PRINTLOG("\r\n $%4d,%4d",lgPulseL,lgPulseR);  // <Dev> Watch lgPulseL/lgPulseR
            break;

        // Waiting for relay finish
        case 0x01:
            if(HAL_GetTick() - timeTick > LG_RELAY_DELAY)
            {
                stage = 0x00;
                lgProgress = 0;
                if(pos == LG_POS_UP)        LED_ON(LED2);
                else                        LED_OFF(LED2);
                return LG_MODE_STANDBY;
            }
            break;

        default: break;
    }

    // Adjust PWM duty cycle
    hocl.Pulse=lgPulseL;
    HAL_TIM_PWM_ConfigChannel(&htim3,&hocl,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
    hocr.Pulse=lgPulseR;
    HAL_TIM_PWM_ConfigChannel(&htim3,&hocr,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);

    return LG_MODE_CHANGING;
}

void LG_Reset(void)
{
    static uint8_t stage = 0;
    static uint32_t timeTick = 0;

    switch(stage)
    {
        case 0x00:
            Relay_ON();

            hocl.Pulse=PUL_LEFT_DOWN;
            HAL_TIM_PWM_ConfigChannel(&htim3,&hocl,TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
            hocr.Pulse=PUL_RIGHT_DOWN;
            HAL_TIM_PWM_ConfigChannel(&htim3,&hocr,TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);

            timeTick = HAL_GetTick();
            stage = 0x01;

        case 0x01:
            if(HAL_GetTick() - timeTick > LG_RELAY_DELAY)
            {
                Relay_OFF();
                lgPositionCurr = LG_POS_DOWN;
                lgModeCurr = LG_MODE_STANDBY;
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
