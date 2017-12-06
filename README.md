# CompanionComputer_STM32

## Function
- Communicate with Drone Control Board through Mavlink
- Landing Gear Control
- Battery Management (in development)

## Resource
- USART1: F3<->Pixhawk for communication
    - PB6(Tx)-P2.3(USART1-Tx)
    - PB7(Rx)-P2.4(USART1-Rx)
- USART3: F3<->PC for debug
    - PB10(Tx)-P2.5(USART3-Tx)
    - PB11(Rx)-P2.6(USART3-Tx)
- I2C1: Battery communication
    - PB8(SCL)-P6.3/P7.3
    - PB9(SDA)-P6.4/P7.4
- TIM3: Landing gear PWM control (50Hz)
    - Left:PB4(TIM3-CH1)-P3.5(PWM4)
    - Right:PB5(TIM3-CH2)-P3.6(PWM5)
    - Relay:PA11(STEER_PWR)
- TIM6: PWM adjustment (100Hz)
- TIM7: Read & Send Battery message (0.5Hz)

## Milestone
- v0.1 (20171206) : Initial Release
    - Communicate with Drone Control Board through Mavlink
    - Landing Gear Control