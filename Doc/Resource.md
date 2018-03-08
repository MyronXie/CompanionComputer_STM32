## Resource

## Hardware
- Externsion Board Rev2.0 (STM32F334C6)

### Communication

|Chip|Function  | |Board|Name       |Remark |
|-   |-         |-|-    |-          |-      |
|    |          | |P2.1 |+3.3V      |       |
|    |          | |P2.2 |GND        |       |
|PB7 |USART1_Rx | |P2.3 |USART1_Rx  |       |
|PB6 |USART1_Tx | |P2.4 |USART1_Tx  |       |
|-   |-         |-|-    |-          |-      |
|    |          | |P3.1 |+3.3V      |       |
|    |          | |P3.2 |GND        |       |
|PB11|USART3_Rx | |P3.3 |USART3_Rx  |       |
|PB10|USART3_Tx | |P3.4 |USART3_Tx  |       |

### Landing Gear

|Chip|Function  | |Board|Name       |Remark                 |
|-   |-         |-|-    |-          |-                      |
|PB4 |TIM3_CH1  | |P8.1 |PWM1       |Left                   |
|    |          | |P8.2 |7V_OUT     |                       |
|    |          | |P8.3 |GND        |                       |
|-   |-         |-|-    |-          |-                      |
|PB5 |TIM3_CH2  | |P9.1 |PWM2       |Right                  |
|    |          | |P9.2 |7V_OUT     |                       |
|    |          | |P9.3 |GND        |                       |
|-   |-         |-|-    |-          |-                      |
|PC13|PC13      | |     |STEER_PWR  |Relay **PA11->PC13**   |

### Battery Management

|Chip|Function  | |Board    |Name       |Remark |
|-   |-         |-|-        |-          |-      |
|    |          | |P6.1/P7.1|+3.3V      |       |
|    |          | |P6.2/P7.2|GND        |       |
|PB8 |I2C_SCL   | |P6.3/P7.3|BAT_SCL    |       |
|PB9 |I2C_SDA   | |P6.4/P7.4|BAT_SDA    |       |

### Current Monitor

|Chip|Function  | |Board|Name       |Remark |
|-   |-         |-|-    |-          |-      |
|PA0 |ADC1_IN1  | |P5.7 |ESC_CURNT1 |       |
|PA1 |ADC1_IN2  | |P5.6 |ESC_CURNT2 |       |
|PA2 |ADC1_IN3  | |P5.5 |ESC_CURNT3 |       |
|PA3 |ADC1_IN4  | |P5.4 |ESC_CURNT4 |       |
|PB0 |ADC1_IN11 | |P5.3 |ESC_CURNT5 |       |
|PB1 |ADC1_IN12 | |P5.2 |ESC_CURNT6 |       |
|    |          | |P5.1 |GND        |       |

### LED

|Chip|Function  | |Board|Name       |Remark     |
|-   |-         |-|-    |-          |-          |
|PB12|PB12      | |     |LED1       |           |
|PB13|PB13      | |     |LED2       |           |
|PB14|PB14      | |     |LED3       |           |
|PB15|PB15      | |     |LED4       |           |
|    |          | |     |LED5       |Steer PWR  |
|    |          | |     |LED6       |Board PWR  |

### Unused PWM Pin

|Chip|Function  | |Board|Name       |Remark |
|-   |-         |-|-    |-          |-      |
|    |          | |P4.1 |GND        |       |
|PA8 |TIM1_CH1  | |P4.2 |PWM5       |       |
|PA9 |TIM1_CH2  | |P4.3 |PWM6       |       |
|PA10|TIM1_CH3  | |P4.4 |PWM7       |       |
|PA15|TIM2_CH1  | |P4.5 |PWM3       |       |
|PB3 |TIM2_CH2  | |P4.6 |PWM4       |       |

## Software

### System
- TIM2: System Management (1Hz)
- IWDG: Watchdog
- Flash: Save params of system
    - 0: lgPositionCurr
    - 1: lgChangeStatusCurr

### Communication
- USART1: F3<->FMU  for communication   (57600bps)
- USART3: F3<->PC   for debug           (256000bps)

### Landing Gear
- TIM3: Landing gear PWM control (50Hz)
- TIM6: Landing gear PWM adjustment (100Hz)

### Battery Management
- I2C1: Battery communication
- TIM7: Read & Send Battery message (40Hz)

### Current Monitor
- ADC1: Read current of ESC
- TIM15: Send ESC Current Message (20Hz)

### Mavlink
Based on mavlink-c_library_v2 from Gitlab v20180131 (`84a046bc`)