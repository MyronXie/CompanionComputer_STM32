# CompanionComputer_STM32

### Function
- Communication with FMU through Mavlink (bi-direction)
- Landing Gear Control System
- Battery Management System

### To-Do List
* *Rx of USART1 become dummy occasionally (WIP)*

### Changelog
* v0.2 (20180201) : Add Battery Management System
    + Add Battery Management System
        + Read status of battery through SMBus (voltage, current, remaining capacity, etc.)
        + Auto power on/off another battery when one battery is power on/off manually
    * Improve communication with FMU through Mavlink
        + Send `BATTERY_STATUS` to FMU and report on QGC
        + Send `STM32_F3_COMMAND` to FMU

* v0.1.1 (20180104) : Improve stability of Landing Gear System
    * Improve stability of Landing Gear System
        + Ignore frequent commands
        + Put down Landing Gear when communication lost or system reboot
    * Improve communication with FMU through Mavlink
        + Send `COMMAND_ACK` to FMU and report on QGC
    * Improve stability of USART
        + Auto Reinit USART when Rx is dummy

* v0.1 (20171221) : Initial Release
    + Bulid architecture of F3 System
    + Add Landing Gear Control System
        + Control Landing Gear through PWM
    + Add communication with FMU through Mavlink
        + Receive Mavlink message from FMU

### Document
* [Resource](Doc/Resource.md)