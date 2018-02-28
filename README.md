# CompanionComputer_STM32

### Function
- Communication with FMU through Mavlink (bi-direction)
- Landing Gear Control System
- Battery Management System

### Changelog
* v0.2.1 (20180228) : Improve stability of CC_STM32 System
    * Improve F3 board system
        * Fix problem of USART1-Rx (lost package frequently, become dummy occasionally)
    * Improve Battery Management System
        * Rewrite `SINGLE_BATTERY` logic

* v0.2 (20180205) : Add Battery Management System
    + Add Battery Management System
        + Read status of battery through SMBus (voltage, current, remaining capacity, etc.)
        + Auto power on/off another battery when one battery is power on/off manually
    * Improve communication with FMU through Mavlink
        + Send `BATTERY_STATUS` to FMU for battery status reporting
        + Send `STM32_F3_COMMAND` to FMU for error reporting
    * Improve F3 board system
        * Adjust architecture of code
        + Send system status to FMU and report on QGC

* v0.1.1 (20180104) : Improve stability of Landing Gear System
    * Improve stability of Landing Gear System
        + Ignore frequent commands
        + Put down Landing Gear when communication lost or system reboot
    * Improve communication with FMU through Mavlink
        + Send `COMMAND_ACK` to FMU for response landing gear command
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
* [Command](Doc/Command.md)