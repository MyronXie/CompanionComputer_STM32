# CompanionComputer_STM32

### Function
- Communicate with FC through Mavlink (bi-direction)
- Landing Gear Control System
- Battery Management System (WIP)

### Milestone
* v0.2 (WIP) : Add Battery Management System
    + Add Battery Management System
        + Read status of battery through SMBus/I2C (voltage, current, remaining capacity, ...)
        + Send `BATTERY_STATUS` msg to FC and report on QGC
        + Auto power on/off another battery when one battery is power on/off manually
    + *Sends F3 log to FC and report on QGC (need to verify msg type)*

* v0.1.1 (20180104) : Improve stability of Landing Gear System
    * Improve stability of Landing Gear System
        + Ignore frequent commands
        + Put down Landing Gear when communication lost or system reboot
        + Send `COMMAND_ACK` msg to FC and report on QGC
    * Improve stability of USART

* v0.1 (20171221) : Initial Release
    + Add Communication from FC through Mavlink
    + Add Landing Gear Control System

### Doc
* [Resource](Doc/Resource.md)