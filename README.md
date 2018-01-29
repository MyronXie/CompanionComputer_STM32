# CompanionComputer_STM32

### Function
- Communication with FC through Mavlink (bi-direction)
- Landing Gear Control System
- Battery Management System (WIP)

### To-Do List
- *Sends F3 log to FC and report on QGC*
- *Improve stability of USART: Rx occasionally failed and step into ReInit Process*
- *Consider logic between landing gear and battery*

### Changelog
* v0.2 (WIP) : Add Battery Management System
    + Add Battery Management System
        + Read status of battery through SMBus (voltage, current, remaining capacity, etc.)
        + Auto power on/off another battery when one battery is power on/off manually
    * Improve communication with FC through Mavlink
        + Send `BATTERY_STATUS` to FC and report on QGC

* v0.1.1 (20180104) : Improve stability of Landing Gear System
    * Improve stability of Landing Gear System
        + Ignore frequent commands
        + Put down Landing Gear when communication lost or system reboot
    * Improve communication with FC through Mavlink
        + Send `COMMAND_ACK` to FC and report on QGC
    * Improve stability of USART

* v0.1 (20171221) : Initial Release
    + Add Landing Gear Control System
        + Control Landing Gear through PWM
    + Add communication with FC through Mavlink
        + Receive Mavlink message from FC

### Document
* [Resource](Doc/Resource.md)