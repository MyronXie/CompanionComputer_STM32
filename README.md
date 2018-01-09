## CompanionComputer_STM32

### Function
- Communicate with FC through Mavlink (bi-direction)
- Landing Gear Control System
- Battery Management System (WIP)

### Milestone
- v0.2 (WIP) : Add Battery Management System
    - Add Battery Management System
        - Read status of battery through SMBus (voltage, current, remaining capacity, ...)
        - Send battery massage to FC via Mavlink
    - Sends F3 log to FC and report on QGC

- v0.1.1 (20180104) : Improve stability of Landing Gear System
    - Improve stability of Landing Gear System
        - Ignore frequent commands
        - Put down Landing Gear when communication lost or system reboot
    - Improve stability of USART
    - Sends CMD_ACK msg to FC and report on QGC

- v0.1 (20171221) : Initial Release
    - Add Communication from FC through Mavlink
    - Add Landing Gear Control System