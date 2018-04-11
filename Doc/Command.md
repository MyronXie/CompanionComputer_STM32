## Command

Used for [mavlink_msg_stm32_f3_command.h](../Drivers/mavlink-c_library_v2/common/mavlink_msg_stm32_f3_command.h)

### Message Code

|Code   |Name                   |Content                    |
|:-:    |-                      |-                          |
|0x00   |MSG_BLANK              |Blank Message              |
|0x01   |MSG_BATT_INIT          |Battery Init success       |
|0x02   |MSG_BATT_ONBOARD       |Battery onboard            |
|0x03   |MSG_BATT_PWROFF_START  |Start Power Off process    |
|0x04   |MSG_BATT_REINIT        |Battery Reinit process     |
|0x05   |MSG_BATT_PWROFF_END    |Power Off success          |
|0x06   |MSG_LG_RESET           |Landing Gear Auto Reset    |

### Error Code

|Code   |Name                   |Content                    |
|:-:    |-                      |-                          |
|0x10   |ERR_SYS_GENERAL        |System Error               |
|0x11   |ERR_SYS_SERIAL         |Serial Error               |
|-      |-                      |-                          |
|0x20   |ERR_BATT_INIT          |Battery Init Fail          |
|0x21   |ERR_BATT_OFFBOARD      |Battery offboard           |
|0x22   |ERR_BATT_VDIFF         |Voltage mismatch           |
|0x23   |ERR_BATT_POWERON       |Power On Fail              |
|0x24   |ERR_BATT_ENABLEFET     |FET Enable Fail            |
|0x25   |ERR_BATT_POWEROFF      |Power Off Fail             |
|0x26   |ERR_BATT_LOSTPWR       |Lost power in the air      |
|0x27   |ERR_BATT_STILLPWR      |Still power on after pwroff|
|0x28   |ERR_BATT_UNDERVOLT     |Undervoltage               |
|0x29   |ERR_BATT_LOWSOC        |Low battery                |
|0x2A   |ERR_BATT_OVERCURR      |Overcurrent                |
|0x2B   |ERR_BATT_OVERTEMP      |Overtemperature            |
|0x2C   |ERR_BATT_UNDERTEMP     |Undertemperature           |
|0x2D   |ERR_BATT_INNER         |Inner status error         |
|-      |-                      |-                          |


### Command Code

|Code    |Name               |Content                   |
|:-:     |-                  |-                         |
|0x80    |CMD_ARMED          |Drone armed               |
|0x81    |CMD_DISARMED       |Drone disarmed            |