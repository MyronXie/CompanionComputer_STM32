## Command

Used for `mavlink_msg_stm32_f3_command.h`

### Msg Code & Error Code

Code    |Name               |Content
-       |-                  |-
**0x00**|**MSG_SYSTEM**     |**System**
0x01    |ERR_SYS_GENERAL    |System Error
0x02    |ERR_SYS_SERIAL     |Serial Error
-       |-                  |-
**0x10**|**MSG_BATTERY**    |**Battery Management**
0x11    |ERR_BATT_OFFBOARD  |Battery offboard
0x12    |ERR_BATT_VDIFF     |Voltage mismatch
0x13    |ERR_BATT_POWERON   |Power On Fail
0x14    |ERR_BATT_ENABLEFET |FET Enable Fail
0x15    |ERR_BATT_INIT      |Battery Init Fail
0x16    |ERR_BATT_POWEROFF  |Power Off Fail
0x17    |ERR_BATT_LOSTAIR   |Battery lost in the air 
-       |-                  |-
**0x20**|**MSG_LANDINGGEAR**|**Landing Gear**
0x21    |ERR_LG_RESET       |Landing Gear Auto Reset

### Action Code

WIP