/**
  ******************************************************************************
  * File Name       : BattMgmt.h
  * Description     : Battery Management Drivers (through I2C)
  *
  * Version         : v0.3.1
  * Created Date    : 2017.09.25
  * Revised Date    : 2018.03.15
  *
  * Author          : Mingye Xie
  ******************************************************************************
  */

#ifndef __BATTMGMT_H
#define __BATTMGMT_H

#include "stm32f3xx_hal.h"
#include "bsp_usart.h"
#include "bsp_i2c.h"
#include "System.h"
#include "math.h"

typedef struct
{
    char        name[10];
    uint8_t     id;
    uint8_t     status;             // 0x02:BATT_INUSE, 0x01:BATT_ONBOARD
    uint8_t     lostCnt;            // times
    uint8_t     fet;                // 0x20:PWR_ON, 0x10:FET_LOCK
    uint16_t    temperature;        // Celsius
    uint16_t    voltage;            // mV
    int16_t     current;            // mA
    uint8_t     soc;                // %
    uint16_t    remainingCapacity;  // mAh
    uint16_t    fullChargeCapacity; // mAh
    uint16_t    designCapacity;     // mAh
    uint32_t    safetyStatus;
    uint32_t    pfStatus;
    uint32_t    operationStatus;
}BattMsg;

//========== Battery Management Config ==========

#define CNCT_ATTEMPT        3       // times
#define PWRON_ATTEMPT       4       // times
#define ENFET_ATTEMPT       4       // times
#define PWROFF_ATTEMPT      4       // times
#define CNCT_DELAY          20      // ms
#define PWRON_DELAY         2000    // ms
#define PWROFF_DELAY        2000    // ms
#define ENFET_DELAY         2000    // ms
#define VDIFF_INIT_TOL      90      // mv
#define VDIFF_RUNNING_TOL   1000    // mv

#define BATT_FUNC_CYCLE     40      // Hz

//========== Register of Smart Battery ==========
#define BATT_BatteryMode        0x03
#define BATT_Temperature        0x08
#define BATT_Voltage            0x09
#define BATT_Current            0x0A
#define BATT_MaxError           0x0C
#define BATT_RelativeSOC        0x0D
#define BATT_AbsoluteSOC        0x0E
#define BATT_RemainingCapacity  0x0F
#define BATT_FullChargeCapacity 0x10
#define BATT_BatteryStatus      0x16
#define BATT_CycleCount         0x17
#define BATT_DesignCapacity     0x18
#define BATT_SpecificationInfo  0x1A
#define BATT_SerialNumber       0x1C
#define BATT_DeviceChemistry    0x22
#define BATT_SafetyAlert        0x50
#define BATT_SafetyStatus       0x51
#define BATT_PFAlert            0x52
#define BATT_PFStatus           0x53
#define BATT_OperationStatus    0x54
#define BATT_ChargingStatus     0x55
#define BATT_GaugingStatus      0x56

#define	SerialNumber            0x0001
#define	SpecificationInfo       0x0031

#define BATT_PowerControl       0x71
    #define BATT_POWERON        0xAA55
    #define BATT_POWEROFF       0x55AA
    #define BATT_ENABLEFET      0x54AA

#define BATT_FETStatus          0x72
    #define BATT_DUMMY          0xC0
    #define PWR_ON              (1<<5)
    #define FET_LOCK            (1<<4)
    #define PRE_EN              (1<<3)
    #define DFET_EN             (1<<2)
    #define CFET_EN             (1<<1)
    #define FET_EN              (1<<0)

//============BATT_BattStatus============
#define BATT_INUSE              (1<<1)
#define BATT_ONBOARD            (1<<0)

//============battCycleCnt============
#define BATT_SYS_JUDGE          (1<<5)
#define BATT_SYS_BATTB          (1<<4)
#define BATT_SYS_MASK_CMD       0x0F

//============ BattMode ============
#define BATT_NONE               0x00
#define BATT_SINGLE             0x10
#define BATT_DUAL               0x20
#define BATT_DUAL_ONLYONE       0x21    // Only one battery is onboard
#define BATT_DUAL_VDIFF         0x22    // Only one battery can be power on

//============Batt_Measure============
#define BATT_MEAS_FET           0x00
#define BATT_MEAS_VOLT          0x01
#define BATT_MEAS_TEMP          0x02
#define BATT_MEAS_CURR          0x03
#define BATT_MEAS_SOC           0x04
#define BATT_MEAS_RCAP          0x05
#define BATT_MEAS_FCCAP         0x06
#define BATT_MEAS_DCAP          0x07
#define BATT_MEAS_SAFESTA       0x08
#define BATT_MEAS_PFSTA         0x09
#define BATT_MEAS_OPSSTA        0x0A

//============Batt_Init============
#define BATT_INIT_BEGIN         0x00
#define BATT_CNCT_CHECK         0x01
#define BATT_CNCT_WAIT          0x02
#define BATT_VDIFF_CHECK        0x03
#define BATT_PWRON_CHECK        0x04
#define BATT_PWRON_WAIT         0x05
#define BATT_ENFET_CHECK        0x06
#define BATT_ENFET_WAIT         0x07
#define BATT_INIT_CHECK         0x08
#define BATT_PWROFF_CHECK       0x09
#define BATT_PWROFF_WAIT        0x0A

//========== Batt_Mgmt ==========
#define BATT_MGMT_SEND_MSG      0x01
#define BATT_MGMT_CNCT_LOST     0x02
#define BATT_MGMT_VDIFF_CHECK   0x03
#define BATT_MGMT_PWROFF        0x04
#define BATT_MGMT_RECNCT        0x05
#define BATT_MGMT_PWRCHECK      0x06

#define BATT_MGMT_SEND_LOG      0x0E
#define BATT_MGMT_CNCT_COUNT    0x0F


extern uint8_t battPwrOff;
extern uint8_t battReinit;

uint8_t Batt_Init(void);
void Batt_Measure(BattMsg* _batt, uint8_t _cmd);
uint8_t Batt_PowerOff(void);
uint8_t Battery_Management(void);
void Battery_MavlinkPack(mavlink_battery_status_t* mav, BattMsg* batt);
//void Battery_MavlinkPack(mavlink_battery_status_t* mav,uint8_t mode);


uint8_t Batt_WriteByte(uint8_t _addr, uint8_t _reg, uint8_t _data);
uint8_t Batt_WriteWord(uint8_t _addr, uint8_t _reg, uint16_t _data);

uint8_t Batt_ReadByte(uint8_t _addr, uint8_t _reg, uint8_t* _data);
uint8_t Batt_ReadWord(uint8_t _addr, uint8_t _reg, uint16_t* _data);
uint8_t Batt_ReadBlock(uint8_t _addr, uint8_t _reg, uint8_t* _data, uint8_t _num);

#endif /* __BATTMGMT_H */

/******************************END OF FILE******************************/
