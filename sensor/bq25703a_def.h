/*
 * bq25703a_def.h
 *
 *  Created on: 2020Äê7ÔÂ31ÈÕ
 *      Author: admin
 */

#ifndef BQ25703A_DEF_H_
#include "util_def.h"
#define BQ25703A_DEF_H_
#define BQ25703A_ADDR    0x6B
#define BQ25703A_MID_REG 0x2E

//#define BQ25703A_CHARGE_OPTION0_REG 0x12

#define BQ25703A_CHARGE_CURRENT_REG 0x02
#define BQ25703A_ADC_OPTION_REG     0x3a
#define BQ25703A_CHARGE_OPTION0_REG 0x00
#define BQ25703A_IIN_DPM_REG        0x24
#define BQ25703A_ADCVBUSPSYS_REG    0x26
#define BQ25703A_IIN_HOST_REG       0x0e
#define BQ25703A_CHARGE_VOLTAGE_REG 0x04
#define BQ25703A_CHARGER_STATUS_REG 0x20
#define BQ25703A_MAX_CHARGE_CURRENT 8128
#define BQ25703A_MAX_CHARGE_VOLTAGE 16800
#define BQ25703A_MIN_CHARGE_CURRENT 0

#define AW_BATTERY_1S_4192V  4192
#define AW_BATTERY_2S_8400V  8400
#define AW_BATTERY_3S_12592V 12592
#define AW_BATTERY_4S_16800V 16800
typedef struct ADCData_Typedef {
    uint16_t CMPIN;
    uint16_t VBUS;
    uint16_t PSYS;
    uint16_t IIN;

    uint16_t IDCHG;
    uint16_t ICHG;
    uint16_t VSYS;
    uint16_t VBAT;
} ADCData_t;

typedef struct SettingData_Typedef {
    uint16_t ChargeCurrent;
    uint16_t IIN_DPM;
    uint16_t IIN_HOST;
} SettingData_t;

typedef union ChargeStatus_Typedef {
    uint8_t byte[2];
    struct {
        uint8_t Fault_OTG_UCP : 1;
        uint8_t Fault_OTG_OVP : 1;
        uint8_t Fault_Latchoff : 1;
        uint8_t Reserved : 1;
        uint8_t SYSOVP_STAT : 1;
        uint8_t Fault_ACOC : 1;
        uint8_t Fault_BATOC : 1;
        uint8_t Fault_ACOV : 1;

        uint8_t IN_OTG : 1;
        uint8_t IN_PCHRG : 1;
        uint8_t IN_FCHRG : 1;
        uint8_t IN_IINDPM : 1;
        uint8_t IN_VINDPM : 1;
        uint8_t Reserved2 : 1;
        uint8_t ICO_DONE : 1;
        uint8_t AC_STAT : 1;
    };
} ChargeStatus_t;
#endif /* BQ25703A_DEF_H_ */
