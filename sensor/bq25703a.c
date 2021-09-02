/*
 * bq25703a.c
 *
 *  Created on: 2020Äê7ÔÂ31ÈÕ
 *      Author: admin
 */

#include "bq25703a.h"
#include "i2c_driver.h"
#include "util_printf.h"
#include "i2c_dev.h"
#include "util_other.h"

#ifdef IS_USE_DEV
static aw_i2c_dev_t *_dev = NULL;
#else
#define _i2c_read  i2c_read
#define _i2c_write i2c_write
#define _delay_ms  aw_delay_ms
#endif
#define _bq25703a_enable    1
#define _bq25703a_disenable 0
static ADCData_t      _adcData;
static SettingData_t  _settingData;
static ChargeStatus_t _chargeStatus;
static bool           _is_enable = false;
const SettingData_t * bq25703a_get_setting_data() { return &_settingData; }
const ADCData_t *     bq25703a_get_adc_data() { return &_adcData; }
const ChargeStatus_t *bq25703a_get_status_data() { return &_chargeStatus; }

#define _BQ25703a_IINHOST_CUR_MAX 6000

static s8 _bq25703a_init(u16 current) {
    s8 rslt = 0;
    // bq25703a_enable_set(_bq25703a_disenable);
    // rslt = bq25703a_set_voltage(AW_BATTERY_3S_12592V);
    //    if (rslt != 0) {
    //        return rslt;
    //    }
    rslt = bq25703a_enable_adcInBatteryMode();
    if (rslt != 0) {
        return rslt;
    }
    rslt = bq25703a_set_IINHost(_BQ25703a_IINHOST_CUR_MAX);
    if (rslt != 0) {
        return rslt;
    }

    //    rslt = bq25703a_set_current(current);
    //    if (rslt != 0) {
    //        return rslt;
    //    }

    rslt = bq25703a_refresh_settingData(NULL);
    if (rslt != 0) {
        return rslt;
    }
    rslt = bq25703a_refresh_adcData(NULL, NULL, NULL);
    if (rslt != 0) {
        return rslt;
    }
    bq25703a_set_current(10);
    return rslt;
}
#ifdef IS_USE_DEV
int8_t bq25703a_init(aw_i2c_dev_t *i2c_dev, u16 current) {
    s8 rslt   = 0;
    u8 chipId = 0;
    rslt      = i2c_dev_check_null(i2c_dev);
    if (rslt == 0) {
        _dev = i2c_dev;
        rslt = bq25703a_get_id(&chipId);
        if (rslt != 0) {
            return rslt;
        }
        rslt = _bq25703a_init(current);
        if (rslt != 0) {
            return rslt;
        }
    } else {
        rslt = -2;
    }
    return rslt;
}
#else
int8_t bq25703a_init(u16 current) {
    s8 rslt   = 0;
    u8 chipId = 0;
    rslt      = bq25703a_get_id(&chipId);
    if (rslt != 0) {
        return rslt;
    }
    rslt = _bq25703a_init(current);
    if (rslt != 0) {
        return rslt;
    }
    return rslt;
}
#endif
int8_t bq25703a_get_id(u8 *chipId) {
    s8 rslt = 0;
#ifdef IS_USE_DEV
    rslt = i2c_dev_check_null(_dev);
    if (rslt != 0) {
        return -1;
    }
#endif
    u8 buf[2] = {0};
#ifdef IS_USE_DEV
    rslt = _dev->read(BQ25703A_ADDR, BQ25703A_MID_REG, buf, 1);
#else
    rslt = _i2c_read(BQ25703A_ADDR, BQ25703A_MID_REG, buf, 1);
#endif
    if (rslt == 0) {
        *chipId = buf[0];
#ifdef IS_USE_DEV
        _dev->chip_id = *chipId;
#endif

    } else {
        util_printf("BQ25703A communication err rslt=%d\r\n", rslt);
    }

    return rslt;
}

int8_t bq25703a_set_current(u16 current) {
    if (current > BQ25703A_MAX_CHARGE_CURRENT) {
        return -3;
    }
    s8 rslt = 0;
    if (!_is_enable) {
        return -1;
    }
    if (0 == current) {
        _is_enable = false;
    }
#ifdef IS_USE_DEV
    rslt = i2c_dev_check_null(_dev);
    if (rslt != 0) {
        return -1;
    }
#endif
    u8 buf[2] = {0};
    buf[0]    = (current & 0xFF);
    buf[1]    = (current >> 8) & 0xFF;
#ifdef IS_USE_DEV
    rslt = _dev->write(BQ25703A_ADDR, BQ25703A_CHARGE_CURRENT_REG, buf, 2);
#else
    rslt = _i2c_write(BQ25703A_ADDR, BQ25703A_CHARGE_CURRENT_REG, buf, 2);
#endif
    return rslt;
}
static s8 _bq25703a_set_min_system_voltage(u16 volt) {
    if (volt > BQ25703A_MAX_CHARGE_VOLTAGE || volt <= 0) {
        return -3;
    }
    s8 rslt = 0;
#ifdef IS_USE_DEV
    rslt = i2c_dev_check_null(_dev);
    if (rslt != 0) {
        return -1;
    }
#endif
    u8 val = volt / 256;

#ifdef IS_USE_DEV
    rslt = _dev->write(BQ25703A_ADDR, BQ25703A_CHARGE_VOLTAGE_REG, &val, 1);
#else
    rslt = _i2c_write(BQ25703A_ADDR, BQ25703A_CHARGE_VOLTAGE_REG, &val, 1);
#endif
    return rslt;
}
s8 bq25703a_set_voltage(u16 volt) {
    if (volt > BQ25703A_MAX_CHARGE_VOLTAGE || volt <= 0) {
        return -3;
    }
    _is_enable = true;
    s8 rslt    = 0;
#ifdef IS_USE_DEV
    rslt = i2c_dev_check_null(_dev);
    if (rslt != 0) {
        return -1;
    }
#endif
    _bq25703a_set_min_system_voltage(volt);
    u8 buf[2] = {0};
    buf[0]    = (volt & 0xFF);
    buf[1]    = (volt >> 8) & 0xFF;
#ifdef IS_USE_DEV
    rslt = _dev->write(BQ25703A_ADDR, BQ25703A_CHARGE_VOLTAGE_REG, buf, 2);
#else
    rslt = _i2c_write(BQ25703A_ADDR, BQ25703A_CHARGE_VOLTAGE_REG, buf, 2);
#endif
    return rslt;
}
s8 bq25703a_get_voltage(u16 *volt) {
    s8 rslt = 0;
#ifdef IS_USE_DEV
    rslt = i2c_dev_check_null(_dev);
    if (rslt != 0) {
        return -1;
    }
#endif
    u8 buf[2] = {0};
#ifdef IS_USE_DEV
    rslt = _dev->read(BQ25703A_ADDR, BQ25703A_CHARGE_VOLTAGE_REG, buf, 2);
#else
    rslt = _i2c_read(BQ25703A_ADDR, BQ25703A_CHARGE_VOLTAGE_REG, buf, 2);
#endif
    u16 v = buf[1] << 8 | buf[0];
    *volt = v;
    return rslt;
}

int8_t bq25703a_enable_adc() {
    s8 rslt = 0;
#ifdef IS_USE_DEV
    rslt = i2c_dev_check_null(_dev);
    if (rslt != 0) {
        return -1;
    }
#endif
    u8 buf[2] = {0};
    buf[0]    = 0xFF;
    buf[1]    = 0b01100000;
#ifdef IS_USE_DEV
    rslt = _dev->write(BQ25703A_ADDR, BQ25703A_ADC_OPTION_REG, buf, 2);
#else
    rslt = _i2c_write(BQ25703A_ADDR, BQ25703A_ADC_OPTION_REG, buf, 2);
#endif
    return rslt;
}
int8_t bq25703a_enable_adcInBatteryMode() {
    s8 rslt = 0;
    rslt    = bq25703a_enable_adc();
    if (rslt == 0) {
        u8 val = 0x02;
#ifdef IS_USE_BAT_CHARGE_TIMER
        val = 0x62;
#endif
#ifdef IS_USE_DEV
        rslt = _dev->write(BQ25703A_ADDR, BQ25703A_CHARGE_OPTION0_REG + 1, &val, 1);
#else
        rslt = _i2c_write(BQ25703A_ADDR, BQ25703A_CHARGE_OPTION0_REG + 1, &val, 1);
#endif
    }
    return rslt;
}

s8 bq25703a_enable_set(u8 enable) {
    s8 rslt = 0;
    u8 val  = 0;
    bq25703a_enable_get(&val);
    if (enable == 0 || enable == 1) {
        if (enable) {
            val &= ~(1 << 0);
        } else {
            val |= 0x01;
        }
#ifdef IS_USE_DEV
        rslt = _dev->write(BQ25703A_ADDR, BQ25703A_CHARGE_OPTION0_REG, &val, 1);
#else
        rslt = _i2c_write(BQ25703A_ADDR, BQ25703A_CHARGE_OPTION0_REG, &val, 1);
#endif
    }
    return rslt;
}
s8 bq25703a_enable_get(u8 *disable) {
    s8 rslt = 0;
#ifdef IS_USE_DEV
    rslt = _dev->read(BQ25703A_ADDR, BQ25703A_CHARGE_OPTION0_REG, disable, 1);
#else
    rslt = _i2c_read(BQ25703A_ADDR, BQ25703A_CHARGE_OPTION0_REG, disable, 1);
#endif

    return rslt;
}
int8_t bq25703a_set_IINHost(u16 current) {
    s8 rslt = 0;
#ifdef IS_USE_DEV
    rslt = i2c_dev_check_null(_dev);
    if (rslt != 0) {
        return -1;
    }
#endif
    u8 val = current / 50;
#ifdef IS_USE_DEV
    rslt = _dev->write(BQ25703A_ADDR, BQ25703A_IIN_HOST_REG + 1, &val, 1);
#else
    rslt = _i2c_write(BQ25703A_ADDR, BQ25703A_IIN_HOST_REG + 1, &val, 1);
#endif
    return rslt;
}

int8_t bq25703a_refresh_adcData(u16 *vBus, u16 *vBat, u16 *vSys) {
    s8 rslt = 0;
#ifdef IS_USE_DEV
    rslt = i2c_dev_check_null(_dev);
    if (rslt != 0) {
        return -1;
    }
#endif
    u8 buf[8] = {0};
#ifdef IS_USE_DEV
    rslt = _dev->read(BQ25703A_ADDR, BQ25703A_ADCVBUSPSYS_REG, buf, 8);
#else
    rslt = _i2c_read(BQ25703A_ADDR, BQ25703A_ADCVBUSPSYS_REG, buf, 8);
#endif
    if (rslt == 0) {
        _adcData.PSYS  = buf[0] * 12;
        _adcData.VBUS  = buf[1] * 64 + 3200;
        _adcData.IDCHG = buf[2] * 256;
        _adcData.ICHG  = buf[3] * 64;
        _adcData.CMPIN = buf[4] * 12;
        _adcData.IIN   = buf[5] * 50;
        _adcData.VBAT  = buf[6] * 64 + 2880;
        _adcData.VSYS  = buf[7] * 64 + 2880;
        if (vBus != NULL && vBat != NULL && vSys != NULL) {
            *vBus = _adcData.VBUS;
            *vBat = _adcData.VBAT;
            *vSys = _adcData.VSYS;
        }
    }
#ifdef IS_USE_DEV
    rslt = _dev->read(BQ25703A_ADDR, BQ25703A_ADC_OPTION_REG, buf, 2);
#else
    rslt = _i2c_read(BQ25703A_ADDR, BQ25703A_ADC_OPTION_REG, buf, 2);
#endif
    return rslt;
}
int8_t bq25703a_refresh_settingData(u16 *chargeCurrent) {
    s8 rslt = 0;
#ifdef IS_USE_DEV
    rslt = i2c_dev_check_null(_dev);
    if (rslt != 0) {
        return -1;
    }
#endif
    u8 buf[2] = {0};
#ifdef IS_USE_DEV
    rslt = _dev->read(BQ25703A_ADDR, BQ25703A_CHARGE_CURRENT_REG, buf, 2);
#else
    rslt = _i2c_read(BQ25703A_ADDR, BQ25703A_CHARGE_CURRENT_REG, buf, 2);
#endif
    if (rslt != 0) return -1;
    _settingData.ChargeCurrent = (buf[1] << 8) | buf[0];
    if (chargeCurrent != NULL) {
        *chargeCurrent = _settingData.ChargeCurrent;
    }
#ifdef IS_USE_DEV
    rslt = _dev->read(BQ25703A_ADDR, BQ25703A_IIN_DPM_REG, buf, 2);
#else
    rslt = _i2c_read(BQ25703A_ADDR, BQ25703A_IIN_DPM_REG, buf, 2);
#endif
    if (rslt != 0) return -1;
    _settingData.IIN_DPM = buf[1] * 50;
#ifdef IS_USE_DEV
    rslt = _dev->read(BQ25703A_ADDR, BQ25703A_IIN_HOST_REG, buf, 2);
#else
    rslt = _i2c_read(BQ25703A_ADDR, BQ25703A_IIN_HOST_REG, buf, 2);
#endif
    if (rslt != 0) return -1;
    _settingData.IIN_HOST = buf[1] * 50;
    return rslt;
}

int8_t bq25703a_refresh_chargeStatus(ChargeStatus_t *_status) {
    s8 rslt = 0;
#ifdef IS_USE_DEV
    rslt = i2c_dev_check_null(_dev);
    if (rslt != 0) {
        return -1;
    }
    rslt = _dev->read(BQ25703A_ADDR, BQ25703A_CHARGER_STATUS_REG, _chargeStatus.byte, 2);
#else
    rslt = _i2c_read(BQ25703A_ADDR, BQ25703A_CHARGER_STATUS_REG, _chargeStatus.byte, 2);
#endif
    if (_status != NULL) {
        *_status = _chargeStatus;
    }
    return rslt;
}
