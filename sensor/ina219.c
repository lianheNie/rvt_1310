/*
 * ina219.c
 *
 *  Created on: 2020年9月21日
 *      Author: admin
 */

/*
 * ina219.c
 *
 *  Created on: 2020年7月31日
 *      Author: admin
 */

/*
 * ina219.c
 *
 *  Created on: 2020年8月12日
 *      Author: york_
 */
#include "ina219.h"
#include "ina219_def.h"
#include "i2c_driver.h"
#include "util_printf.h"

#ifdef IS_USE_DEV
static aw_i2c_dev_t *_ina219_dev = NULL;
#else
#define _i2c_read  i2c_read
#define _i2c_write i2c_write
#define _delay_ms  aw_delay_ms
#endif
static uint8_t _ina219_i2c_addr;
#define MAX_ShuntVoltage 0.04f
static float _ina219_shuntresistor = 0.01f;
static float _current_LSB          = 0;
#define _INA219_PARA 0.04096f
s8 ina219_config(void) {
    // Set Config register to take into account the settings above
    uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V | INA219_CONFIG_GAIN_1_40MV |
                      INA219_CONFIG_BADCRES_12BIT | INA219_CONFIG_SADCRES_12BIT_1S_532US |
                      INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
    s8 rslt = 0;
#ifdef IS_USE_DEV
    if (_ina219_dev == NULL || _ina219_dev->write == NULL) {
        return -2;
    }
#endif
    u8 buf[2] = {0};
    buf[0]    = (config | 0xFF00) >> 8;
    buf[1]    = (config | 0x00FF);
#ifdef IS_USE_DEV
    rslt = _ina219_dev->write(_ina219_i2c_addr, INA219_REG_CONFIG, buf, 2);
#else
    rslt = _i2c_write(_ina219_i2c_addr, INA219_REG_CONFIG, buf, 2);
#endif
    float _cur = MAX_ShuntVoltage / _ina219_shuntresistor;

    _current_LSB = _cur / (32767);

    float _cal = ((_INA219_PARA) / (_current_LSB * _ina219_shuntresistor));

    u16 _calVal = (u16)(_cal);
    buf[0]      = (_calVal | 0xFF00) >> 8;
    buf[1]      = (_calVal | 0x00FF);

#ifdef IS_USE_DEV
    rslt = _ina219_dev->write(_ina219_i2c_addr, INA219_REG_CALIBRATION, buf, 2);
#else
    rslt = _i2c_write(_ina219_i2c_addr, INA219_REG_CALIBRATION, buf, 2);
#endif

    return rslt;
}
#ifdef IS_USE_DEV
s8 ina219_init(aw_i2c_dev_t *ina219_dev, float shuntresistor) {
    _ina219_i2c_addr      = INA219_ADDR;
    _ina219_shuntresistor = shuntresistor;
    _ina219_dev           = ina219_dev;
    if (_ina219_dev != NULL) {
        return 0;
    } else {
        return -2;
    }
}
#else
s8 ina219_init(float shuntresistor) {
    _ina219_i2c_addr = INA219_ADDR;
    _ina219_shuntresistor = shuntresistor;
    return 0;
}
#endif

s8 ina219_getBusVoltage_raw(s16 *busV) {
    uint16_t value;
    s8       rslt = 0;
#ifdef IS_USE_DEV
    if (_ina219_dev == NULL || _ina219_dev->read == NULL) {
        return -2;
    }
#endif
    u8 reg    = INA219_REG_BUSVOLTAGE;
    u8 buf[2] = {0};

#ifdef IS_USE_DEV
    rslt = _ina219_dev->read(_ina219_i2c_addr, reg, buf, 2);
#else
    rslt = _i2c_read(_ina219_i2c_addr, reg, buf, 2);
#endif

    if (rslt == 0) {
        value = buf[0] << 8 | buf[1];
        value = (value >> 3) * 4;
        *busV = value;
    }
    return rslt;
}

s8 ina219_getShuntVoltage_raw(float *shutV) {
    uint16_t value;
    s8       rslt = 0;
#ifdef IS_USE_DEV
    if (_ina219_dev == NULL || _ina219_dev->read == NULL) {
        return -2;
    }
#endif
    u8 reg    = INA219_REG_SHUNTVOLTAGE;
    u8 buf[2] = {0};
#ifdef IS_USE_DEV
    rslt = _ina219_dev->read(_ina219_i2c_addr, reg, buf, 2);
#else
    rslt = _i2c_read(_ina219_i2c_addr, reg, buf, 2);
#endif
    if (rslt == 0) {
        value  = (buf[0] << 8 | buf[1]);
        *shutV = (float)(value) / 100.0f;
    }
    return rslt;
}

s8 ina219_getCurrent_raw(u16 *cur) {
    uint16_t value;
    s8       rslt = 0;
#ifdef IS_USE_DEV
    if (_ina219_dev == NULL || _ina219_dev->read == NULL) {
        return -2;
    }
#endif
    u8 reg    = INA219_REG_CURRENT;
    u8 buf[2] = {0};
#ifdef IS_USE_DEV
    rslt = _ina219_dev->read(_ina219_i2c_addr, reg, buf, 2);
#else
    rslt = _i2c_read(_ina219_i2c_addr, reg, buf, 2);
#endif
    if (rslt == 0) {
        value = buf[0] << 8 | buf[1];
        *cur  = value;
    }
    return rslt;
}

s8 ina219_getCurrent_mA_f(float *cur) {
    // u16   raw   = 0;
    float shutV = 0;
    ina219_getShuntVoltage_raw(&shutV);
    *cur = shutV / _ina219_shuntresistor;

    //    ina219_getCurrent_raw(&raw);
    //    *cur = raw * _current_LSB * 1000;
    return 0;
}

s8 ina219_get_current_mA(u16 *cur) {
    float shutV = 0;
    ina219_getShuntVoltage_raw(&shutV);
    *cur = (shutV / _ina219_shuntresistor);
    return 0;
}

s8 ina219_getPower_raw(float *power) {
    uint16_t value;
    s8       rslt = 0;
#ifdef IS_USE_DEV
    if (_ina219_dev == NULL || _ina219_dev->read == NULL) {
        return -2;
    }
#endif
    u8 reg    = INA219_REG_POWER;
    u8 buf[2] = {0};
#ifdef IS_USE_DEV
    rslt = _ina219_dev->read(_ina219_i2c_addr, reg, buf, 2);
#else
    rslt = _i2c_read(_ina219_i2c_addr, reg, buf, 2);
#endif
    if (rslt == 0) {
        value = (buf[0] << 8 | buf[1]) * 2;

        *power = value;
    }
    return rslt;
}

s8 ina219_getManufID(u16 *mID) {
    uint16_t value;
    s8       rslt = 0;
#ifdef IS_USE_DEV
    if (_ina219_dev == NULL || _ina219_dev->read == NULL) {
        return -2;
    }
#endif
    u8 reg    = 0x00;
    u8 buf[2] = {0};
#ifdef IS_USE_DEV
    rslt = _ina219_dev->read(_ina219_i2c_addr, reg, buf, 2);
#else
    rslt = _i2c_read(_ina219_i2c_addr, reg, buf, 2);
#endif
    if (rslt == 0) {
        value = buf[0] << 8 | buf[1];
        *mID  = value;
    }
    return rslt;
}
