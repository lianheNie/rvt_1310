/*
 * ina3221.c
 *
 *  Created on: 2020Äê7ÔÂ31ÈÕ
 *      Author: admin
 */

#include "ina3221.h"
#include "ina3221_def.h"
#include "i2c_driver.h"
#include "util_printf.h"

#ifdef IS_USE_DEV
static aw_i2c_dev_t *_ina3221_dev;
#else
#define _i2c_read  i2c_read
#define _i2c_write i2c_write
#define _delay_ms  aw_delay_ms
#endif

static uint8_t _ina3221_i2c_addr;
static float   _ina3221_shuntresistor;

s8 ina3221_config(void) {
    // Set Config register to take into account the settings above
    uint16_t config = INA3221_CONFIG_ENABLE_CHAN1 | INA3221_CONFIG_ENABLE_CHAN2 |
                      INA3221_CONFIG_ENABLE_CHAN3 | INA3221_CONFIG_AVG1 | INA3221_CONFIG_VBUS_CT2 |
                      INA3221_CONFIG_VSH_CT2 | INA3221_CONFIG_MODE_2 | INA3221_CONFIG_MODE_1 |
                      INA3221_CONFIG_MODE_0;
    s8 rslt = 0;
#ifdef IS_USE_DEV
    if (_ina3221_dev == NULL || _ina3221_dev->write == NULL) {
        return -2;
    }
#endif
    u8 reg    = INA3221_REG_CONFIG;
    u8 buf[2] = {0};
    buf[0]    = (config | 0xFF00) >> 8;
    buf[1]    = (config | 0x00FF);
#ifdef IS_USE_DEV
    rslt = _ina3221_dev->write(_ina3221_i2c_addr, reg, buf, 2);
#else
    rslt = _i2c_write(_ina3221_i2c_addr, reg, buf, 2);
#endif
    return rslt;
}

/**************************************************************************/
/*!
    @brief  Instantiates a new AW_INA3221 class
*/
/**************************************************************************/
#ifdef IS_USE_DEV
s8 ina3221_init(aw_i2c_dev_t *ina3221_dev, float shuntresistor) {
    _ina3221_i2c_addr      = INA3221_ADDR;
    _ina3221_shuntresistor = shuntresistor;
    _ina3221_dev           = ina3221_dev;
    if (_ina3221_dev != NULL) {
        return 0;
    } else {
        return -2;
    }
}
#else
s8 ina3221_init(float shuntresistor) {
    _ina3221_i2c_addr = INA3221_ADDR;
    _ina3221_shuntresistor = shuntresistor;
    return 0;
}
#endif
/**************************************************************************/
/*!
    @brief  Gets the raw bus voltage (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
s8 ina3221_getBusVoltage_raw(int channel, s16 *busV) {
    uint16_t value;
    s8       rslt = 0;
#ifdef IS_USE_DEV
    if (_ina3221_dev == NULL || _ina3221_dev->read == NULL) {
        return -2;
    }
#endif

    u8 reg    = INA3221_REG_BUSVOLTAGE_1 + (channel - 1) * 2;
    u8 buf[2] = {0};
#ifdef IS_USE_DEV
    rslt = _ina3221_dev->read(_ina3221_i2c_addr, reg, buf, 2);
#else
    rslt = _i2c_read(_ina3221_i2c_addr, reg, buf, 2);
#endif
    if (rslt == 0) {
        value = buf[0] << 8 | buf[1];
        if (value & 0x8000) value = ~(value - 1);
        *busV = (s16)(value);
    }
    return rslt;
}

/**************************************************************************/
/*!
    @brief  Gets the raw shunt voltage (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
s8 ina3221_getShuntVoltage_raw(int channel, s16 *shutV) {
    uint16_t value;
    s8       rslt = 0;
#ifdef IS_USE_DEV
    if (_ina3221_dev == NULL || _ina3221_dev->read == NULL) {
        return -2;
    }
#endif
    u8 reg    = INA3221_REG_SHUNTVOLTAGE_1 + (channel - 1) * 2;
    u8 buf[2] = {0};
#ifdef IS_USE_DEV
    rslt = _ina3221_dev->read(_ina3221_i2c_addr, reg, buf, 2);
#else
    rslt = _i2c_read(_ina3221_i2c_addr, reg, buf, 2);
#endif
    if (rslt == 0) {
        value = buf[0] << 8 | buf[1];
        if (value & 0x8000) value = ~(value - 1);
        *shutV = (s16)(value);
    }
    return rslt;
}

/**************************************************************************/
/*!
    @brief  Gets the shunt voltage in mV (so +-168.3mV)
*/
/**************************************************************************/
s8 ina3221_getShuntVoltage_mV(int channel, float *shuntV) {
    int16_t value;
    s8      rslt = 0;
    rslt         = ina3221_getShuntVoltage_raw(channel, &value);
    if (rslt == 0) {
        *shuntV = value * 0.005;
    }
    return rslt;
}

/**************************************************************************/
/*!
    @brief  Gets the shunt voltage in volts
*/
/**************************************************************************/
s8 ina3221_getBusVoltage_V(int channel, float *busV) {
    int16_t value;
    s8      rslt = 0;
    rslt         = ina3221_getBusVoltage_raw(channel, &value);
    if (rslt == 0) {
        *busV = value * 0.001;
    }
    return rslt;
}

/**************************************************************************/
/*!
    @brief  Gets the current value in mA, taking into account the
            config settings and current LSB
*/
/**************************************************************************/
s8 ina3221_getCurrent_mA(int channel, float *cur) {
    float valueDec;
    s8    rslt = 0;
    rslt       = ina3221_getShuntVoltage_mV(channel, &valueDec);
    if (rslt == 0) {
        *cur = valueDec / _ina3221_shuntresistor;
    }
    return rslt;
}

s8 ina3221_get_current_mA(int channel, u16 *cur) {
    float valueDec;
    s8    rslt = 0;
    rslt       = ina3221_getShuntVoltage_mV(channel, &valueDec);
    if (rslt == 0) {
        *cur = (u16)(valueDec / _ina3221_shuntresistor);
    }
    return rslt;
}
/**************************************************************************/
/*!
    @brief  Gets the Manufacturers ID
*/
/**************************************************************************/
s8 ina3221_getManufID(u16 *mID) {
    uint16_t value;
    s8       rslt = 0;
#ifdef IS_USE_DEV
    if (_ina3221_dev == NULL || _ina3221_dev->read == NULL) {
        return -2;
    }
#endif
    u8 reg    = 0xFE;
    u8 buf[2] = {0};
#ifdef IS_USE_DEV
    rslt = _ina3221_dev->read(_ina3221_i2c_addr, reg, buf, 2);
#else
    rslt = _i2c_read(_ina3221_i2c_addr, reg, buf, 2);
#endif
    if (rslt == 0) {
        value = buf[0] << 8 | buf[1];
        *mID  = (s16)(value);
    }
    return rslt;
}
