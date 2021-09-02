/*
 * aw_ammeter.c
 *
 *  Created on: 2020Äê9ÔÂ27ÈÕ
 *      Author: admin
 */

#include "aw_ammeter.h"
#include "util_delay.h"
#include "i2c_dev.h"
#include "ina3221.h"
#include "ina219.h"

#define _INA219_RESITOR  0.01f
#define _INA3221_RESITOR 0.01f

#ifdef IS_USE_DEV
static aw_i2c_dev_t _ina3221_dev = {
    .chip_id = 0, .id = 0, .read = i2c_read, .write = i2c_write, .delay_ms = aw_delay_ms};
static aw_i2c_dev_t _ina219_dev = {
    .chip_id = 0, .id = 0, .read = i2c_read, .write = i2c_write, .delay_ms = aw_delay_ms};
#endif

static s8 _ina219Init() {
    s8 rslt = 0;
#ifdef IS_USE_DEV
    rslt = ina219_init(&_ina219_dev, _INA219_RESITOR);  // 10mr
    rslt = ina219_getManufID(&_ina219_dev.chip_id);
#else
    rslt = ina219_init(_INA219_RESITOR);  // 10mr
#endif
    rslt        = ina219_config();
    u16 cur_12v = 0;
    ina219_get_current_mA(&cur_12v);
    return rslt;
}

static s8 _ina3221Init() {
    s8 rslt = 0;
#ifdef IS_USE_DEV
    rslt = ina3221_init(&_ina3221_dev, _INA3221_RESITOR);
    rslt = ina3221_getManufID(&_ina3221_dev.chip_id);
#else
    rslt = ina3221_init(_INA3221_RESITOR);
#endif
    rslt       = ina3221_config();
    u16 cur_4v = 0;
    ina3221_get_current_mA(1, &cur_4v);
    return rslt;
}

s8 aw_ammeter_init() {
    s8 res = 0;
    _ina3221Init();
    _ina219Init();
    return res;
}

s8 aw_ammeter_get(u16* cur_4v, u16* cur_5v, u16* cur_12v) {
    s8 res = 0;
    ina3221_get_current_mA(1, cur_4v);
    ina3221_get_current_mA(3, cur_5v);
    ina219_get_current_mA(cur_12v);
    return res;
}
