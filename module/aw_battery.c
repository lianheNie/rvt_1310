/*
 * aw_battery.c 电池管理
 *
 *  Created on: 2020年9月27日
 *      Author: admin
 */
#include "aw_battery.h"
#include "util_timer.h"
#include "i2c_dev.h"
#include "bq25703a.h"
#include "util_delay.h"
#ifdef IS_USE_DEV
static aw_i2c_dev_t _bq25703a_dev = {
    .chip_id = 0, .id = 0, .read = i2c_read, .write = i2c_write, .delay_ms = aw_delay_ms};
;
#endif

static aw_battery_status_t _bat_status = {
    .bus_in = 0, .cur = 0, .cur_set = 1000, .v_bat = 0, .v_bus = 0, .bat_max = 0};
#ifdef IS_USE_BAT_CHARGE_TIMER
static Clock_Struct      _bq2570_clockS;
static AwCallBack_void_t _timeout_cb = NULL;
static void              bq2570_timeOutCb(u32 arg) {
    if (NULL != _timeout_cb) {
        _timeout_cb();
    }
}
#define _BQ2570_TIMEOUT 30 * 1000  // 160s
static void _bq25703a_timer_init() {
    Timer_construct(&_bq2570_clockS, bq2570_timeOutCb, _BQ2570_TIMEOUT, _BQ2570_TIMEOUT, false, 0);
    Timer_start(&_bq2570_clockS);  //开定时器
}
#endif
static s8 _bq25703aInit() {
    s8 rslt = 0;
#ifdef IS_USE_DEV
    rslt = bq25703a_init(&_bq25703a_dev, _bat_status.cur_set);
#else
    rslt = bq25703a_init(_bat_status.cur_set);
#endif

#ifdef IS_USE_BAT_CHARGE_TIMER
    _bq25703a_timer_init();
#endif

    return rslt;
}

static void _refresh_bq25703a() {
    // bq25703a_enable_adcInBatteryMode();
    bq25703a_enable_adc();
    bq25703a_refresh_settingData(&_bat_status.cur);
    bq25703a_refresh_adcData(&_bat_status.v_bus, &_bat_status.v_bat, &_bat_status.v_sys);
    bq25703a_get_voltage(&_bat_status.bat_max);
    bq25703a_refresh_chargeStatus(NULL);
    _bat_status.bus_in = bq25703a_get_status_data()->AC_STAT;
}

s8 aw_batttey_update() {
    _refresh_bq25703a();
    if (0 == bq25703a_get_setting_data()->ChargeCurrent && bq25703a_get_status_data()->AC_STAT) {
        bq25703a_set_current(_bat_status.cur_set);
    }
    return 0;
}

#define _MAX_BAT_CURRENT 6000
void aw_battery_set_current(u16 cur) {
    if ((cur > 0 && cur < _MAX_BAT_CURRENT) || cur == 0) {
        _bat_status.cur_set = cur;
        // if (bq25703a_get_status_data()->AC_STAT)
        { bq25703a_set_current(_bat_status.cur_set); }
    }
}
s8 aw_battery_set_voltage(u16 volt) {
    bq25703a_set_current(0);
    bq25703a_set_voltage(volt);
    bq25703a_set_current(_bat_status.cur_set);
    return 0;
}
const aw_battery_status_t* aw_battery_get_status() {
    _refresh_bq25703a();
    return &_bat_status;
}
s8 aw_batttey_enable_set(u8 enable) { return bq25703a_enable_set(enable); }
s8 aw_batttey_enable_get(u8* disable) { return bq25703a_enable_get(disable); }
s8 aw_battery_init(AwCallBack_void_t timeout_cb) {
#ifdef IS_USE_BAT_CHARGE_TIMER
    _timeout_cb = timeout_cb;
#endif
    return _bq25703aInit();
}
