/*
 * aw_battery.h
 *
 *  Created on: 2020年9月27日
 *      Author: admin
 */

#ifndef MODULE_AW_BATTERY_H_
#define MODULE_AW_BATTERY_H_
#include "util_def.h"

typedef struct _aw_battery_status_t {
    u16 cur;      //当前充电电流
    u16 cur_set;  //设定充电电流
    u16 bat_max;  //充电最大电压
    u16 v_bat;    //电池电压
    u16 v_bus;    //输入电压
    u16 v_sys;    //
    u8  bus_in;   //输入电压与否
} aw_battery_status_t;

s8                         aw_battery_init(AwCallBack_void_t timeout_cb);
void                       aw_battery_set_current(u16 cur);
s8                         aw_battery_set_voltage(u16 volt);
s8                         aw_batttey_update();
s8                         aw_batttey_enable_set(u8 enable);
s8                         aw_batttey_enable_get(u8* disable);
const aw_battery_status_t* aw_battery_get_status();
#endif /* MODULE_AW_BATTERY_H_ */
