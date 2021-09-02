/*
 * aw_powerctl.h
 *
 *  Created on: 2020Äê9ÔÂ23ÈÕ
 *      Author: admin
 */

#ifndef MODULE_AW_POWER_H_
#define MODULE_AW_POWER_H_
#include "util_def.h"
#include "aw_config.h"
typedef enum { AW_POWER_OFF, AW_POWER_ON } AW_power_ctl_t;
bool aw_power_switch_set(AW_power_ctl_t val);
u32  aw_power_switch_get();
bool aw_power_adc_update(u16 *mV);
u16  aw_power_adc_get();
void aw_power_init(AwCallBack_void_t switchCb, AwCallBack_void_t adcCb);
#endif /* MODULE_AW_POWER_H_ */
