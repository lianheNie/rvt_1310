/*
 * bq25703a.h
 *
 *  Created on: 2020Äê7ÔÂ31ÈÕ
 *      Author: admin
 */

#ifndef BQ25703A_H_
#define BQ25703A_H_

#include "driver_comm.h"
#include "bq25703a_def.h"
#include "aw_config.h"
#ifdef IS_USE_DEV
int8_t bq25703a_init(aw_i2c_dev_t *i2c_dev, u16 current);
#else
int8_t bq25703a_init(u16 current);
#endif
int8_t                bq25703a_get_id(u8 *chipId);
int8_t                bq25703a_set_current(u16 current);
int8_t                bq25703a_enable_adc();
int8_t                bq25703a_enable_adcInBatteryMode();
s8                    bq25703a_set_voltage(u16 volt);
s8                    bq25703a_get_voltage(u16 *volt);
s8                    bq25703a_enable_set(u8 enable);
s8                    bq25703a_enable_get(u8 *disable);
int8_t                bq25703a_set_IINHost(u16 current);
int8_t                bq25703a_refresh_adcData(u16 *vBus, u16 *vBat, u16 *vSys);
int8_t                bq25703a_refresh_settingData(u16 *chargeCurrent);
int8_t                bq25703a_refresh_chargeStatus(ChargeStatus_t *_status);
const SettingData_t * bq25703a_get_setting_data();
const ADCData_t *     bq25703a_get_adc_data();
const ChargeStatus_t *bq25703a_get_status_data();

#endif /* BQ25703A_H_ */
