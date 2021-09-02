/*
 * ina219.h
 *
 *  Created on: 2020Äê9ÔÂ21ÈÕ
 *      Author: admin
 */

#ifndef SENSOR_INA219_H_
#define SENSOR_INA219_H_

#include "driver_comm.h"
#include "ina219_def.h"
#include "aw_config.h"
#ifdef IS_USE_DEV
s8 ina219_init(aw_i2c_dev_t *ina219_dev, float shuntresistor);
#else
s8 ina219_init(float shuntresistor);
#endif
s8 ina219_get_current_mA(u16 *cur);
s8 ina219_getCurrent_raw(u16 *cur);
s8 ina219_getCurrent_mA_f(float *cur);
s8 ina219_getManufID(u16 *mID);
s8 ina219_config(void);
s8 ina219_getBusVoltage_raw(s16 *busV);
s8 ina219_getShuntVoltage_raw(float *shutV);
s8 ina219_getPower_raw(float *power);

#endif /* SENSOR_INA219_H_ */
