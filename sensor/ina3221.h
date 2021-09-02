/*
 * ina3221.h
 *
 *  Created on: 2020Äê7ÔÂ31ÈÕ
 *      Author: admin
 */

#ifndef INA3221_H_
#define INA3221_H_
#include "driver_comm.h"
#include "ina3221_def.h"
#include "aw_config.h"
#ifdef IS_USE_DEV
s8 ina3221_init(aw_i2c_dev_t *ina3221_dev, float shuntresistor);
#else
s8 ina3221_init(float shuntresistor);
#endif
s8 ina3221_getBusVoltage_V(int channel, float *busV);
s8 ina3221_getShuntVoltage_mV(int channel, float *shuntV);
s8 ina3221_getCurrent_mA(int channel, float *cur);
s8 ina3221_get_current_mA(int channel, u16 *cur);
s8 ina3221_getManufID(u16 *mID);
s8 ina3221_config(void);
s8 ina3221_getBusVoltage_raw(int channel, s16 *busV);
s8 ina3221_getShuntVoltage_raw(int channel, s16 *shutV);

#endif /* INA3221_H_ */
