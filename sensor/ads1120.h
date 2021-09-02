/*
 * ads1120.h
 *
 *  Created on: 2020Äê11ÔÂ6ÈÕ
 *      Author: admin
 */

#ifndef SENSOR_ADS1120_H_
#define SENSOR_ADS1120_H_

#include "spi_driver.h"
s8 ads1120_test();
s8 ads1120_init();
s8 ads1120_close();
s8 ads1120_read_raw(s16* val);
s8 ads1120_read_temp(s16* val);
s8 ads1120_spi_open();
s8 ads1120_spi_close();
s8 ads1120_set_internal_source(s16* temp_val);
s8 ads1120_set_external_source();
#endif /* SENSOR_ADS1120_H_ */
