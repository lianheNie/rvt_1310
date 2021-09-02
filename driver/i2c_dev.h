/*
 * i2c_dev.h
 *
 *  Created on: 2020Äê8ÔÂ4ÈÕ
 *      Author: admin
 */

#ifndef I2C_DEV_H_
#define I2C_DEV_H_
#include "i2c_driver.h"
int8_t i2c_dev_check_null(const aw_i2c_dev_t *i2c_dev);
int8_t i2c_dev_set_routine(aw_i2c_dev_t *i2c_dev, const aw_i2c_dev_t *i2c_routine);

#endif /* I2C_DEV_H_ */
