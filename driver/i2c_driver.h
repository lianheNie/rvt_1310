/*
 * i2c_driver.h
 *
 *  Created on: 2020Äê7ÔÂ31ÈÕ
 *      Author: admin
 */

#ifndef I2C_DRIVER_H_
#define I2C_DRIVER_H_
#include "driver_comm.h"
int8_t i2c_init();
int8_t i2c_close();
s8     i2c_open();
int8_t i2c_readByCmd(uint8_t addr, uint8_t *cmd, uint8_t cmdCnt, uint8_t *dataBuf,
                     uint16_t dataCnt);
int8_t i2c_writeByCmd(uint8_t addr, uint8_t *cmd, uint8_t cmdCnt, uint8_t *dataBuf,
                      uint16_t dataCnt);
int8_t i2c_read(uint8_t addr, uint8_t reg, uint8_t *dataBuf, uint16_t dataCnt);
int8_t i2c_write(uint8_t addr, uint8_t reg, uint8_t *dataBuf, uint16_t dataCnt);
int8_t i2c_burst_read(uint8_t addr, uint8_t reg, uint8_t *dataBuf, uint32_t dataCnt);

#endif /* I2C_DRIVER_H_ */
