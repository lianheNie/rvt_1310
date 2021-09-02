/*
 * spi_driver.h
 *
 *  Created on: 2020Äê10ÔÂ3ÈÕ
 *      Author: admin
 */

#ifndef DRIVER_SPI_DRIVER_H_
#define DRIVER_SPI_DRIVER_H_
#include "aw_config.h"
#include "util_def.h"
#include <ti/drivers/SPI.h>
s8 spi_init();
s8 spi_open(SPI_FrameFormat frameFormat);
s8 spi_close();
s8 spi_write(u8 reg, u8 *buf, u8 cnt);
s8 spi_read(u8 reg, u8 *buf, u8 cnt);
s8 spi_write_cmd(u8 cmd);
#endif /* DRIVER_SPI_DRIVER_H_ */
