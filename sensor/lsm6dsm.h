/*
 * lsm6dsm.h
 *
 *  Created on: 2020Äê10ÔÂ21ÈÕ
 *      Author: admin
 */

#ifndef SENSOR_LSM6DSM_H_
#define SENSOR_LSM6DSM_H_
#include "lsm6dsm_def.h"

#include "util_def.h"
s8   lsm6dsm_init();
s8   lsm6dsm_test();
s16  lsm6dsm_read_accel(E_xyz_axis_t axis);
s16* lsm6dsm_calibration();
u16  lsm6dsm_fifo_full();
s16  lsm6dsm_fifo_read();
s16  lsm6dsm_fifo_reads(E_xyz_axis_t axis);
s8   lsm6dsm_accel_enable(uint8_t enable);
s8   lsm6dsm_spi_open();
s8   lsm6dsm_spi_close();
#endif /* SENSOR_LSM6DSM_H_ */
