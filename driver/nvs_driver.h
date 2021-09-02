/*
 * nvs_driver.h
 *
 *  Created on: 2020Äê10ÔÂ8ÈÕ
 *      Author: admin
 */

#ifndef DRIVER_NVS_DRIVER_H_
#define DRIVER_NVS_DRIVER_H_

#include "util_def.h"
#define NVS_NULL 0xFFFF
s8 nvs_init();
s8 nvs_write(u8 *buffer, u8 size);
s8 nvs_write_u16(u16 val);
s8 nvs_write_u32(u32 val);
s8 nvs_read(u8 *buffer, u8 size);
s8 nvs_read_u16(u16 *pVal);
s8 nvs_read_u32(u32 *pVal);
s8 nvs_erase();
s8 nvs_test();
#endif /* DRIVER_NVS_DRIVER_H_ */
