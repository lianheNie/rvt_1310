/*
 * nvs_driver.c
 *
 *  Created on: 2020Äê10ÔÂ8ÈÕ
 *      Author: admin
 */
#include <stdlib.h>
#include <ti/drivers/NVS.h>
#include "nvs_driver.h"
#include "board.h"
static NVS_Handle _nvsHandle = NULL;
static NVS_Attrs  _regionAttrs;
static NVS_Params _nvsParams;
s8                nvs_init() {
    NVS_init();
    NVS_Params_init(&_nvsParams);
    _nvsHandle = NVS_open(Board_NVSINTERNAL, &_nvsParams);
    if (_nvsHandle == NULL) {
        return (NULL);
    }
    NVS_getAttrs(_nvsHandle, &_regionAttrs);
    return 0;
}
s8 nvs_write(u8 *buffer, u8 size) {
    return NVS_write(_nvsHandle, 0, (void *)buffer, size, NVS_WRITE_ERASE | NVS_WRITE_POST_VERIFY);
}

s8 nvs_write_u16(u16 val) {
    u8 buf[2] = {0x00};
    buf[0]    = (val >> 8) & 0xFF;
    buf[1]    = val & 0xFF;
    return nvs_write(buf, 2);
}

s8 nvs_write_u32(u32 val) {
    u8 buf[4] = {0x00};
    buf[0]    = (val >> 24) & 0xFF;
    buf[1]    = (val >> 16) & 0xFF;
    buf[2]    = (val >> 8) & 0xFF;
    buf[3]    = val & 0xFF;
    return nvs_write(buf, 4);
}

s8 nvs_read(u8 *buffer, u8 size) { return NVS_read(_nvsHandle, 0, (void *)buffer, size); }

s8 nvs_read_u16(u16 *pVal) {
    s8 res    = 0;
    u8 buf[2] = {0x00};
    res       = nvs_read(buf, 2);
    *pVal     = buf[0] << 8 | buf[1];
    return res;
}
s8 nvs_read_u32(u32 *pVal) {
    s8 res    = 0;
    u8 buf[4] = {0x00};
    res       = nvs_read(buf, 4);
    *pVal     = buf[0] << 24 | buf[1] << 16 | buf[2] << 8 | buf[3];
    return res;
}
s8 nvs_erase() { return NVS_erase(_nvsHandle, 0, _regionAttrs.sectorSize); }

s8 nvs_test() {
    u8  bufW[2] = {0xAA, 0xBB};
    u8  bufR[2] = {0x00};
    u16 val16   = 0;
    u32 val32   = 0;
    s8  res     = nvs_init();
    res         = nvs_read_u32(&val32);
    res         = nvs_write(bufW, 2);
    res         = nvs_read(bufR, 2);
    res         = nvs_write_u16(1234);
    res         = nvs_read_u16(&val16);
    res         = nvs_write_u32(1234567);
    res         = nvs_erase();
    res         = nvs_read_u32(&val32);
    return res;
}
