/*
 * tcs34725.c
 *
 *  Created on: 2020Äê7ÔÂ31ÈÕ
 *      Author: admin
 */
#include "tcs34725_def.h"
#include "tcs34725.h"
#include "i2c_driver.h"
#include "util_printf.h"

int8_t tcs34725_init() {
    //    int8_t res = i2c_read(TCS34725_ADDR, TCS34725_ID_REG | TCS34725_CMD_BIT, _rxBuffer, 1);
    //    if (res != 0) {
    //        util_printf("TCS34725 communication err\r\n");
    //    } else {
    //        // util_printf("TCS34725 MID=0x%02x\r\n", _rxBuffer[0]);
    //    }
    //    return res;
    return 0;
}
