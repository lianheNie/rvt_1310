/*
 * i2c_dev.c
 *
 *  Created on: 2020Äê8ÔÂ4ÈÕ
 *      Author: admin
 */
#include "i2c_dev.h"
#include "util_delay.h"
int8_t i2c_dev_set_routine(aw_i2c_dev_t *i2c_dev,const aw_i2c_dev_t* i2c_routine) {
    s8 rslt = 0;
    if (i2c_dev != NULL&&(i2c_dev_check_null(i2c_routine)==0)) {
        i2c_dev->read     = i2c_routine->read;
        i2c_dev->write    = i2c_routine->write;
        i2c_dev->delay_ms = i2c_routine->delay_ms;
    } else {
        rslt = -2;
    }
    return rslt;
}

int8_t i2c_dev_check_null(const aw_i2c_dev_t *i2c_dev) {
    s8 rslt = 0;
    if ((i2c_dev != NULL) && (i2c_dev->read != NULL) && (i2c_dev->write != NULL) &&
        (i2c_dev->delay_ms != NULL)) {
        rslt = 0;
    } else {
        rslt = -2;
    }
    return rslt;
}
