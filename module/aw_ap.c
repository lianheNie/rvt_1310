/*
 * aw_ap.c
 *
 *  Created on: 2020年11月1日
 *      Author: admin
 */

#include "aw_ap.h"

#include "Board.h"
#include "io_driver.h"
#include "util_delay.h"

#define IO_ON      1
#define IO_OFF     0
#define PA_PIN_ID  IOID_29//IOID_6（开门报警）  //  IOID_29（老网关）
#define LNA_PIN_ID IOID_30//IOID_1（开门报警） //  IOID_30（老网关）

static PIN_Handle _ioPinHandle;
static PIN_State  _ioPinState;
static PIN_Config _ioPinTable[] = {
    PA_PIN_ID | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    LNA_PIN_ID | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, PIN_TERMINATE};

void aw_pa_enable(u8 is_enable) {
    PIN_setOutputValue(_ioPinHandle, PA_PIN_ID, is_enable);
    PIN_setOutputValue(_ioPinHandle, LNA_PIN_ID, is_enable);
}

void aw_lna_enable(u8 is_enable) { PIN_setOutputValue(_ioPinHandle, LNA_PIN_ID, is_enable); }
s8   aw_ap_init() {
    s8 res       = 0;
    _ioPinHandle = PIN_open(&_ioPinState, _ioPinTable);
    if (_ioPinHandle == NULL) {
        return -1;
    }
    return res;
}
