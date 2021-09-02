/*
 * aw_ap.c
 *
 *  Created on: 2020��11��1��
 *      Author: admin
 */

#include "aw_ap.h"

#include "Board.h"
#include "io_driver.h"
#include "util_delay.h"

#define IO_ON      1
#define IO_OFF     0
#define PA_PIN_ID  IOID_29//IOID_6�����ű�����  //  IOID_29�������أ�
#define LNA_PIN_ID IOID_30//IOID_1�����ű����� //  IOID_30�������أ�

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
