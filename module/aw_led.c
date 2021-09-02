/*
 * aw_led.c
 *
 *  Created on: 2020Äê9ÔÂ23ÈÕ
 *      Author: admin
 */

#include "aw_led.h"
#include "Board.h"
#include "io_driver.h"
#include "util_delay.h"
#define IO_ON      1
#define IO_OFF     0
#define LED_PIN_ID IOID_6
static PIN_Handle _ioLedPinHandle;
static PIN_State  _ioLedPinState;
static bool       _is_enable       = false;
static PIN_Config _ioLedPinTable[] = {
    LED_PIN_ID | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, PIN_TERMINATE};


void aw_led_enable(bool is_enable) {
    PIN_setOutputValue(_ioLedPinHandle, LED_PIN_ID, (u8)is_enable);
    _is_enable = is_enable;
}
void aw_led_init() {
    _ioLedPinHandle = PIN_open(&_ioLedPinState, _ioLedPinTable);
    if (_ioLedPinHandle == NULL) {
        return;
    }
    PIN_setOutputValue(_ioLedPinHandle, LED_PIN_ID, 0);
}

void aw_set_led(u8 val) {
    if (!_is_enable) {
        return;
    }
    if (NULL != _ioLedPinHandle) {
        PIN_setOutputValue(_ioLedPinHandle, LED_PIN_ID, val);
    }
}

void aw_toggle_led() { aw_set_led(!PIN_getOutputValue(LED_PIN_ID)); }

void aw_blink_led(u8 cnt, u16 ms) {
    if (!_is_enable) {
        return;
    }
    while (cnt > 0) {
        aw_toggle_led();
        aw_delay_ms(ms);
        aw_toggle_led();
        aw_delay_ms(ms);
        cnt--;
    }
}
