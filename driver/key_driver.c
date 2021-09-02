/*
 * key_driver.c
 *
 *  Created on: 2020Äê8ÔÂ1ÈÕ
 *      Author: admin
 */
#include <ti/drivers/PIN.h>
#include "key_driver.h"

static PIN_Handle _keyPinHandle = NULL;
static PIN_State  _keyPinState;
#define _BTN1_ID IOID_13
#define _BTN2_ID IOID_13
static PIN_Config      _keyPinTable[] = {_BTN1_ID | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
                                    _BTN2_ID | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
                                    PIN_TERMINATE};
static void            keyCallbackFxn(PIN_Handle handle, PIN_Id pinId);
static AwCallBack_u8_t _keyCB = NULL;
int8_t                 key_init(AwCallBack_u8_t keyCb) {
    _keyCB = keyCb;
    return io_input_init(&_keyPinHandle, &_keyPinState, _keyPinTable, &keyCallbackFxn);
}
uint32_t    key_read(uint8_t pinId) { return PIN_getInputValue(pinId); }
static void keyCallbackFxn(PIN_Handle handle, PIN_Id pinId) {
    CPUdelay(8000 * 50);
    if (!PIN_getInputValue(pinId)) {
        if (_keyCB != NULL) {
            _keyCB(pinId);
        }
    }
}
