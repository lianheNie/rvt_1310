/*
 * io_driver.c
 *
 *  Created on: 2020Äê8ÔÂ1ÈÕ
 *      Author: admin
 */

#include "io_driver.h"
int8_t io_input_init(PIN_Handle *handle, PIN_State *state, const PIN_Config pinList[],
                     PIN_IntCb callbackFxn) {
    PIN_Handle _handle = NULL;
    _handle            = PIN_open(state, pinList);
    if (_handle == NULL) {
        /* Error initializing button pins */
        return AW_ERR;
    }
    /* Setup callback for button pins */
    if (PIN_registerIntCb(_handle, callbackFxn) != 0) {
        return AW_ERR;
    }
    *handle = _handle;
    return AW_OK;
}
int8_t io_output_init(PIN_Handle *handle, PIN_State *state, const PIN_Config pinList[]) {
    PIN_Handle _handle = NULL;
    _handle            = PIN_open(state, pinList);
    if (_handle == NULL) {
        /* Error initializing button pins */
        return AW_ERR;
    }
    *handle = _handle;
    return AW_OK;
}
