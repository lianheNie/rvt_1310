/*
 * aw_pir.c
 *
 *  Created on: 2020年10月5日
 *      Author: admin
 */
#include "aw_pir.h"
#include <ti/drivers/PIN.h>
#include <ti/devices/cc13x0/driverlib/ioc.h>
#include <stddef.h>
#define _AW_PIR_PIN_ID IOID_2  // IOID_1 IOID_2（红外报警）
static AwCallBack_s8_t _pirCb = NULL;
static PIN_Handle _ioPinHandle = NULL;
static PIN_State _ioPinState;
#define AW_PIR_VAL 1
static PIN_Config _ioPinTable[] = { _AW_PIR_PIN_ID | PIN_INPUT_EN | PIN_PULLDOWN
                                            | PIN_IRQ_POSEDGE,
                                    PIN_TERMINATE };
static void _ioCallbackFxn(PIN_Handle handle, PIN_Id pinId)
{
    if (PIN_getInputValue(pinId))
    {
        if (_pirCb != NULL)
        {
            _pirCb(AW_PIR_VAL);
        }
    }
}
s8 aw_pir_init(AwCallBack_s8_t pirCb)
{
    _pirCb = pirCb;
    _ioPinHandle = PIN_open(&_ioPinState, _ioPinTable);
    if (_ioPinHandle == NULL)
    {
        /* Error initializing button pins */
        return AW_ERR;
    }
    /* Setup callback for button pins */
    if (PIN_registerIntCb(_ioPinHandle, _ioCallbackFxn) != 0)
    {
        return AW_ERR;
    }
    return AW_OK;
}

void aw_pir_close()
{
    PIN_setConfig(_ioPinHandle, PIN_BM_IRQ, _AW_PIR_PIN_ID | PIN_IRQ_DIS);
    PIN_close(_ioPinHandle);
}
