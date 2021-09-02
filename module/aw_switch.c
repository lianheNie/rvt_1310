/*
 * aw_remoteSwitch.c
 *
 *  Created on: 2020Äê9ÔÂ23ÈÕ
 *      Author: admin
 */

#include <aw_switch.h>
#include "Board.h"
#include "util_delay.h"
#include "io_driver.h"
static const PIN_Id _Sw_Pins[SW_CH_CNT] = {SW_CH1_PIN_ID
#if SW_CH_CNT > 1
                                           ,
                                           SW_CH2_PIN_ID
#endif
#if SW_CH_CNT > 2
                                           ,
                                           SW_CH3_PIN_ID
#endif
};

static PIN_Handle _ioSwPinHandle = NULL;
static PIN_State  _ioSwPinState;
static PIN_Config _ioSwPinTable[] = {
    SW_CH1_PIN_ID | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#if SW_CH_CNT > 1
    SW_CH2_PIN_ID | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#endif
#if SW_CH_CNT > 2
    SW_CH3_PIN_ID | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#endif
    PIN_TERMINATE};
static E_sw_sts_t _sw_sts[AW_SW_CNT] = {AW_SW_OFF};
bool              sw_set_ch(E_sw_chs_t ch, E_sw_sts_t sts) {
    if (NULL == _ioSwPinHandle) {
        return false;
    }
#ifdef IS_USE_SERVO_SWITCH
    u8 ch_id = ch * 2;
    if (ch_id >= SW_CH_CNT) {
        return false;
    }
    if (AW_SW_NEG == sts) {
        PIN_setOutputValue(_ioSwPinHandle, _Sw_Pins[ch_id], 1);
        PIN_setOutputValue(_ioSwPinHandle, _Sw_Pins[ch_id + 1], 0);
    } else if (AW_SW_ON == sts) {
        PIN_setOutputValue(_ioSwPinHandle, _Sw_Pins[ch_id], 0);
        PIN_setOutputValue(_ioSwPinHandle, _Sw_Pins[ch_id + 1], 1);
    } else {
        PIN_setOutputValue(_ioSwPinHandle, _Sw_Pins[ch_id], 0);
        PIN_setOutputValue(_ioSwPinHandle, _Sw_Pins[ch_id + 1], 0);
    }
#else
    if (sts == AW_SW_NEG) {
    } else {
        if (NULL != _ioSwPinHandle) {
            PIN_setOutputValue(_ioSwPinHandle, _Sw_Pins[ch], sts);
        } else {
            return false;
        }
    }
#endif
    _sw_sts[ch] = sts;
    return true;
}
bool sw_set_chs(E_sw_chs_t *chs, u8 cnt, E_sw_sts_t sts) {
    if (cnt > SW_CH_CNT) {
        return false;
    }
    u8 i = 0;
    for (i = 0; i < cnt; i++) {
        if (!sw_set_ch(chs[i], sts)) {
            return false;
        }
    }
    return true;
}
#define IS_USE_SW_DELAY
#ifdef IS_USE_SW_DELAY

#define IS_USE_SW_TIMER
#ifdef IS_USE_SW_TIMER
#include "util_timer.h"
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
static Clock_Struct _sw_clockS;
static Clock_Handle _sw_clock = NULL;
// static Semaphore_Struct _sw_timeoutSemS;
static Semaphore_Handle _sw_timeoutSem;

#include <xdc/runtime/Error.h>
static int8_t _sw_sem_init() {
    Semaphore_Params params;
    Error_Block      eb;
    Semaphore_Params_init(&params);
    Error_init(&eb);
    _sw_timeoutSem = Semaphore_create(0, &params, &eb);
    if (_sw_timeoutSem == NULL) {
        // System_abort("Semaphore creation failed");
    }

    return AW_OK;
}

static void _sw_timeout_cb(u32 arg) {
    if (NULL != _sw_timeoutSem) {
        Semaphore_post(_sw_timeoutSem);
    }
}

static void _sw_timer_init(u32 timeout) {
    _sw_clock = Timer_construct(&_sw_clockS, _sw_timeout_cb, timeout, 0, false, 0);
}
#endif

bool sw_set_chs_delay(E_sw_chs_t *chs, u8 cnt, E_sw_sts_t sts, u16 ms) {
    bool isOK = sw_set_chs(chs, cnt, sts);
    if (!isOK) {
        return false;
    }
    if (ms > 0) {
#ifdef IS_USE_SW_TIMER
        Timer_setTimeout(_sw_clock, ms);
        Timer_start(&_sw_clockS);
        Semaphore_pend(_sw_timeoutSem, ((ms + 10) * 1000) / Clock_tickPeriod);
#else
        aw_delay_ms(ms);
#endif
        isOK = sw_set_chs(chs, cnt, AW_SW_OFF);
    }
    return isOK;
}

const E_sw_sts_t *sw_get_chs() { return _sw_sts; }
bool              sw_set_chs_sts(E_sw_sts_t *sts, u16 ms) {
    E_sw_chs_t _sw_chs[SW_CH_CNT] = {
        AW_SW_CH1
#if SW_CH_CNT > 1
        ,
        AW_SW_CH2
#endif
#if SW_CH_CNT > 2
        ,
        AW_SW_CH3
#endif
    };
    E_sw_sts_t chSts = AW_SW_NULL;
    u8         i     = 0;
    u8         j     = 0;
    for (i = 0; i < SW_CH_CNT; i++) {
        if (sts[i] == AW_SW_NEG || sts[i] == AW_SW_ON || sts[i] == AW_SW_OFF) {
            chSts      = sts[i];
            _sw_chs[j] = (E_sw_chs_t)i;
            j++;
        }
    }
    if (chSts != AW_SW_NULL && j <= SW_CH_CNT && j > 0) {
        sw_set_chs_delay(_sw_chs, j, chSts, ms);
    }
    return true;
}

#endif

static s8 _sw_init(E_sw_chs_t *chs, u8 cnt, E_sw_sts_t sts) {
    _ioSwPinHandle = PIN_open(&_ioSwPinState, _ioSwPinTable);
    if (_ioSwPinHandle == NULL) {
        return -1;
    }
#ifdef IS_USE_SW_DELAY
#ifdef IS_USE_SW_TIMER
    _sw_sem_init();
    _sw_timer_init(30000);
#endif
#endif
    sw_set_chs(chs, cnt, sts);
    return 0;
}
s8 aw_sw_init(E_sw_sts_t sts) {
    E_sw_chs_t _sw_chs[SW_CH_CNT] = {
        AW_SW_CH1
#if SW_CH_CNT > 1
        ,
        AW_SW_CH2
#endif
#if SW_CH_CNT > 2
        ,
        AW_SW_CH3
#endif
    };
    return _sw_init(_sw_chs, SW_CH_CNT, sts);
}
