/*
 * aw_watchdog.c
 *
 *  Created on: 2020年9月23日
 *      Author: admin
 */

#include "aw_watchdog.h"
#define IS_CLEAR_WATCHDOG
#include "util_timer.h"
#include <ti/drivers/Watchdog.h>
#include "Board.h"
#define WATCHDOG_TIMER_EVENT    0x0100
#define WATCHDOG_TIMEOUT_MS     60000  // 1分钟
#define WATCHDOG_TIMEOUT_ERR_MS 1000
static Clock_Struct    _watchdog_clockS;
static Watchdog_Handle _watchdogHandle;
static Watchdog_Params _watchdog_params;
static uint32_t        _watchdog_reloadValue = 0;

static void _watchdog_timeOutCb(u32 arg) {
    //_events |= WATCHDOG_TIMER_EVENT;
#ifdef IS_CLEAR_WATCHDOG
    Watchdog_clear(_watchdogHandle);
#endif
    //    if (NULL != _appSem) {
    //        Semaphore_post(_appSem);
    //    }
}

static void _watchdog_timer_init(u32 timeout) {
    Timer_construct(&_watchdog_clockS, _watchdog_timeOutCb, timeout, timeout, false, 0);
    Timer_start(&_watchdog_clockS);  //开定时器
}

#include "sys_ctrl.h"
static void _watchdogCallback(uintptr_t watchdogHandle) {
    SysCtrlSystemReset();
    while (1) {
    }
}

static void _watchdog_init(u32 timeout) {
    Watchdog_init();
    Watchdog_Params_init(&_watchdog_params);
    _watchdog_params.callbackFxn    = (Watchdog_Callback)_watchdogCallback;
    _watchdog_params.debugStallMode = Watchdog_DEBUG_STALL_ON;
    _watchdog_params.resetMode      = Watchdog_RESET_ON;
    _watchdogHandle                 = Watchdog_open(Board_WATCHDOG0, &_watchdog_params);
    if (_watchdogHandle == NULL) {
        /* Error opening Watchdog */
        while (1) {
        }
    }
    _watchdog_reloadValue = Watchdog_convertMsToTicks(_watchdogHandle, WATCHDOG_TIMEOUT_MS);

    if (_watchdog_reloadValue != 0) {
        Watchdog_setReload(_watchdogHandle, _watchdog_reloadValue);
    }
    _watchdog_timer_init(timeout);  // WATCHDOG_TIMEOUT_MS - WATCHDOG_TIMEOUT_ERR_MS);  // 10s
}

void aw_watchdog_init() { _watchdog_init(WATCHDOG_TIMEOUT_MS - WATCHDOG_TIMEOUT_ERR_MS); }
void aw_watchdog_clear() {
#ifdef IS_CLEAR_WATCHDOG
    Watchdog_clear(_watchdogHandle);
#endif
}
