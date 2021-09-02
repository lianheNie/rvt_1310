/*
 * hc-sr04.c
 *
 *  Created on: 2020年10月1日
 *      Author: admin
 */

#include "hc-sr04.h"
#include <ti/drivers/PIN.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include "util_delay.h"
#include "uart_driver.h"
#include <ti/devices/cc13x0/driverlib/ioc.h>
#define TRIG_PIN_ID IOID_6  // IOID_8
#define ECHO_PIN_ID IOID_8

#define _ECHO_ACK_TIMEOUT_MS (100 * 1000) / Clock_tickPeriod  // 100ms

static Semaphore_Handle _echoSem = NULL;
static bool             _echo_sem_init() {
    Semaphore_Params params;
    Error_Block      eb;
    Semaphore_Params_init(&params);
    Error_init(&eb);
    _echoSem = Semaphore_create(0, &params, &eb);
    if (_echoSem == NULL) {
        return false;
    }
    return true;
}
static void _echo_int_cb(PIN_Handle handle, PIN_Id pinId) {
    if (_echoSem != NULL) {
        Semaphore_post(_echoSem);
    }
}
static PIN_Handle _ioPinHandle = NULL;
static PIN_State  _ioPinState;
static PIN_Config _ioPinTable[] = {
    TRIG_PIN_ID | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PULLUP | PIN_DRVSTR_MAX,
    ECHO_PIN_ID | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES, PIN_TERMINATE};

#define _TRIG_ON      1
#define _TRIG_OFF     0
#define _TRIG_TIME_US 20
void _hc_sr04_trig_set(u32 val) {
    if (_ioPinHandle != NULL) {
        PIN_setOutputValue(_ioPinHandle, TRIG_PIN_ID, val);
    }
}

s8 hc_sr04_init() {
    _echo_sem_init();
    _ioPinHandle = PIN_open(&_ioPinState, _ioPinTable);
    if (_ioPinHandle == NULL) {
        return AW_ERR;
    }
    if (PIN_registerIntCb(_ioPinHandle, _echo_int_cb) != 0) {
        return AW_ERR;
    }
    return AW_OK;
}
#define _DISTANCE_RATIO 58
int _hc_sr04_get() {
    _hc_sr04_trig_set(_TRIG_ON);
    aw_delay_us(_TRIG_TIME_US);
    _hc_sr04_trig_set(_TRIG_OFF);
    if (!Semaphore_pend(_echoSem, _ECHO_ACK_TIMEOUT_MS)) return -1;
    int ticks_last = Clock_getTicks();
    if (!Semaphore_pend(_echoSem, _ECHO_ACK_TIMEOUT_MS)) return -1;
    int ticks_now = Clock_getTicks();
    int time_us   = (ticks_now - ticks_last) * Clock_tickPeriod;
    if (time_us < 0) time_us = 0;
    return (time_us / _DISTANCE_RATIO);
}
#define _TRIG_TRY_CNT 10
int hc_sr04_get() {
    u8  cnt = 0;
    int dis = -1;
    dis     = _hc_sr04_get();
    while (dis < 0 && cnt < _TRIG_TRY_CNT) {
        aw_delay_us(10);
        dis = _hc_sr04_get();
        cnt++;
    }
    return dis;
}
static void _app_thread(UArg arg0, UArg arg1) {
    hc_sr04_init();
    uart_init(9600, NULL);               //串口初始化
    uart_printf("hc_sr04 test...\r\n");  //串口初始化
    while (1) {
        uart_printf("dis=%d\r\n", _hc_sr04_get());
    }
}
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#define _TASK_STACK_SIZE 512
static Task_Struct _task_struct;
static Char        _task_stack[_TASK_STACK_SIZE];
void               aw_hc_sr04_task_test_init() {
    Task_Params taskParams;
    Task_Params_init(&taskParams);
    taskParams.stackSize = _TASK_STACK_SIZE;
    taskParams.stack     = &_task_stack;
    taskParams.priority  = 2;
    Task_construct(&_task_struct, (Task_FuncPtr)_app_thread, &taskParams, NULL);
}
