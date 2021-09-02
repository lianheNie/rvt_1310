/*
 * task_easylink.c
 *
 *  Created on: 2020年9月15日
 *      Author: admin
 */

/* EasyLink API Header files */
#include <iot/easylink/aw_easylink.h>
#include <iot/easylink/aw_easylink_def.h>
#include <iot/easylink/EasyLink.h>
#include <iot/easylink/EasyLink.h>
#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
/* Board Header files */
#include "Board.h"
#include "util_delay.h"
#include "util_def.h"
#include "uart_driver.h"
#include "util_other.h"

#ifdef IS_USE_PA_LNA
#include "aw_ap.h"
#endif
#if defined(IS_USE_NVS)
#include "nvs_driver.h"
#endif
#define _EASYLINK_RX_EVENT 0x0001
#define _EASYLINK_TX_EVENT 0x0002

static u16              _events  = 0;
static Semaphore_Handle _appSem  = NULL;
static Semaphore_Handle _txOkSem = NULL;

#if defined(IS_USE_RX) | defined(IS_USE_PAIR)
static EasyLink_RxPacket _easyLink_rxPacket;
static Easylink_ACK_t    _easylink_ack;
static void              _easylink_rxCB(EasyLink_RxPacket* rxPacket, EasyLink_Status status) {
    _events |= _EASYLINK_RX_EVENT;
    if (EasyLink_Status_Success == status) {
        memcpy(&_easyLink_rxPacket, rxPacket, sizeof(EasyLink_RxPacket));
    }
    if (NULL != _appSem) {
        Semaphore_post(_appSem);
    }
}
//extern void easylink_rx_cb(EasyLink_RxPacket* rxPacket);
static void _proc_easylink_rx_event() {
    if (_easyLink_rxPacket.payload[0]) {
        //easylink_rx_cb(&_easyLink_rxPacket);
        Easylink_cmdIds_t cmdId = (Easylink_cmdIds_t)_easyLink_rxPacket.payload[0];
        u16 srcId = _easyLink_rxPacket.payload[1] << 8 | _easyLink_rxPacket.payload[2];
        if (Easylink_cmdIds_configReq == cmdId) {
            _easylink_ack.ackCmdId = Easylink_cmdIds_configRsp;
        } else if (Easylink_cmdIds_config == cmdId) {
            _easylink_ack.ackCmdId = Easylink_cmdIds_configRsp;
        } else if (Easylink_cmdIds_notify == cmdId) {
            _easylink_ack.ackCmdId = Easylink_cmdIds_notifyRsp;
        } else if (Easylink_cmdIds_sensor_data == cmdId) {
            _easylink_ack.ackCmdId = Easylink_cmdIds_sensor_dataRsp;
        } else if (Easylink_cmdIds_sensor_dataReq == cmdId) {
            _easylink_ack.ackCmdId = Easylink_cmdIds_sensor_dataRsp;
        } else if (Easylink_cmdIds_status_data == cmdId) {
            _easylink_ack.ackCmdId = Easylink_cmdIds_status_dataRsp;
        } else if (Easylink_cmdIds_status_dataReq == cmdId) {
            _easylink_ack.ackCmdId = Easylink_cmdIds_status_dataRsp;
        } else if (Easylink_cmdIds_IOReq == cmdId) {
            _easylink_ack.ackCmdId = Easylink_cmdIds_IORsp;
        } else if (Easylink_cmdIds_pairReq == cmdId) {
            _easylink_ack.ackCmdId = Easylink_cmdIds_pairRsp;
        } else {
        }
        _easylink_ack.id = AW_SELF_ADDRESS;

#ifdef IS_USE_EASYLINK_ACK
        aw_easylink_send_ack(&_easylink_ack, srcId);
#endif

        _easyLink_rxPacket.payload[0] = 0;

    } else {
        return;
    }
}
#endif

#include <ti/sysbios/knl/Clock.h>
#define _TX_OK_TIMEOUT_MS (AW_RX_TIMEOUT_MS * 1000 / Clock_tickPeriod)  // 10s
static Easylink_TX_t _easylink_tx_packet;
bool                 easylink_txCB(Easylink_TX_t* txPacket) {
    _events |= _EASYLINK_TX_EVENT;
    memcpy(&_easylink_tx_packet, txPacket, sizeof(Easylink_TX_t));
    if (NULL != _appSem) {
        Semaphore_post(_appSem);
        if (NULL != _txOkSem) {
            if (Semaphore_pend(_txOkSem, _TX_OK_TIMEOUT_MS)) {
                return true;
            }
        }
    }
    return false;
}

static void _proc_easylink_tx_event() {
    bool isOK = aw_easylink_send_with_ack(_easylink_tx_packet.payload, _easylink_tx_packet.len,
                                          _easylink_tx_packet.dstAdd);
    if (isOK && NULL != _txOkSem) {
        Semaphore_post(_txOkSem);
    }
}
static void _proc_events() {
#if defined(IS_USE_RX) | defined(IS_USE_PAIR)
    if (_events & _EASYLINK_RX_EVENT) {
        _proc_easylink_rx_event();
        Util_clearEvent(&_events, _EASYLINK_RX_EVENT);
    }
#endif
    if (_events & _EASYLINK_TX_EVENT) {
        _proc_easylink_tx_event();
        Util_clearEvent(&_events, _EASYLINK_TX_EVENT);
    }
}

static void _app_process() {
    while (1) {
        Semaphore_pend(_appSem, BIOS_WAIT_FOREVER);
        _proc_events();
#ifdef IS_USE_RX
        aw_easylink_rx();
#endif
    }
}

static void _app_sem_init() {
    Semaphore_Params params;
    Error_Block      eb;
    Semaphore_Params_init(&params);
    Error_init(&eb);
    _appSem = Semaphore_create(0, &params, &eb);
    if (_appSem == NULL) {
        // System_abort("Semaphore creation failed");
    }
    _txOkSem = Semaphore_create(0, &params, &eb);
    if (_txOkSem == NULL) {
        // System_abort("Semaphore creation failed");
    }
}

static void _app_init() {
    _app_sem_init();

#if defined(IS_USE_NVS)
    nvs_init();
#ifdef IS_USE_PAIR  //启用配对
    u16 pair = 0;
    //    nvs_erase();
    //    nvs_write_u16(123);
    nvs_read_u16(&pair);
    if (NVS_NULL == pair) {
        pair = aw_easylink_pair_get();
        nvs_write_u16(pair);
    }
    aw_easylink_pair_set(pair);
#endif
#endif

#ifdef IS_USE_PA_LNA
    aw_ap_init();
#endif
#if defined(IS_USE_RX) | defined(IS_USE_PAIR)
    aw_easylink_init(_easylink_rxCB);
#else
    aw_easylink_init(NULL);
#endif

    if (aw_easylink_pair_get() > 0 && aw_easylink_pair_get() != AW_SELF_ADDRESS) {
        Easylink_Notify_t notify;
        notify.id = AW_SELF_ADDRESS;
        aw_easylink_send_notify(&notify, aw_easylink_pair_get());
    }

#ifdef IS_USE_RX
    aw_easylink_rx();
#endif
}

static void _rfEasyLinkRxThread(UArg arg0, UArg arg1) {
    _app_init();
    _app_process();
}

#include <ti/sysbios/knl/Task.h>
#define _TASK_STACK_SIZE 600
static Task_Struct _task_struct;
static Char        _task_stack[_TASK_STACK_SIZE];
void               aw_easylink_task_init() {
    Task_Params taskParams;
    Task_Params_init(&taskParams);
    taskParams.stackSize = _TASK_STACK_SIZE;
    taskParams.stack     = &_task_stack;
    taskParams.priority  = 2;
    Task_construct(&_task_struct, (Task_FuncPtr)_rfEasyLinkRxThread, &taskParams, NULL);
}
