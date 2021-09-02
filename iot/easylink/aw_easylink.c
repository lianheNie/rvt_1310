/*
 * aw_easylink.c
 *
 *  Created on: 2020年9月23日
 *      Author: admin
 */

#include <iot/easylink/aw_easylink.h>
#include "aw_config.h"
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/BIOS.h>
#ifdef IS_USE_PA_LNA
#include "aw_ap.h"
#endif
#define _TX_TIMEOUT_MS (500 * 1000 / Clock_tickPeriod)  // 500ms
static aw_easylink_rxCB_t _easylink_rxCb    = NULL;
static Semaphore_Handle   _easylinkSem      = NULL;
static Easylink_Status_t  _easylink_status  = Easylink_status_null;
static u16                _easylink_pair_id = AW_EASYLINK_PAIR_DEFAULT_ADDRESS;
u16                       aw_easylink_pair_get() { return _easylink_pair_id; }
void                      aw_easylink_pair_set(u16 pair) { _easylink_pair_id = pair; }
Easylink_Status_t         aw_easylink_status_get() { return _easylink_status; }
static void               _easylink_sem_init() {
    Semaphore_Params params;
    Semaphore_Params_init(&params);
    _easylinkSem = Semaphore_create(0, &params, NULL);
    if (_easylinkSem == NULL) {
        // System_abort("Semaphore creation failed");
    }
}
#ifdef IS_USE_EASYLINK
#if defined(IS_USE_RX) | defined(IS_USE_PAIR)
#define _EASYLINK_RXPACKET_SIZE 3
typedef EasyLink_RxPacket AW_RxPacket_t;
typedef struct {
    EasyLink_RxPacket buf[_EASYLINK_RXPACKET_SIZE];
    u8                head;
    u8                next;
    u8                tail;
} AW_Easylink_RxBuf_t;

static AW_Easylink_RxBuf_t _rxPacketBuf;

static AW_BOOL _putRxBuf(AW_Easylink_RxBuf_t* buf, EasyLink_RxPacket* data) {
    if (buf->head == buf->next) {
        return AW_FALSE;
    }
    buf->buf[buf->tail] = *data;
    buf->tail           = buf->next;
    buf->next++;
    if (buf->next == _EASYLINK_RXPACKET_SIZE) {
        buf->next = 0;
    }
    return AW_TRUE;
}

static AW_BOOL _readRxBuf(AW_Easylink_RxBuf_t* buf, EasyLink_RxPacket* data) {
    if (buf->head == buf->tail) {
        return AW_FALSE;
    }
    *data = buf->buf[buf->head];
    buf->head++;
    if (buf->head == _EASYLINK_RXPACKET_SIZE) {
        buf->head = 0;
    }
    return AW_TRUE;
}

static AW_BOOL _resetRxBuf(AW_Easylink_RxBuf_t* buf) {
    memset(buf->buf, 0, _EASYLINK_RXPACKET_SIZE);  // Clear planner struct
    buf->head = 0;
    buf->tail = 0;
    buf->next = 1;
    return AW_TRUE;
}

AW_BOOL aw_easylink_put_buf(EasyLink_RxPacket* data) { return _putRxBuf(&_rxPacketBuf, data); }
AW_BOOL aw_easylink_read_buf(EasyLink_RxPacket* data) { return _readRxBuf(&_rxPacketBuf, data); }
AW_BOOL aw_easylink_init_buf() { return _resetRxBuf(&_rxPacketBuf); }
#endif
#endif

static void _easylink_txDoneCb(EasyLink_Status status) {
    if (status == EasyLink_Status_Success) {
    } else if (status == EasyLink_Status_Aborted) {
    } else {
    }
    if (_easylinkSem != NULL) {
        Semaphore_post(_easylinkSem);
    }
}

#define _RX_TIMEOUT_MS (AW_RX_TIMEOUT_MS * 1000 / Clock_tickPeriod)  // 3s
static void _easylink_rxDoneCb(EasyLink_RxPacket* rxPacket, EasyLink_Status status) {
#ifdef IS_USE_PA_LNA
    aw_lna_enable(false);
    aw_pa_enable(false);
#endif
    if (status == EasyLink_Status_Success) {
        Easylink_cmdIds_t _cmdId = (Easylink_cmdIds_t)rxPacket->payload[0];
        switch (_cmdId) {
            case Easylink_cmdIds_configReq:
            case Easylink_cmdIds_config:
            case Easylink_cmdIds_notify:
            case Easylink_cmdIds_sensor_dataReq:
            case Easylink_cmdIds_sensor_data:
            case Easylink_cmdIds_IOReq:
            case Easylink_cmdIds_status_dataReq:
            case Easylink_cmdIds_status_data:
            case Easylink_cmdIds_pairReq:
                _easylink_status = Easylink_status_free;
                if (NULL != _easylink_rxCb) {
                    _easylink_rxCb(rxPacket, status);
                } else {
                    if (_easylinkSem != NULL) {
                        Semaphore_post(_easylinkSem);
                    }
                }
                break;
            case Easylink_cmdIds_configRsp:
            case Easylink_cmdIds_sensor_dataRsp:
            case Easylink_cmdIds_status_dataRsp:
            case Easylink_cmdIds_IORsp:
            case Easylink_cmdIds_notifyRsp:
            case Easylink_cmdIds_pairRsp:
                if (_easylinkSem != NULL) {
                    Semaphore_post(_easylinkSem);
                }
                break;
            default:
                break;
        }
    } else if (status == EasyLink_Status_Aborted) {
        if (_easylinkSem != NULL) {
            Semaphore_post(_easylinkSem);
        }
    } else {
        if (NULL != _easylink_rxCb) {
            _easylink_rxCb(rxPacket, status);
        }
    }
}

static bool _easylink_init() {
    EasyLink_Params easyLink_params;
    EasyLink_Params_init(&easyLink_params);
    if (EasyLink_init(&easyLink_params) != EasyLink_Status_Success) {
        return false;
    }
    return true;
}

static EasyLink_TxPacket _txPacket = {{0}, 0, 0, {0}};
#define AW_ABORT_TIMEOUT_MS (300 * 1000 / Clock_tickPeriod)  // 300ms
static bool _waiting_abort() {
    if (true != Semaphore_pend(_easylinkSem, AW_ABORT_TIMEOUT_MS)) {
        return false;
    }
    return true;
}

static bool _easylink_tx(EasyLink_TxPacket* txPacket, bool isAck) {
    bool isOK = false;
    if (Easylink_status_rxing == _easylink_status) {
        if (EasyLink_abort() == EasyLink_Status_Success) {
            _easylink_status = Easylink_status_waitRxAborting;
            _waiting_abort();
        } else {
            isOK = false;
        }
    }
    if (Easylink_status_txing == _easylink_status) {
        isOK = false;
    }
    _easylink_status = Easylink_status_txing;
#ifdef IS_USE_PA_LNA
    aw_pa_enable(true);
#endif
    EasyLink_Status status = EasyLink_transmitAsync(txPacket, _easylink_txDoneCb);
    if (status != EasyLink_Status_Success) {
        isOK = false;
    }
    isOK = Semaphore_pend(_easylinkSem, _TX_TIMEOUT_MS);
    if (isOK) {
        if (isAck) {
#ifdef IS_USE_PA_LNA
            aw_pa_enable(false);
#endif
            _easylink_status = Easylink_status_waitAcking;
            EasyLink_receiveAsync(_easylink_rxDoneCb, 0);
            isOK = Semaphore_pend(_easylinkSem, _RX_TIMEOUT_MS);
            if (isOK) {

            } else {  //接收超时，取消接收
                if (EasyLink_abort() == EasyLink_Status_Success) {
                    _easylink_status = Easylink_status_waitAckAborting;
                    _waiting_abort();
                }
            }
        } else {
#ifdef IS_USE_PA_LNA
            aw_pa_enable(false);
#endif
        }
    } else {  //发送超时,取消发送
        if (EasyLink_abort() == EasyLink_Status_Success) {
            _easylink_status = Easylink_status_waitTxAborting;
            _waiting_abort();
        }
#ifdef IS_USE_PA_LNA
        aw_pa_enable(false);
#endif
    }
    //_easylink_status = Easylink_status_free;
    return isOK;
}

static bool _easylink_send(u8* pData, u8 len, u16 dstAddr, bool isAck) {
    bool isTxRxOK = false;
    memcpy(_txPacket.payload, pData, len);
    _txPacket.len        = len;
    _txPacket.dstAddr[0] = dstAddr & 0xFF;
    _txPacket.dstAddr[1] = dstAddr >> 8 & 0xFF;
    isTxRxOK             = _easylink_tx(&_txPacket, isAck);
    return isTxRxOK;
}

bool aw_easylink_send_with_ack(u8* pData, u8 len, u16 dstAddr) {
#ifdef IS_USE_EASYLINK_ACK
    return _easylink_send(pData, len, dstAddr, true);
#else
    return _easylink_send(pData, len, dstAddr, false);
#endif
}

bool aw_easylink_init(aw_easylink_rxCB_t rxCb) {
    _easylink_rxCb = rxCb;
#if defined(IS_USE_RX) | defined(IS_USE_PAIR)
    aw_easylink_init_buf();
#endif
    _easylink_sem_init();
    bool isok = _easylink_init();
    if (isok) {
        _easylink_status = Easylink_status_init;
    }
    return isok;
}

bool aw_easylink_rx() {
    bool isOK = true;
#ifdef IS_USE_PA_LNA
    aw_pa_enable(true);
    aw_lna_enable(false);
#endif
    EasyLink_Status status = EasyLink_receiveAsync(_easylink_rxDoneCb, 0);
    if (EasyLink_Status_Success != status) {
        isOK = false;
    }
    _easylink_status = Easylink_status_rxing;
    return isOK;
}

bool aw_easylink_send_ack(Easylink_ACK_t* ack, u16 dstAddr) {
    u8 payload[4] = {0};
    payload[0]    = ack->ackCmdId;
    payload[1]    = ack->id >> 8 & 0xFF;
    payload[2]    = ack->id & 0xFF;
    payload[3]    = ack->type;
    bool isOK     = _easylink_send(payload, sizeof(payload), dstAddr, false);
    return isOK;
}

#define _AW_EASYLINK_CONFIG_SIZE 9

#if _AW_EASYLINK_CONFIG_SIZE > EASYLINK_MAX_DATA_LENGTH
#err
#endif

int aw_configToPayload(Aw_Config_t* config, u8* payload) {
    payload[0] = Easylink_cmdIds_configReq;
    payload[1] = config->id >> 8 & 0xFF;
    payload[2] = config->id & 0xFF;
    payload[3] = config->bat_cur_set >> 8 & 0xFF;
    payload[4] = config->bat_cur_set & 0xFF;
    payload[5] = config->type;
    payload[6] = config->cmd;
    payload[7] = config->bat_vol_set >> 8 & 0xFF;
    payload[8] = config->bat_vol_set & 0xFF;

    return _AW_EASYLINK_CONFIG_SIZE;
}

void aw_payloadToConfig(u8* payload, Aw_Config_t* config) {
    config->id          = payload[1] << 8 | payload[2];
    config->bat_cur_set = payload[3] << 8 | payload[4];
    config->type        = payload[5];
    config->cmd         = payload[6];
    config->bat_vol_set = payload[7] << 8 | payload[8];
}
#define _AW_EASYLINK_NOTIFY_SIZE 6

#if _AW_EASYLINK_NOTIFY_SIZE > EASYLINK_MAX_DATA_LENGTH
#err
#endif

int aw_notifyToPayload(Easylink_Notify_t* notify, u8* payload) {
    payload[0] = Easylink_cmdIds_notify;
    payload[1] = notify->id >> 8 & 0xFF;
    payload[2] = notify->id & 0xFF;
    payload[3] = notify->cnt >> 8 & 0xFF;
    payload[4] = notify->cnt & 0xFF;
    payload[5] = notify->type;
    return _AW_EASYLINK_NOTIFY_SIZE;
}

void aw_payloadToNotify(u8* payload, Easylink_Notify_t* notify) {
    notify->id   = payload[1] << 8 | payload[2];
    notify->cnt  = payload[3] << 8 | payload[4];
    notify->type = payload[5];
}

bool aw_easylink_send_notify(Easylink_Notify_t* notify, u16 dstAddr) {
    u8 payload[_AW_EASYLINK_NOTIFY_SIZE] = {0};
    aw_notifyToPayload(notify, payload);
    return _easylink_send(payload, sizeof(payload), dstAddr, true);
}

#define _AW_EASYLINK_PAIR_SIZE 4
#if _AW_EASYLINK_PAIR_SIZE > EASYLINK_MAX_DATA_LENGTH
#err
#endif

int aw_pairToPayload(Easylink_Pair_t* pair, u8* payload) {
    payload[0] = Easylink_cmdIds_pairReq;
    payload[1] = pair->id >> 8 & 0xFF;
    payload[2] = pair->id & 0xFF;
    payload[3] = pair->type;
    return _AW_EASYLINK_PAIR_SIZE;
}

void aw_payloadToPair(u8* payload, Easylink_Pair_t* pair) {
    pair->id   = payload[1] << 8 | payload[2];
    pair->type = payload[3];
}

#define _AW_EASYLINK_SENSOR_SIZE 74
#if _AW_EASYLINK_SENSOR_SIZE > EASYLINK_MAX_DATA_LENGTH
#err
#endif

int aw_sensorToPayload(Aw_Sensor_t* sensor, u8* payload) {
    u16 i = 0;
    u16 j = 14;
    payload[0]  = Easylink_cmdIds_sensor_data;
    payload[1]  = (sensor->src_id >> 8) & 0xFF;
    payload[2]  = (sensor->src_id & 0xFF);
    payload[3]  = sensor->id >> 8 & 0xFF;
    payload[4]  = sensor->id & 0xFF;
    payload[5]  = (sensor->Xval >> 8) & 0xFF;
    payload[6]  = sensor->Xval & 0xFF;
    payload[7]  = (sensor->Yval >> 8) & 0xFF;
    payload[8]  = (sensor->Yval & 0xFF);
    payload[9]  = (sensor->Zval >> 8) & 0xFF;
    payload[10] = (sensor->Zval & 0xFF);
    payload[11] = sensor->type;
    payload[12] = (sensor->temp >> 8) & 0xFF;
    payload[13] = (sensor->temp & 0xFF);
    for(i = 0; i < 30; i++) {

        payload[j+i] = (sensor->axisArray[i] >> 8) & 0xFF;
        j = j + 1;
        payload[j+i] = (sensor->axisArray[i] & 0xFF);

    }
//    payload[14] = (sensor->axisArray[0] >> 8) & 0xFF;
//    payload[15] = (sensor->axisArray[0] & 0xFF);
//    payload[16] = (sensor->axisArray[1] >> 8) & 0xFF;
//    payload[17] = (sensor->axisArray[1] & 0xFF);
//    payload[18] = (sensor->axisArray[2] >> 8) & 0xFF;
//    payload[19] = (sensor->axisArray[2] & 0xFF);
    return _AW_EASYLINK_SENSOR_SIZE;
}

void aw_payloadToSensor(u8* payload, Aw_Sensor_t* sensor) {
    u16 i = 0;
    u16 j = 14;
    sensor->src_id = payload[1] << 8 | payload[2];
    sensor->id     = payload[3] << 8 | payload[4];
    sensor->Xval    = payload[5] << 8 | payload[6];
    sensor->Yval    = (payload[7] << 8) | payload[8];
    sensor->Zval    = payload[9] << 8 | payload[10];
    sensor->type   = payload[11];
    sensor->temp = (payload[12] << 8) | payload[13];
    for(i = 0; i < 30; i++) {
        sensor->axisArray[i] = (payload[j+i] << 8);
        j = j + 1;
        sensor->axisArray[i] = (sensor->axisArray[i]) | payload[j+i];
    }
//    sensor->axisArray[0] = (payload[14] << 8) | payload[15];
//    sensor->axisArray[1] = (payload[16] << 8) | payload[17];
//    sensor->axisArray[2] = (payload[18] << 8) | payload[19];
}

bool aw_easylink_send_sensor(Aw_Sensor_t* sensor, u16 dstAddr) {
    u8 payload[_AW_EASYLINK_SENSOR_SIZE] = {0};
    aw_sensorToPayload(sensor, payload);
    return _easylink_send(payload, sizeof(payload), dstAddr, true);
}

#define _AW_EASYLINK_STATUS_DATA_SIZE 23
#if _AW_EASYLINK_STATUS_DATA_SIZE > EASYLINK_MAX_DATA_LENGTH
#err
#endif

int aw_statusDataToPayload(Easylink_Status_data_t* status, u8* payload) {
    payload[0] = Easylink_cmdIds_status_data;
    payload[1] = status->id >> 8 & 0xFF;
    payload[2] = status->id & 0xFF;
    payload[3] = (status->bat >> 8) & 0xFF;
    payload[4] = (u8)(status->bat & 0xFF);

    payload[5] = (status->cur[0] >> 8) & 0xFF;
    payload[6] = (u8)(status->cur[0] & 0xFF);
    payload[7] = (status->cur[1] >> 8) & 0xFF;
    payload[8] = (u8)(status->cur[1] & 0xFF);

    payload[9]  = (status->cur[2] >> 8) & 0xFF;
    payload[10] = (u8)(status->cur[2] & 0xFF);

    payload[11] = (u8)status->chs[0];
    payload[12] = (u8)status->chs[1];
    payload[13] = (u8)status->chs[2];

    payload[14] = (status->bat_c >> 8) & 0xFF;
    payload[15] = (u8)(status->bat_c & 0xFF);
    payload[16] = (status->bus >> 8) & 0xFF;
    payload[17] = (u8)(status->bus & 0xFF);
    payload[18] = status->type;
    payload[19] = (status->version >> 8) & 0xFF;
    payload[20] = status->version & 0xFF;
    payload[21] = (status->bat_v >> 8) & 0xFF;
    payload[22] = status->bat_v & 0xFF;
    return _AW_EASYLINK_STATUS_DATA_SIZE;
}

void aw_payloadToStatusData(u8* payload, Easylink_Status_data_t* status) {
    status->id      = payload[1] << 8 | payload[2];
    status->bat     = (payload[3] << 8) | payload[4];
    status->cur[0]  = (payload[5] << 8) | payload[6];
    status->cur[1]  = (payload[7] << 8) | payload[8];
    status->cur[2]  = (payload[9] << 8) | payload[10];
    status->chs[0]  = (E_sw_sts_t)payload[11];
    status->chs[1]  = (E_sw_sts_t)payload[12];
    status->chs[2]  = (E_sw_sts_t)payload[13];
    status->bat_c   = (payload[14] << 8) | payload[15];
    status->bus     = (payload[16] << 8) | payload[17];
    status->type    = payload[18];
    status->version = (payload[19] << 8) | payload[20];
    status->bat_v   = (payload[21] << 8) | payload[22];
}

#define _AW_EASYLINK_IO_SIZE 9
#if _AW_EASYLINK_IO_SIZE > EASYLINK_MAX_DATA_LENGTH
#err
#endif

int aw_ioToPayload(Easylink_IO_t* io, u8* payload) {
    payload[0] = Easylink_cmdIds_IOReq;
    payload[1] = io->id >> 8 & 0xFF;
    payload[2] = io->id & 0xFF;
    payload[3] = io->chs[AW_SW_CH1];
    payload[4] = io->chs[AW_SW_CH2];
    payload[5] = io->chs[AW_SW_CH3];
    payload[6] = (io->delay >> 8) & 0xFF;
    payload[7] = (u8)(io->delay & 0xFF);
    payload[8] = io->type;
    return _AW_EASYLINK_IO_SIZE;
}

void aw_payloadToIO(u8* payload, Easylink_IO_t* io) {
    io->id             = payload[1] << 8 | payload[2];
    io->chs[AW_SW_CH1] = (E_sw_sts_t)payload[3];
    io->chs[AW_SW_CH2] = (E_sw_sts_t)payload[4];
    io->chs[AW_SW_CH3] = (E_sw_sts_t)payload[5];
    io->delay          = (payload[6] << 8) | payload[7];
    io->type           = payload[8];
}

bool aw_easylink_send_io(Easylink_IO_t* io, u16 dstAddr) {
    u8 payload[_AW_EASYLINK_IO_SIZE] = {0};
    aw_ioToPayload(io, payload);
    return _easylink_send(payload, sizeof(payload), dstAddr, true);
}
