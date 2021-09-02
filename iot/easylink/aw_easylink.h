/*
 * aw_easylink.h
 *
 *  Created on: 2020Äê9ÔÂ23ÈÕ
 *      Author: admin
 */

#ifndef EASYLINK_AW_EASYLINK_H_
#define EASYLINK_AW_EASYLINK_H_
#include <iot/easylink/aw_easylink_def.h>
#include <iot/easylink/EasyLink.h>
#include "aw_config.h"
#include "util_def.h"
#include <ti/sysbios/knl/Semaphore.h>
typedef void (*aw_easylink_rxCB_t)(EasyLink_RxPacket* rxPacket, EasyLink_Status status);
bool              aw_easylink_init(aw_easylink_rxCB_t rxCb);
bool              aw_easylink_rx();
Easylink_Status_t aw_easylink_status_get();
u16               aw_easylink_pair_get();
void              aw_easylink_pair_set(u16 pair);
bool              aw_easylink_send_with_ack(u8* pData, u8 len, u16 dstAddr);
bool              aw_easylink_send_ack(Easylink_ACK_t* ack, u16 dstAddr);
bool              aw_easylink_send_notify(Easylink_Notify_t* notify, u16 dstAddr);
bool              aw_easylink_send_sensor(Aw_Sensor_t* sensor, u16 dstAddr);
bool              aw_easylink_send_io(Easylink_IO_t* io, u16 dstAddr);
void              aw_payloadToIO(u8* payload, Easylink_IO_t* io);
int               aw_ioToPayload(Easylink_IO_t* io, u8* payload);
void              aw_payloadToSensor(u8* payload, Aw_Sensor_t* sensor);
int               aw_sensorToPayload(Aw_Sensor_t* sensor, u8* payload);

int     aw_configToPayload(Aw_Config_t* config, u8* payload);
void    aw_payloadToConfig(u8* payload, Aw_Config_t* config);
int     aw_pairToPayload(Easylink_Pair_t* pair, u8* payload);
void    aw_payloadToPair(u8* payload, Easylink_Pair_t* pair);
int     aw_statusDataToPayload(Easylink_Status_data_t* status, u8* payload);
void    aw_payloadToStatusData(u8* payload, Easylink_Status_data_t* status);
void    aw_payloadToNotify(u8* payload, Easylink_Notify_t* notify);
int     aw_notifyToPayload(Easylink_Notify_t* notify, u8* payload);
AW_BOOL aw_easylink_put_buf(EasyLink_RxPacket* data);
AW_BOOL aw_easylink_read_buf(EasyLink_RxPacket* data);
AW_BOOL aw_easylink_init_buf();
#define AW_RX_TIMEOUT_MS (3 * 1000)  // 2s
#endif /* EASYLINK_AW_EASYLINK_H_ */
