/*
 * aw_easylink_def.h
 *
 *  Created on: 2020��9��23��
 *      Author: admin
 */

#ifndef EASYLINK_AW_EASYLINK_DEF_H_
#define EASYLINK_AW_EASYLINK_DEF_H_
#include "util_def.h"
#define EASYLINK_MAX_DATA_LENGTH 80
typedef enum {
    Easylink_cmdIds_configReq = 1,  //��������
    Easylink_cmdIds_configRsp,      //����Ӧ��
    Easylink_cmdIds_config,         //��������

    Easylink_cmdIds_sensor_data,     //�������ݽ���
    Easylink_cmdIds_sensor_dataRsp,  //��������Ӧ��
    Easylink_cmdIds_sensor_dataReq,  //��������Ӧ��

    Easylink_cmdIds_status_data,     //״̬���ݽ���
    Easylink_cmdIds_status_dataRsp,  //״̬����Ӧ��
    Easylink_cmdIds_status_dataReq,  //״̬��������

    Easylink_cmdIds_IOReq,  // IO����
    Easylink_cmdIds_IORsp,  // IOӦ��

    Easylink_cmdIds_notify,     // ����֪ͨ����
    Easylink_cmdIds_notifyRsp,  // ����֪ͨӦ��

    Easylink_cmdIds_pairReq,  // �������
    Easylink_cmdIds_pairRsp   // ���Ӧ��

} Easylink_cmdIds_t;

typedef enum {
    Easylink_cmdId = 0,  //��������
    Easylink_payload
} Easylink_packet_t;

#define AW_CPU_RESET_BIT   0
#define AW_NB_RESET_BIT    1
#define AW_LED_SET_BIT     2
#define AW_NET_LED_SET_BIT 3
typedef struct _Easylink_config_t {
    u8  type;
    u16 id;
    u16 bat_cur_set;
    u16 bat_vol_set;
    u8  cmd;
} Aw_Config_t;

typedef struct _Easylink_Status_data_t {
    u8  type;
    u16 id;
    u16 version;

    E_sw_sts_t chs[3];  // 12v 5v 4v��ͨ������״̬
    u16        bat;
    u16        cur[3];  // 12v 5v 4v��ͨ������
    u16        bat_c;   //��ص���
    u16        bus;     //��ѹ����
    u16        bat_v;   //����ѹ

} Easylink_Status_data_t;

typedef struct _Easylink_IO_t {
    u8         type;
    u16        id;
    E_sw_sts_t chs[3];
    uint16_t   delay;
} Easylink_IO_t;

typedef struct _Easylink_ACK_t {
    u8                type;
    Easylink_cmdIds_t ackCmdId;
    u16               id;
} Easylink_ACK_t;

typedef struct _Easylink_PAIR_t {
    u8  type;
    u16 id;
} Easylink_Pair_t;

typedef struct _Easylink_TX_t {
    Easylink_cmdIds_t ackCmdId;
    u8                payload[EASYLINK_MAX_DATA_LENGTH];
    u8                len;
    u16               dstAdd;
} Easylink_TX_t;

typedef enum {
    Easylink_powerOn = 0,  //��������
    Easylink_lowpower
} Easylink_NotifyStatus_t;

typedef struct _Easylink_Notify_t {
    u8                      type;
    u16                     id;
    Easylink_NotifyStatus_t sts;
    uint16_t                bat;
    u16                     cnt;
} Easylink_Notify_t;
typedef enum {
    Easylink_status_rxing,
    Easylink_status_txing,
    Easylink_status_waitAcking,
    Easylink_status_waitRxAborting,
    Easylink_status_waitTxAborting,
    Easylink_status_waitAckAborting,
    Easylink_status_init,
    Easylink_status_free,
    Easylink_status_null
} Easylink_Status_t;
#define AW_DEV_LOWPOWER 0
#endif /* EASYLINK_AW_EASYLINK_DEF_H_ */
