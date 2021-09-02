/*
 * util_buffer.c
 *
 *  Created on: 2020Äê9ÔÂ7ÈÕ
 *      Author: admin
 */
#include "util_buffer.h"
AW_BOOL putBuf(BufDatas_t* buf, BufData_t* data) {
    if (buf->head == buf->next) {
        return AW_FALSE;
    }
    buf->buf[buf->tail] = *data;
    buf->tail           = buf->next;
    buf->next++;
    if (buf->next == BUF_SIZE) {
        buf->next = 0;
    }
    return AW_TRUE;
}
AW_BOOL readBuf(BufDatas_t* buf, BufData_t* data) {
    if (buf->head == buf->tail) {
        return AW_FALSE;
    }
    *data = buf->buf[buf->head];
    buf->head++;
    if (buf->head == BUF_SIZE) {
        buf->head = 0;
    }
    return AW_TRUE;
}
u8 getBufCnt(BufDatas_t* buf) {
    if (buf->tail >= buf->head) {
        return (buf->tail - buf->head);
    }
    return (BUF_SIZE - (buf->head - buf->tail));
}
AW_BOOL checkFullBuf(BufDatas_t* buf) {
    if (buf->head == buf->next) {
        return AW_TRUE;
    }
    return AW_FALSE;
}
AW_BOOL checkNullBuf(BufDatas_t* buf) {
    if (buf->head == buf->tail) {
        return AW_TRUE;
    }
    return AW_FALSE;
}
#include <string.h>
AW_BOOL resetBuf(BufDatas_t* buf) {
    memset(buf->buf, 0, BUF_SIZE);  // Clear planner struct
    buf->head = 0;
    buf->tail = 0;
    buf->next = 1;
    return AW_TRUE;
}
