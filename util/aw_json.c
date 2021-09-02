/*
 * aw_json.c
 *
 *  Created on: 2020年9月26日
 *      Author: admin
 */

#include "aw_json.h"
#include <string.h>
#include <stdio.h>
void json_init(JsonBuf_t* buf) {
    buf->head = 0;
    buf->tail = 0;
    buf->next = 1;
    buf->size = AW_JSON_BUF_SIZE;
}
bool parse_json(JsonBuf_t* buf, const char* str) {
    char* pStr = NULL;
    if (buf->head == buf->next) {
        return false;
    }
    pStr      = buf->buf[buf->tail].str;
    pStr[0]   = '\0';  //原始数据清空
    char* pS  = NULL;
    char* pE  = NULL;
    int   len = 0;
    if ((pS = strstr(str, "{")) && (pE = strstr(pS, "}"))) {
        len = pE - pS + 1;
        if (len > 0) {
            len = (len < (AW_JSON_STR_SIZE - 1)) ? len : (AW_JSON_STR_SIZE - 1);
            memcpy(pStr, pS, len);
            pStr[len] = '\0';
        } else {
            return false;
        }
    } else {
        return false;
    }
    buf->tail = buf->next;
    buf->next++;
    if (buf->next == buf->size) {
        buf->next = 0;
    }
    return true;
}
bool read_json(JsonBuf_t* buf, char* json_str) {
    char* pStr = NULL;
    if (buf->head == buf->tail) {
        return false;
    }
    json_str[0] = '\0';  //清空原始数据
    pStr    = buf->buf[buf->head].str;
    int len = strlen(pStr);
    if (len > 0) {
        len = (len < (AW_JSON_STR_SIZE - 1)) ? len : (AW_JSON_STR_SIZE - 1);
        memcpy(json_str, pStr, len);
        json_str[len] = '\0';
    }
    buf->head++;
    if (buf->head == buf->size) {
        buf->head = 0;
    }
    return true;
}
