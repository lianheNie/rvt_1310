/*
 * aw_json.h
 *
 *  Created on: 2020Äê9ÔÂ26ÈÕ
 *      Author: admin
 */

#ifndef UTIL_AW_JSON_H_
#define UTIL_AW_JSON_H_

#include "util_def.h"
#define AW_JSON_STR_SIZE 64
#define AW_JSON_BUF_SIZE  3
typedef struct {
    char str[AW_JSON_STR_SIZE];
} Json_t;
typedef struct {
    Json_t buf[AW_JSON_BUF_SIZE];
    u8     head;
    u8     tail;
    u8     next;  // tail next
    u8     size;
} JsonBuf_t;
void json_init(JsonBuf_t* buf);
bool parse_json(JsonBuf_t* buf, const char* str);
bool read_json(JsonBuf_t* buf, char* json);

#endif /* UTIL_AW_JSON_H_ */
