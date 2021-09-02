/*
 * util_byteStrem.h
 *
 *  Created on: 2020Äê9ÔÂ8ÈÕ
 *      Author: admin
 */

#ifndef UTIL_UTIL_BYTEBUF_H_
#define UTIL_UTIL_BYTEBUF_H_
#include "util_def.h"

#define BYTE_BUF_SIZE 200
struct _ByteBuf_t;
typedef u8   Byte_t;
typedef char Char_t;
typedef struct _ByteBuf_ops_t {
    s32 (*getUsed)(struct _ByteBuf_t* buf);
    s32 (*getUsedWithSp)(struct _ByteBuf_t* buf, s32 tgtSp);
    s32 (*getUsedWithEp)(struct _ByteBuf_t* buf, s32 tgtSp);
    s32 (*getFree)(struct _ByteBuf_t* buf);
    void (*reset)(struct _ByteBuf_t* buf);
    bool (*push)(struct _ByteBuf_t* buf, Byte_t byte, bool force);
    Byte_t (*pop)(struct _ByteBuf_t* buf);
} ByteBuf_ops_t;

struct _ByteBuf_t {
    Byte_t        buf[BYTE_BUF_SIZE];
    s32           sp;    //¶Á
    s32           ep;    //Ð´
    s32           size;  //
    ByteBuf_ops_t ops;
};
typedef struct _ByteBuf_t ByteBuf_t;

s8 byteBufInit(ByteBuf_t* buf);
#endif /* UTIL_UTIL_BYTEBUF_H_ */
