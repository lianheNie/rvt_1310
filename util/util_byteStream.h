/*
 * util_byteStream.h
 *
 *  Created on: 2020年9月8日
 *      Author: admin
 */

#ifndef UTIL_UTIL_BYTESTREAM_H_
#define UTIL_UTIL_BYTESTREAM_H_
#include "util_byteBuf.h"
#include <string.h>
#include <stdio.h>
struct _ByteStream_t;
typedef s8 (*uart_writeCb_t)(const void* buffer, size_t size);
typedef struct _readWrite_ops_t {
    s32 (*readOffset)(struct _ByteStream_t* stream, u8* rBuffer, s32 rOffset, s32 rSize);
    s32 (*read)(struct _ByteStream_t* stream, u8* rBuffer, s32 rSize);
    s32 (*readAll)(struct _ByteStream_t* stream, u8* rBuffer, s32 maxSize);
    u8 (*readByte)(struct _ByteStream_t* stream);
    s8 (*readStr)(struct _ByteStream_t* stream, char* str, const char* splitStr, s32* size,
                  bool keepSplitStr);

    s32 (*writeOffset)(struct _ByteStream_t* stream, u8* wBuffer, s32 wOffset, s32 wSize);
    s32 (*write)(struct _ByteStream_t* stream, u8* wBuffer, s32 wSize);
    s32 (*writeByte)(struct _ByteStream_t* stream, u8 wData);
    s32 (*writeStr)(struct _ByteStream_t* stream, char* str);

    // s32 printf(const char* format, ...) __attribute__((format(printf, 2, 3)));
    s32 (*Printf)(struct _ByteStream_t* stream, const char* format, ...);
    s32 (*vPrintf)(struct _ByteStream_t* stream, const char* format, va_list va);

    s32 (*availableRead)(struct _ByteStream_t* stream);
    s32 (*availableWrite)(struct _ByteStream_t* stream);
    s32 (*discardRead)(struct _ByteStream_t* stream, s32 num);
    s32 (*discardWrite)(struct _ByteStream_t* stream, s32 num);
    bool (*isWriteIdle)(struct _ByteStream_t* stream);
    bool (*waitWriteIdle)(struct _ByteStream_t* stream, int64_t timeoutMs);
} ReadWrite_ops_t;

typedef struct _byteStream_ops_t {
    ReadWrite_ops_t baseOps;
    /////read fuc///////
    s8 (*readLine)(struct _ByteStream_t* stream, char* line, s32* size);
    u8 (*peekWithSp)(struct _ByteStream_t* stream, int32_t sp);
    u8 (*peek)(struct _ByteStream_t* stream);
    u8 (*peekNextDigital)(struct _ByteStream_t* stream, int32_t sp, uint8_t ignore);
    s32 (*peekStrPos)(struct _ByteStream_t* stream, const char* str);
    ///////////////////

    /////write fuc///////
    s32 (*touchOffset)(struct _ByteStream_t* stream, u8* wBuffer, s32 wOffset, s32 wSize);
    s32 (*touch)(struct _ByteStream_t* stream, u8* wBuffer, s32 wSize);
    s32 (*touchByte)(struct _ByteStream_t* stream, u8 wData);
    //将缓冲区的数据发送出去 当发送繁忙时应返回true 只有当发送超时时才应该返回false
    bool (*flushWrite)(struct _ByteStream_t* stream);
    /////////////////////
    s32 (*buffededWrite)(struct _ByteStream_t* stream);
    bool (*nextInt)(struct _ByteStream_t* stream, s64* num, u8 ignore);
    bool (*nextFloat)(struct _ByteStream_t* stream, double* flo, u8 ignore);
    void (*doSendedCallBack)(struct _ByteStream_t* stream);
    void (*doReceivedCallBack)(struct _ByteStream_t* stream);

} ByteStream_ops;
struct _ByteStream_t {
    ByteBuf_t      rBuf;
    ByteBuf_t      tBuf;
    bool           txBusy;
    ByteStream_ops ops;
};
typedef struct _ByteStream_t ByteStream_t;
s8                           byteStreamInit(struct _ByteStream_t* stream, uart_writeCb_t writeCb);
#endif /* UTIL_UTIL_BYTESTREAM_H_ */
