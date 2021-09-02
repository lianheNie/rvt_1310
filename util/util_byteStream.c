/*
 * util_byteStream.c
 *
 *  Created on: 2020年9月8日
 *      Author: admin
 */
#include "util_byteStream.h"
#include <stdlib.h>
static s32 readOffset(struct _ByteStream_t* stream, u8* rBuffer, s32 rOffset, s32 rSize) {
    s32 avaLen = stream->ops.baseOps.availableRead(stream);
    //判断读取是否越界，记录可读取长度
    s32 rLen = 0;
    avaLen   = rSize <= avaLen ? rSize : avaLen;
    while (avaLen != 0) {
        s32 avaCopySize = (s32)(stream->rBuf.size - stream->rBuf.sp);
        s32 copySize    = avaLen <= avaCopySize ? avaLen : avaCopySize;
        //拷贝字节数组
        memcpy(rBuffer + rOffset, (void*)(stream->rBuf.buf + stream->rBuf.sp), copySize);
        //偏移读取指针
        stream->rBuf.sp = (stream->rBuf.sp + copySize) % stream->rBuf.size;
        //移动已读取指针
        rOffset += copySize;
        rLen += copySize;
        //长度减去已读取长度
        avaLen -= copySize;
    }
    return rLen;
}
static s32 read(struct _ByteStream_t* stream, u8* rBuffer, s32 rSize) {
    return stream->ops.baseOps.readOffset(stream, rBuffer, 0, rSize);
}
static s32 readAll(struct _ByteStream_t* stream, u8* rBuffer, s32 maxSize) {
    int32_t ava = stream->ops.baseOps.availableRead(stream);
    return stream->ops.baseOps.read(stream, rBuffer, ava <= maxSize ? ava : maxSize);
}

static u8 readByte(struct _ByteStream_t* stream) {
    uint8_t byte = 0;
    stream->ops.baseOps.read(stream, &byte, 1);
    return byte;
}
static s8 readStr(struct _ByteStream_t* stream, char* str, const char* splitStr, s32* size,
                  bool keepSplitStr) {
    int32_t sizeSp   = stream->ops.peekStrPos(stream, splitStr);
    int32_t splitLen = strlen(splitStr);
    if (sizeSp == -1) {
        return -1;
    }
    if (keepSplitStr) {
        sizeSp += splitLen;
    }
    //判断大小是否大于字符串长度+\0
    if ((sizeSp + 1) >= *size) {
        *size = sizeSp;
        return -2;
    }

    if (stream->ops.baseOps.read(stream, (uint8_t*)str, sizeSp) != sizeSp) {
        return -3;
    }
    str[sizeSp] = '\0';
    *size       = sizeSp;
    return 0;
}

static s32 writeOffset(struct _ByteStream_t* stream, u8* wBuffer, s32 wOffset, s32 wSize) {
    s32 wLen = 0;
    // int64_t last = uTick.Millis();
    while (wSize != 0) {
        s32 singleTouch = stream->ops.touchOffset(stream, wBuffer, wOffset, wSize);
        if (singleTouch == -1) {
            return -1;
        }
        if (singleTouch == 0) {
            s32 buffered = stream->ops.buffededWrite(stream);
            if (buffered == 0) {
                //                if ((uTick.Millis() - last) > _writeTimeoutSpan) {
                //                    //写入超时
                //                    _writeTimoutFlag = true;
                //                    return -1;
                //                }
                //
                continue;
            } else {
                //有之前缓冲好的数据，尝试Flush
                // ULOG_D("Write: there has buffered data->%d", int(buffered));
            }
        }

        if (!stream->ops.flushWrite(stream)) {
            // ULOG_W("Write: Flush timeout or error");
            //传输超时或发生了错误
            return -1;
        }

        // last = uTick.Millis();
        wLen += singleTouch;
        wOffset += singleTouch;
        wSize -= singleTouch;
    }
    return wLen;
}
static s32 write(struct _ByteStream_t* stream, u8* wBuffer, s32 wSize) {
    return stream->ops.baseOps.writeOffset(stream, wBuffer, 0, wSize);
}
static s32 writeByte(struct _ByteStream_t* stream, u8 wData) {
    return stream->ops.baseOps.writeOffset(stream, &wData, 0, 1);
}
static s32 writeStr(struct _ByteStream_t* stream, char* str) {
    return stream->ops.baseOps.writeOffset(stream, (uint8_t*)str, 0, strlen(str));
}
static s32 Printf(struct _ByteStream_t* stream, const char* format, ...) {
    s32     wLen;
    va_list va;
    va_start(va, format);
    wLen = stream->ops.baseOps.vPrintf(stream, format, va);
    va_end(va);
    return wLen;
}

// static s32 vPrintf(struct _ByteStream_t* stream, const char* format, va_list va) {
//    s32    wLen;
//    char   buf[64];                    //默认使用64字节的区块作为发送缓冲
//    size_t bufSize = sizeof(buf) - 1;  //留出\0空间
//    int    cnt     = vsnprintf(buf, bufSize, format, va);
//    wLen           = stream->ops.baseOps.write(stream, (uint8_t*)buf, bufSize);
//    return wLen;
//}

static s32 vPrintf(struct _ByteStream_t* stream, const char* format, va_list va) {
    int32_t wLen;
    char*   buf = (char*)&stream->tBuf.buf[stream->tBuf.ep];  //直接使用txBuf作为缓冲，这样效率最高
    size_t  bufSize =
        stream->tBuf.size - stream->tBuf.ep - 1;  //缓冲到缓冲末尾的可用空间大小,留出\0空间
    int cnt = vsnprintf(buf, bufSize, format, va);
    stream->tBuf.ep += cnt;
    wLen = cnt;
    stream->ops.flushWrite(stream);
    return wLen;
}

static s32 availableRead(struct _ByteStream_t* stream) {
    return stream->rBuf.ops.getUsed(&stream->rBuf);
}
static s32 availableWrite(struct _ByteStream_t* stream) {
    return stream->tBuf.ops.getUsed(&stream->tBuf);
}

static s32 discardRead(struct _ByteStream_t* stream, s32 num) {
    s32 avaLen = stream->rBuf.ops.getUsed(&stream->rBuf);
    if ((num != 0) && (num < avaLen)) {
        //如果丢弃的字节小于缓冲剩余字节数，丢弃相应的字节并退出
        stream->rBuf.sp = (stream->rBuf.sp + num) % stream->rBuf.size;
        return num;
    } else {
        //丢弃所有数据
        stream->rBuf.sp = stream->rBuf.ep;
        return avaLen;
    }
}
static s32 discardWrite(struct _ByteStream_t* stream, s32 num) {
    int32_t avaLen = stream->tBuf.ops.getUsed(&stream->tBuf);
    if ((num != 0) && (num < avaLen)) {
        //如果丢弃的字节小于缓冲剩余字节数，丢弃相应的字节并退出
        stream->tBuf.sp = (stream->tBuf.sp + num) % stream->tBuf.size;
        return num;
    } else {
        //丢弃所有数据
        stream->tBuf.sp = stream->tBuf.ep;
        return avaLen;
    }
}
static bool isWriteIdle(struct _ByteStream_t* stream) { return stream->tBuf.ep == stream->tBuf.sp; }
static bool waitWriteIdle(struct _ByteStream_t* stream, int64_t timeoutMs) { return true; }
static s8   readLine(struct _ByteStream_t* stream, char* line, s32* size) {
    return stream->ops.baseOps.readStr(stream, line, "\r\n", size, true);
}
static u8 peekWithSp(struct _ByteStream_t* stream, int32_t sp) { return stream->rBuf.buf[sp]; }
static u8 peek(struct _ByteStream_t* stream) {
    return stream->ops.peekWithSp(stream, stream->rBuf.sp);
}
static u8 peekNextDigital(struct _ByteStream_t* stream, s32 sp, u8 ignore) {
    //查看当前流顶字节
    u8 data = stream->ops.peekWithSp(stream, sp);
    //当读到的字符为 '-','+','0'-'9','.'（detectDecimal为true）时返回
    if ((data == '-') || (data == '+') || ((data >= '0') && (data <= '9')) || (data == '.') ||
        (data == ignore)) {
        return data;
    }
    return 0;
}
static s32 peekStrPos(struct _ByteStream_t* stream, const char* str) {
    int32_t i           = 0;
    int32_t machEp      = 0;
    bool    firsttMatch = false;
    bool    mach        = false;
    s32     ep          = 0;
    for (ep = stream->rBuf.sp; ep != stream->rBuf.ep; ep = (ep + 1) % stream->rBuf.size) {
        if (!firsttMatch) {
            //查找匹配的第一个字节
            if (stream->rBuf.buf[ep] == str[0]) {
                //第一个字节匹配
                firsttMatch = true;
                i           = 1;
                machEp      = ep;

                if (str[i] == '\0') {
                    //字符串只有一个字节，并且匹配
                    //全字符串匹配
                    mach = true;
                    break;
                }
            }
        } else {
            if (stream->rBuf.buf[ep] == str[i++]) {
                if (str[i] == '\0') {
                    //当前匹配的字节是最后一个字节
                    //全字符串匹配
                    mach = true;
                    break;
                }
            } else {
                if (stream->rBuf.buf[ep] == str[0]) {
                    //疑似新的字节头，复位指针
                    i      = 1;
                    machEp = ep;
                } else {
                    firsttMatch = false;
                }
            }
        }
    }
    if (mach) {
        return stream->rBuf.ops.getUsedWithEp(&stream->rBuf, machEp);
    } else {
        return -1;
    }
}

/////write fuc///////
static s32 touchOffset(struct _ByteStream_t* stream, u8* wBuffer, s32 wOffset, s32 wSize) {
    int32_t wLen     = 0;
    int32_t freeSize = stream->tBuf.ops.getFree(&stream->tBuf);
    //计算可以发送的字节大小
    int32_t copySize   = wSize <= freeSize ? wSize : freeSize;
    int32_t singleSize = 0;
    //拷贝字节到缓冲区
    while (copySize != 0) {
        int32_t tail = stream->tBuf.size - stream->tBuf.ep;
        // sp读取瞬间需要存档(ep只会在这里修改，安全)，防止中断安全问题。这里只读取了一次，所以不需要考虑
        singleSize =
            (stream->tBuf.sp <= stream->tBuf.ep) ? (copySize <= tail ? copySize : tail) : copySize;
        memcpy((void*)(stream->tBuf.buf + stream->tBuf.ep), wBuffer + wOffset, singleSize);
        stream->tBuf.ep = (stream->tBuf.ep + singleSize) % stream->tBuf.size;
        //偏移掉已发送字节
        wOffset += singleSize;
        wLen += singleSize;
        //长度减去单次已填充长度
        copySize -= singleSize;
    }

    return wLen;
}
static s32 touch(struct _ByteStream_t* stream, u8* wBuffer, s32 wSize) {
    return stream->ops.touchOffset(stream, wBuffer, 0, wSize);
}
static s32 touchByte(struct _ByteStream_t* stream, u8 wData) {
    return stream->ops.touchOffset(stream, &wData, 0, 1);
}
static uart_writeCb_t _writeCb = NULL;
//将缓冲区的数据发送出去 当发送繁忙时应返回true 只有当发送超时时才应该返回false
static bool flushWrite(struct _ByteStream_t* stream) {
    if (!stream->txBusy) {
        s32 _singleSend = stream->tBuf.sp <= stream->tBuf.ep ? stream->tBuf.ep - stream->tBuf.sp
                                                             : stream->tBuf.size - stream->tBuf.sp;
        if (_singleSend != 0) {
            if (_writeCb != NULL) {
                stream->txBusy = true;
                _writeCb((void*)(stream->tBuf.buf + stream->tBuf.sp), _singleSend);
                stream->tBuf.sp = (stream->tBuf.sp + _singleSend) % stream->tBuf.size;
                stream->txBusy  = false;
            }
        }
    }
    return true;
}
/////////////////////
static s32 buffededWrite(struct _ByteStream_t* stream) {
    return stream->tBuf.ops.getUsed(&stream->tBuf);
}
static bool nextInt(struct _ByteStream_t* stream, s64* num, u8 ignore) {
    bool    status    = false;
    bool    firstChar = true;
    bool    isNeg     = false;
    uint8_t c         = 0;
    int64_t n         = 0;
    int32_t sp        = stream->rBuf.sp;
    while (sp != stream->rBuf.ep) {
        c = stream->ops.peekNextDigital(stream, sp, ignore);
        if (c != 0) {
            //如果读到数字
            if (c == '-') {
                if (firstChar) {
                    //检测到一个'-'
                    isNeg     = true;
                    firstChar = false;
                    sp        = (sp + 1) % stream->rBuf.size;
                    continue;
                } else {
                    //'-'不是第一个数
                    break;
                }
            } else if (c == '+') {
                if (firstChar) {
                    //检测到一个'+'
                    firstChar = false;
                    sp        = (sp + 1) % stream->rBuf.size;
                    continue;
                } else {
                    //'+'不是第一个数
                    break;
                }
            } else if (c == '.') {
                //整数中不包括.
                break;
            } else if ((c == ignore) && (ignore != 0)) {
                if (firstChar) {
                    sp = (sp + 1) % stream->rBuf.size;
                    continue;
                } else {
                    //数据的中间出现忽略字符
                    break;
                }
            } else {
                n         = n * 10 + c - '0';
                sp        = (sp + 1) % stream->rBuf.size;
                firstChar = false;
                status    = true;
            }
        } else {
            //跳出循环
            break;
        }
    }
    if (status) {
        stream->rBuf.sp = sp;
        //有读取到数
        n = isNeg ? -n : n;
    }
    *num = n;
    return status;
}
static bool nextFloat(struct _ByteStream_t* stream, double* flo, u8 ignore) {
    double  f         = 0;
    double  frac      = 1.0;
    bool    status    = false;
    bool    isNeg     = false;
    bool    isFra     = false;
    bool    firstChar = true;
    int32_t sp        = stream->rBuf.sp;
    uint8_t c         = 0;
    while (sp != stream->rBuf.ep) {
        c = stream->ops.peekNextDigital(stream, sp, ignore);
        if (c != 0) {
            if (c == '-') {
                if (firstChar) {
                    //检测到一个'-'
                    isNeg = true;
                    sp    = (sp + 1) % stream->rBuf.size;
                    continue;
                } else {
                    //'-'不是第一个数
                    break;
                }
            } else if (c == '+') {
                if (firstChar) {
                    //检测到一个'+'
                    sp = (sp + 1) % stream->rBuf.size;
                    continue;
                } else {
                    //'-'不是第一个数
                    break;
                }
            } else if ((c == ignore) && (ignore != 0)) {
                if (firstChar) {
                    sp = (sp + 1) % stream->rBuf.size;
                    continue;
                } else {
                    //数据的中间出现忽略字符
                    break;
                }
            } else if (c == '.') {
                if (isFra) {  //不应出现两个'-'
                    break;
                } else {
                    if (!firstChar) {
                        isFra = true;
                        sp    = (sp + 1) % stream->rBuf.size;
                        continue;
                    } else {
                        //第一个字符为点的时候
                        break;
                    }
                }
            }
            if (isFra) {
                frac *= 0.1;
            }
            f         = f * 10 + c - '0';
            status    = true;
            firstChar = false;
            sp        = (sp + 1) % stream->rBuf.size;
        } else {
            break;
        }
    }
    if (status) {
        //有读取到数
        stream->rBuf.sp = sp;
        f               = isNeg ? -f : f;
        f               = isFra ? f * frac : f;
    }

    *flo = f;
    return status;
}
static void doSendedCallBack(struct _ByteStream_t* stream) {}
static void doReceivedCallBack(struct _ByteStream_t* stream) {}

static void baseOpsRegister(struct _ByteStream_t* stream) {
    stream->ops.baseOps.readOffset     = readOffset;
    stream->ops.baseOps.read           = read;
    stream->ops.baseOps.readAll        = readAll;
    stream->ops.baseOps.readByte       = readByte;
    stream->ops.baseOps.readStr        = readStr;
    stream->ops.baseOps.writeOffset    = writeOffset;
    stream->ops.baseOps.write          = write;
    stream->ops.baseOps.writeByte      = writeByte;
    stream->ops.baseOps.writeStr       = writeStr;
    stream->ops.baseOps.Printf         = Printf;
    stream->ops.baseOps.vPrintf        = vPrintf;
    stream->ops.baseOps.availableRead  = availableRead;
    stream->ops.baseOps.availableWrite = availableWrite;
    stream->ops.baseOps.discardRead    = discardRead;
    stream->ops.baseOps.discardWrite   = discardWrite;
    stream->ops.baseOps.isWriteIdle    = isWriteIdle;
    stream->ops.baseOps.waitWriteIdle  = waitWriteIdle;
}
static void opsRegister(struct _ByteStream_t* stream) {
    stream->ops.readLine           = readLine;
    stream->ops.peekWithSp         = peekWithSp;
    stream->ops.peek               = peek;
    stream->ops.peekNextDigital    = peekNextDigital;
    stream->ops.peekStrPos         = peekStrPos;
    stream->ops.touchOffset        = touchOffset;
    stream->ops.touch              = touch;
    stream->ops.touchByte          = touchByte;
    stream->ops.flushWrite         = flushWrite;
    stream->ops.buffededWrite      = buffededWrite;
    stream->ops.nextInt            = nextInt;
    stream->ops.nextFloat          = nextFloat;
    stream->ops.doSendedCallBack   = doSendedCallBack;
    stream->ops.doReceivedCallBack = doReceivedCallBack;
}
s8 byteStreamInit(struct _ByteStream_t* stream, uart_writeCb_t writeCb) {
    byteBufInit(&stream->rBuf);
    byteBufInit(&stream->tBuf);
    baseOpsRegister(stream);
    opsRegister(stream);
    stream->txBusy = false;
    _writeCb       = writeCb;
    return 0;
}
