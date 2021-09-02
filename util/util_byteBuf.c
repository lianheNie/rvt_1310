/*
 * util_byteStream.c
 *
 *  Created on: 2020年9月8日
 *      Author: admin
 */

#include <util_byteBuf.h>
static s32 getUsed(ByteBuf_t* buf) {
    //存档，为了中断安全
    int32_t l_sp = buf->sp, l_ep = buf->ep;
    return (l_sp <= l_ep) ? (l_ep - l_sp) : (buf->size - l_sp + l_ep);
}
static s32 getUsedWithSp(ByteBuf_t* buf, s32 tgtSp) {
    //存档，为了中断安全
    int32_t l_sp = tgtSp, l_ep = buf->ep;
    return (l_sp <= l_ep) ? (l_ep - l_sp) : (buf->size - l_sp + l_ep);
}
static s32 getUsedWithEp(ByteBuf_t* buf, s32 tgtEp) {
    //存档，为了中断安全
    int32_t l_sp = buf->sp, l_ep = tgtEp;
    return (l_sp <= l_ep) ? (l_ep - l_sp) : (buf->size - l_sp + l_ep);
}
static s32 getFree(ByteBuf_t* buf) {
    int32_t l_sp = buf->sp, l_ep = buf->ep;
    return (l_sp <= l_ep) ? (buf->size - (l_ep - l_sp) - 1) : (l_sp - l_ep - 1);
}

static void reset(ByteBuf_t* buf) {
    buf->sp   = 0;
    buf->ep   = 0;
    buf->size = BYTE_BUF_SIZE;
}
static bool push(ByteBuf_t* buf, Byte_t byte, bool force) {
    int32_t nextEp = (buf->ep + 1) % buf->size;
    if (nextEp == buf->sp) {
        if (force) {
            //强制推入，偏移sp指针
            //需要注意，这里的访问是有风险的。sp可能被同时访问
            buf->sp = (buf->sp + 1) % buf->size;
        } else {
            return false;
        }
    } else {
        buf->buf[buf->ep] = byte;
        buf->ep           = nextEp;
    }
    return true;
}
static Byte_t pop(ByteBuf_t* buf) {
    Byte_t t;
    if (buf->sp != buf->ep) {
        t       = buf->buf[buf->sp];
        buf->sp = (buf->sp + 1) % buf->size;
    }
    return t;
}
s8 byteBufInit(ByteBuf_t* buf) {
    buf->ops.getUsed       = getUsed;
    buf->ops.getUsedWithSp = getUsedWithSp;
    buf->ops.getUsedWithEp = getUsedWithEp;
    buf->ops.getFree       = getFree;
    buf->ops.reset         = reset;
    buf->ops.push          = push;
    buf->ops.pop           = pop;
    buf->ops.reset(buf);
    return 0;
}
