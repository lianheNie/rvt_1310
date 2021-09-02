/*
 * util_printf.c
 *
 *  Created on: 2020Äê7ÔÂ31ÈÕ
 *      Author: admin
 */
#include "util_printf.h"
#include <string.h>
#include <stdio.h>
#define IS_UART_DEBUG 0
#if IS_UART_DEBUG > 0
static char _strDebug[128] = {'\0'};

AwCallBack_char_t _writeStrCB = NULL;
#endif
int8_t at_printf(const char* __restrict _format, ...) {
#if IS_UART_DEBUG > 0
    if (_writeStrCB == NULL) {
        return -2;
    }
    memset(_strDebug, '\0', sizeof(_strDebug));
    va_list args;
    va_start(args, _format);
    vsprintf(_strDebug, _format, args);
    va_end(args);
    return _writeStrCB(_strDebug);
#else
    return 0;
#endif
}
int8_t util_printf(const char* __restrict _format, ...) { return 0; }
int8_t util_printf_init(AwCallBack_char_t writeStrCb) {
#if IS_UART_DEBUG > 0
    _writeStrCB = writeStrCb;
#endif
    return 0;
}
