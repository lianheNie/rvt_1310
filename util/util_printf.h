/*
 * util_printf.h
 *
 *  Created on: 2020Äê7ÔÂ31ÈÕ
 *      Author: admin
 */

#ifndef UTIL_PRINTF_H_
#define UTIL_PRINTF_H_

#include <stdint.h>
#include <stddef.h>
#include "util_def.h"
int8_t util_printf(const char* __restrict _format, ...);
int8_t at_printf(const char* __restrict _format, ...);
int8_t util_printf_init(AwCallBack_char_t writeStrCb);
#endif /* UTIL_PRINTF_H_ */
