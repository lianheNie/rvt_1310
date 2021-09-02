/*
 * uart_driver.h
 *
 *  Created on: 2020Äê7ÔÂ31ÈÕ
 *      Author: admin
 */

#ifndef UART_DRIVER_H_
#define UART_DRIVER_H_
#include <ti/drivers/UART.h>
#include "driver_comm.h"
#include <string.h>
#include <stdio.h>
typedef void (*UartReadCallBack_t)(void *_buf, size_t _size);
s8          uart_init(int baud, UartReadCallBack_t uartReadCb);
s8          uart_write(const void *buffer, size_t size);
s8          uart_write_str(const char *str);
s8          uart_read(void *buffer, size_t size);
void        uart_read_start();
s8          uart_close();
s8          uart_open();
s8          uart_is_open();
UART_Handle uart_handle();
s8          uart_printf(const char *__restrict _format, ...);
s8          uart_printf_add(char **buf, const char *__restrict _format, ...);
s8          uart_flush();
char *      uart_vsprintf(const char *format, va_list args);
#endif /* UART_DRIVER_H_ */
