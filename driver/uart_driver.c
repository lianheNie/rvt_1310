/*
 * uart_driver.c
 *
 *  Created on: 2020Äê7ÔÂ31ÈÕ
 *      Author: admin
 */
#include "uart_driver.h"
#include "Board.h"
#include "aw_config.h"
#include <ti/drivers/PIN.h>
#include "util_delay.h"
static UartReadCallBack_t _uartReadCb = NULL;
static int                _uartBaud   = 9600;
static UART_Params        _uartParams;
static UART_Handle        _hUart = NULL;

#define _AW_RX_PIN_ID IOID_2  // IOID_1
#define _AW_TX_PIN_ID IOID_3  // IOID_1
static PIN_Handle _ioPinHandle = NULL;
static PIN_State  _ioPinState;
#define AW_PIR_VAL 1
static PIN_Config _ioPinTable[] = {
    _AW_RX_PIN_ID | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    _AW_TX_PIN_ID | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE};

static void uartReadCallback(UART_Handle _handle, void *_buf, size_t _size);
s8          uart_init(int baud, UartReadCallBack_t uartReadCb) {
    _uartBaud   = baud;
    _uartReadCb = uartReadCb;
    UART_init();
    UART_Params_init(&_uartParams);
    _uartParams.baudRate     = _uartBaud;
    _uartParams.readMode     = UART_MODE_CALLBACK;
    _uartParams.readDataMode = UART_DATA_BINARY;
    _uartParams.readCallback = uartReadCallback;
    _hUart                   = UART_open(Board_UART0, &_uartParams);
    return 0;
}
s8 uart_close() {
    if (_hUart == NULL) {
        return -2;
    }
    UART_close(_hUart);
    _hUart = NULL;
    aw_delay_ms(1);
    _ioPinHandle = PIN_open(&_ioPinState, _ioPinTable);
    return 0;
}
s8 uart_open() {
    PIN_close(_ioPinHandle);
    aw_delay_ms(1);
    _hUart = UART_open(Board_UART0, &_uartParams);
    if (_hUart == NULL) {
        return -2;
    }

    aw_delay_ms(1);
    return 0;
}
s8          uart_is_open() { return (_hUart != NULL); }
static void uartReadCallback(UART_Handle _handle, void *_buf, size_t _size) {
    if (_uartReadCb != NULL) {
        _uartReadCb(_buf, _size);
    }
}
s8 uart_write(const void *buffer, size_t size) {
    if (_hUart == NULL) {
        return -2;
    }
    UART_write(_hUart, buffer, size);
    return 0;
}
s8 uart_write_str(const char *str) {
    if (_hUart == NULL) {
        return -2;
    }
    UART_write(_hUart, str, strlen(str));  // CC1310
    return 0;
}
s8 uart_read(void *buffer, size_t size) {
    if (_hUart == NULL) {
        return -2;
    }
    UART_read(_hUart, buffer, size);
    return 0;
}

#ifdef IS_USE_RV
#define _UART_RX_BUF_SIZE 1200
#else
#define _UART_RX_BUF_SIZE 172
#endif

static char _uart_tx_buf[_UART_RX_BUF_SIZE] = {'\0'};

s8 uart_printf(const char *__restrict _format, ...) {
    _uart_tx_buf[0] = '\0';
    va_list args;
    va_start(args, _format);
    vsnprintf(_uart_tx_buf, _UART_RX_BUF_SIZE, _format, args);
    va_end(args);
    s8 res          = uart_write_str(_uart_tx_buf);
    _uart_tx_buf[0] = '\0';
    return res;
}

char *uart_vsprintf(const char *format, va_list args) {
    _uart_tx_buf[0] = '\0';
    vsnprintf(_uart_tx_buf, _UART_RX_BUF_SIZE, format, args);
    return _uart_tx_buf;
}

s8 uart_printf_add(char **buf, const char *__restrict _format, ...) {
    int     res = 0;
    int     len = strlen(_uart_tx_buf);
    va_list args;
    va_start(args, _format);
    int cnt = vsnprintf(&_uart_tx_buf[len], _UART_RX_BUF_SIZE - len, _format, args);
    va_end(args);
    if ((cnt + len) > _UART_RX_BUF_SIZE) {
        res = -1;
    }
    if (0 == res) {
        *buf = _uart_tx_buf;
    } else {
        *buf = NULL;
    }
    return res;
}
s8 uart_flush() {
    s8 res          = uart_write_str(_uart_tx_buf);
    _uart_tx_buf[0] = '\0';
    return res;
}
UART_Handle uart_handle() { return _hUart; }
