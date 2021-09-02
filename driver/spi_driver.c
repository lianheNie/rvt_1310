/*
 * spi_driver.c
 *
 *  Created on: 2020Äê10ÔÂ3ÈÕ
 *      Author: admin
 */

#include "spi_driver.h"

#include "board.h"
static SPI_Handle _spi = NULL;
static SPI_Params _spiParams;
s8                spi_open(SPI_FrameFormat frameFormat) {
    s8 res                 = 0;
    _spiParams.frameFormat = frameFormat;  // SPI_POL0_PHA1;
    _spi                   = SPI_open(Board_SPI_MASTER, &_spiParams);
    if (_spi == NULL) {
        return -1;
    }
    return res;
}

s8 spi_close() {
    s8 res = 0;
    SPI_close(_spi);
    _spi = NULL;
    return res;
}
s8 spi_init() {
    SPI_init();
    SPI_Params_init(&_spiParams);
    return 0;
}

s8 spi_read(u8 reg, u8 *buf, u8 cnt) {
    SPI_Transaction transaction;
    u8              txbuf[10] = {0x00};
    u8              rxBuf[10] = {0};
    txbuf[0]                  = reg;
    transaction.count         = cnt + 1;
    transaction.txBuf         = (void *)txbuf;
    transaction.rxBuf         = (void *)rxBuf;
    bool transferOK           = SPI_transfer(_spi, &transaction);
    if (transferOK) {
        u8 i = 0;
        for (i = 0; i < cnt; i++) {
            buf[i] = rxBuf[i + 1];
        }
        return 0;
    } else {
        return -1;
    }
}
s8 spi_write(u8 reg, u8 *buf, u8 cnt) {
    SPI_Transaction transaction;
    u8              rxBuf[10] = {0};
    u8              txbuf[10] = {0};
    u8              _cnt      = cnt;
    txbuf[0]                  = reg;
    if (buf != NULL) {
        _cnt = _cnt + 1;
        u8 i = 0;
        for (i = 0; i < cnt; i++) {
            txbuf[i + 1] = buf[i];
        }
    }
    transaction.count = _cnt;
    transaction.txBuf = (void *)txbuf;
    transaction.rxBuf = (void *)rxBuf;
    bool transferOK   = SPI_transfer(_spi, &transaction);
    if (transferOK) {
        return 0;
    } else {
        return -1;
    }
}
s8 spi_write_cmd(u8 cmd) { return spi_write(cmd, NULL, 1); }
