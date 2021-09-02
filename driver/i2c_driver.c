/*
 * i2c_driver.c
 *
 *  Created on: 2020Äê7ÔÂ31ÈÕ
 *      Author: admin
 */
#include "i2c_driver.h"
#include <ti/drivers/I2C.h>
#include "Board.h"
static I2C_Handle _i2c = NULL;
static I2C_Params _i2cParams;
int8_t            i2c_init() {
    i2c_open();
    return 0;
}
int8_t i2c_close() {
    if (_i2c == NULL) {
        return -2;
    }
    I2C_close(_i2c);
    return 0;
}
s8 i2c_open() {
    I2C_init();
    I2C_Params_init(&_i2cParams);
    _i2cParams.bitRate = I2C_400kHz;
    _i2c               = I2C_open(Board_I2C_TMP, &_i2cParams);
    if (_i2c == NULL) {
        return -1;
    }
    return 0;
}
static int8_t _i2c_read(I2C_Handle i2cHandle, uint8_t addr, uint8_t *reg, uint8_t regCnt,
                        uint8_t *dataBuf, uint16_t dataCnt) {
    if (i2cHandle == NULL) {
        return -2;
    }
    I2C_Transaction i2cTransaction;
    i2cTransaction.writeBuf     = reg;
    i2cTransaction.writeCount   = regCnt;
    i2cTransaction.readBuf      = dataBuf;
    i2cTransaction.readCount    = dataCnt;
    i2cTransaction.slaveAddress = addr;
    if (!I2C_transfer(i2cHandle, &i2cTransaction)) {
        return -1;
    }
    return 0;
}
static int8_t _i2c_burst_Read(I2C_Handle i2cHandle, uint8_t addr, uint8_t *reg, uint8_t regCnt,
                              uint8_t *dataBuf, uint32_t dataCnt) {
    if (i2cHandle == NULL) {
        return -2;
    }
    I2C_Transaction i2cTransaction;
    i2cTransaction.writeBuf     = reg;
    i2cTransaction.writeCount   = regCnt;
    i2cTransaction.readBuf      = dataBuf;
    i2cTransaction.readCount    = dataCnt;
    i2cTransaction.slaveAddress = addr;
    if (!I2C_transfer(i2cHandle, &i2cTransaction)) {
        return -1;
    }
    return 0;
}
static int8_t _i2c_write(I2C_Handle i2cHandle, uint8_t addr, uint8_t *reg, uint8_t regCnt,
                         const uint8_t *dataBuf, uint16_t dataCnt) {
    if (i2cHandle == NULL) {
        return -2;
    }
    I2C_Transaction i2cTransaction;
    uint8_t         wirteBuf[10] = {0};
    uint8_t         i            = 0;
    for (i = 0; i < regCnt; i++) {
        wirteBuf[i] = reg[i];
    }
    for (i = 0; i < dataCnt; i++) {
        wirteBuf[i + regCnt] = dataBuf[i];
    }
    i2cTransaction.writeBuf     = wirteBuf;
    i2cTransaction.writeCount   = regCnt + dataCnt;
    i2cTransaction.readBuf      = NULL;
    i2cTransaction.readCount    = 0;
    i2cTransaction.slaveAddress = addr;
    if (!I2C_transfer(i2cHandle, &i2cTransaction)) {
        return -1;
    }
    return 0;
}
int8_t i2c_readByCmd(uint8_t addr, uint8_t *cmd, uint8_t cmdCnt, uint8_t *dataBuf,
                     uint16_t dataCnt) {
    return _i2c_read(_i2c, addr, cmd, cmdCnt, dataBuf, dataCnt);
}
int8_t i2c_writeByCmd(uint8_t addr, uint8_t *cmd, uint8_t cmdCnt, uint8_t *dataBuf,
                      uint16_t dataCnt) {
    return _i2c_write(_i2c, addr, cmd, cmdCnt, dataBuf, dataCnt);
}

int8_t i2c_read(uint8_t addr, uint8_t reg, uint8_t *dataBuf, uint16_t dataCnt) {
    return _i2c_read(_i2c, addr, &reg, 1, dataBuf, dataCnt);
}
int8_t i2c_write(uint8_t addr, uint8_t reg, uint8_t *dataBuf, uint16_t dataCnt) {
    return _i2c_write(_i2c, addr, &reg, 1, dataBuf, dataCnt);
}
int8_t i2c_burst_read(uint8_t addr, uint8_t reg, uint8_t *dataBuf, uint32_t dataCnt) {
    return _i2c_burst_Read(_i2c, addr, &reg, 1, dataBuf, dataCnt);
}
