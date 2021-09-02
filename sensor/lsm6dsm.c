/*
 * lsm6dsm.c
 *
 *  Created on: 2020年10月21日
 *      Author: admin
 */

#include "lsm6dsm.h"
#include "i2c_driver.h"
#include "util_delay.h"
#include "io_driver.h"
#include "aw_config.h"
#define _i2c_read   i2c_read
#define _i2c_write  i2c_write
#define _delay_ms   aw_delay_ms
#define _FIFO_DEPTH 2046

#ifdef IS_USE_LSM6DSM_SPI
#include "spi_driver.h"
#include <ti/drivers/PIN.h>
#include <ti/devices/cc13x0/driverlib/ioc.h>
#define IMU_ADS_CS IOID_9
static PIN_Handle _imuPinHandle;
static PIN_State  _imuPinState;
static PIN_Config _imuPinTable[] = {
    IMU_ADS_CS | PIN_GPIO_OUTPUT_EN | PIN_DRVSTR_MIN | PIN_PUSHPULL | PIN_GPIO_HIGH,
    PIN_TERMINATE /* Terminate list */
};
static void _spi_write(u8 reg, u8* buf, u8 cnt) {
    PIN_setOutputValue(_imuPinHandle, IMU_ADS_CS, 0);
    spi_write((reg & 0x7F), buf, cnt);
    PIN_setOutputValue(_imuPinHandle, IMU_ADS_CS, 1);
}
static void _spi_read(u8 reg, u8* buf, u8 cnt) {
    PIN_setOutputValue(_imuPinHandle, IMU_ADS_CS, 0);
    spi_read((reg | 0x80), buf, cnt);
    PIN_setOutputValue(_imuPinHandle, IMU_ADS_CS, 1);
}
s8 lsm6dsm_spi_open() {
    s8 res = 0;
    spi_open(SPI_POL1_PHA1);
    return res;
}
s8 lsm6dsm_spi_close() {
    s8 res = 0;
    spi_close();
    return res;
}
#endif
////////////////////////////////
static float _getAres(u8 Ascale);
static float _getGres(uint8_t Gscale);
static void  _reset();
static s8    _fifo_enable(u8 enable);
static s8    _fifo_init();
static s16   _acc_bias[3] = {0};
static float _aRes, _gRes;
s8           lsm6dsm_init() {
    u8 id  = 0;
    s8 res = 0;
#ifdef IS_USE_LSM6DSM_SPI
    _imuPinHandle = PIN_open(&_imuPinState, _imuPinTable);

    lsm6dsm_spi_open();
    _spi_read(LSM6DSM_WHO_AM_I, &id, 1);
    u8 val = 0;

    u8 data = AODR_6660Hz << 4 | AFS_4G << 2;
    _spi_write(LSM6DSM_CTRL1_XL, &data, 1);
    val = 0;
    _spi_read(LSM6DSM_CTRL1_XL, &val, 1);

    val = 0;
    _spi_read(LSM6DSM_CTRL3_C, &val, 1);
    val = val | 0x40 | 0x04;
    // enable block update (bit 6 = 1), auto-increment registers (bit 2 = 1)
    _spi_write(LSM6DSM_CTRL3_C, &val, 1);

    val = 0;
    _spi_read(LSM6DSM_CTRL3_C, &val, 1);

    val = 0x80 | 0x40 | 0x08;
    _spi_write(LSM6DSM_CTRL8_XL, &val, 1);
    val = 0;
    _spi_read(LSM6DSM_CTRL8_XL, &val, 1);
    lsm6dsm_calibration();
    _fifo_init();
    lsm6dsm_spi_close();

#else

    // u8 Ascale, Gscale, AODR, GODR;
    res = _i2c_read(LSM6DSM_ADDRESS, LSM6DSM_WHO_AM_I, &id, 1);

    u8 data = AODR_6660Hz << 4 | AFS_4G << 2;
    res     = _i2c_write(LSM6DSM_ADDRESS, LSM6DSM_CTRL1_XL, &data, 1);

    //    data = GODR << 4 | Gscale << 2;
    //    res  = _i2c_write(LSM6DSM_ADDRESS, LSM6DSM_CTRL2_G, &data, 1);
    uint8_t temp = 0;
    res          = _i2c_read(LSM6DSM_ADDRESS, LSM6DSM_CTRL3_C, &temp, 1);
    temp         = temp | 0x40 | 0x04;
    // enable block update (bit 6 = 1), auto-increment registers (bit 2 = 1)
    res = _i2c_write(LSM6DSM_ADDRESS, LSM6DSM_CTRL3_C, &temp, 1);
    // by default, interrupts active HIGH, push pull, little endian data
    // (can be changed by writing to bits 5, 4, and 1, resp to above register)

    // enable accel LP2 (bit 7 = 1), set LP2 tp ODR/9 (bit 6 = 1), enable input_composite (bit 3)
    // for low noise
    temp = 0x80 | 0x40 | 0x08;
    res  = _i2c_write(LSM6DSM_ADDRESS, LSM6DSM_CTRL8_XL, &temp, 1);
    lsm6dsm_calibration();
    _fifo_init();
#endif
    return res;
}

s8  lsm6dsm_test() { return 0; }
s16 lsm6dsm_read_accel(E_xyz_axis_t axis) {
#ifdef IS_USE_ACC_FIFO
    return lsm6dsm_fifo_reads(axis);
#else
    u8 rawData[2];  // x/y/z accel register data stored here
    u8 reg = LSM6DSM_OUTZ_L_XL;
    if (axis == AW_X_AXIS) {
        reg = LSM6DSM_OUTX_L_XL;
    } else if (axis == AW_Y_AXIS) {
        reg = LSM6DSM_OUTY_L_XL;
    } else if (axis == AW_Z_AXIS) {
        reg = LSM6DSM_OUTZ_L_XL;
    }
    s8  res = _i2c_read(LSM6DSM_ADDRESS, reg, &rawData[0], 2);
    s16 acc = ((int16_t)rawData[1] << 8) | rawData[0];
    acc     = acc - _acc_bias[axis];
    return acc;
#endif
}
s8 lsm6dsm_accel_enable(uint8_t enable) {
    u8 data = 0;
    if (1 == enable) {
        data = AODR_6660Hz << 4 | AFS_4G << 2;
    } else {
        data = AFS_4G << 2;
    }
    s8 res = 0;
#ifdef IS_USE_LSM6DSM_SPI
    _spi_write(LSM6DSM_CTRL1_XL, &data, 1);
#else
    res = _i2c_write(LSM6DSM_ADDRESS, LSM6DSM_CTRL1_XL, &data, 1);
#endif
    _delay_ms(500);
    _fifo_enable(enable);
    return res;
}
static float _getAres(uint8_t Ascale) {
    switch (Ascale) {
            // Possible accelerometer scales (and their register bit settings) are:
            // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
            // Here's a bit of an algorithm to calculate DPS/(ADC tick) based on that 2-bit value:
        case AFS_2G:
            _aRes = 2.0f / 32768.0f;
            return _aRes;
            // break;
        case AFS_4G:
            _aRes = 4.0f / 32768.0f;
            return _aRes;
            // break;
        case AFS_8G:
            _aRes = 8.0f / 32768.0f;
            return _aRes;
            // break;
        case AFS_16G:
            _aRes = 16.0f / 32768.0f;
            return _aRes;
            // break;
    }
}
static float _getGres(uint8_t Gscale) {
    switch (Gscale) {
            // Possible gyro scales (and their register bit settings) are:
            // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
        case GFS_245DPS:
            _gRes = 245.0f / 32768.0f;
            return _gRes;
            break;
        case GFS_500DPS:
            _gRes = 500.0f / 32768.0f;
            return _gRes;
            break;
        case GFS_1000DPS:
            _gRes = 1000.0f / 32768.0f;
            return _gRes;
            break;
        case GFS_2000DPS:
            _gRes = 2000.0f / 32768.0f;
            return _gRes;
            break;
    }
}
static void _reset() {
    // reset device
    uint8_t temp = 0;
    _i2c_read(LSM6DSM_ADDRESS, LSM6DSM_CTRL3_C, &temp, 1);
    temp = temp | 0x01;
    _i2c_write(LSM6DSM_ADDRESS, LSM6DSM_CTRL3_C, &temp, 1);  // Set bit 0 to 1 to reset LSM6DSM
    _delay_ms(100);                                          // Wait for all registers to reset
}

#define _LSM6DSM_CAL_SIZE 128
s16* lsm6dsm_calibration() {
    u8      ii     = 0;
    int32_t sum[3] = {0, 0, 0};
    lsm6dsm_accel_enable(1);
    _delay_ms(50);
    for (ii = 0; ii < _LSM6DSM_CAL_SIZE; ii++) {
        u8 rawData[6];  // x/y/z accel register data stored here
#ifdef IS_USE_LSM6DSM_SPI
        _spi_read(LSM6DSM_OUTX_L_XL, &rawData[0], 6);
#else
        s8 res = _i2c_read(LSM6DSM_ADDRESS, LSM6DSM_OUTX_L_XL, &rawData[0], 6);
#endif
        s16 acc_x = ((int16_t)rawData[1] << 8) | rawData[0];
        s16 acc_y = ((int16_t)rawData[3] << 8) | rawData[2];
        s16 acc_z = ((int16_t)rawData[5] << 8) | rawData[4];
        sum[0] += acc_x;
        sum[1] += acc_y;
        sum[2] += acc_z;
        _delay_ms(50);
    }
    _acc_bias[AW_X_AXIS] = sum[0] / _LSM6DSM_CAL_SIZE;
    _acc_bias[AW_Y_AXIS] = sum[1] / _LSM6DSM_CAL_SIZE;
    _acc_bias[AW_Z_AXIS] = sum[2] / _LSM6DSM_CAL_SIZE;
    lsm6dsm_accel_enable(0);
    return _acc_bias;
}
static s8 _fifo_init() {
    s8 res    = 0;
    u8 tem[2] = {0};

    tem[0] = 1;
#ifdef IS_USE_LSM6DSM_SPI                    // No decimation
    _spi_write(LSM6DSM_FIFO_CTRL3, tem, 1);  // positive accel self test
#else
    res = _i2c_write(LSM6DSM_ADDRESS, LSM6DSM_FIFO_CTRL3, tem, 1);   // positive accel self test
#endif
    //    tem[0] = 1 << 5;  // FIFO full flag interrupt enable on INT1 pad. // FIFO满中断
    //    res    = _i2c_write(LSM6DSM_ADDRESS, LSM6DSM_INT1_CTRL, tem, 1);  // positive accel self
    //    test
    return res;
}

static s8 _fifo_enable(u8 enable) {
    s8 res = 0;
    u8 tem = AODR_6660Hz << 3 | 1;
    if (0 == enable) {
        tem = 0;
    }
#ifdef IS_USE_LSM6DSM_SPI
    _spi_write(LSM6DSM_FIFO_CTRL5, &tem, 1);  // positive accel self test
#else
    res = _i2c_write(LSM6DSM_ADDRESS, LSM6DSM_FIFO_CTRL5, &tem, 1);  // positive accel self test
#endif
    return res;
}

u16 lsm6dsm_fifo_full() {
    u8 tem[2] = {0};
#ifdef IS_USE_LSM6DSM_SPI
    _spi_read(LSM6DSM_FIFO_STATUS1, tem, 2);  // positive accel self test
#else
    _i2c_read(LSM6DSM_ADDRESS, LSM6DSM_FIFO_STATUS1, tem, 2);        // positive accel self test
#endif
    u16 fifo = ((tem[1] & 0x07) << 8) | tem[0];
    return fifo;
}

s16 lsm6dsm_fifo_read() {
    u8 tem[2] = {0};
#ifdef IS_USE_LSM6DSM_SPI
    _spi_read(LSM6DSM_FIFO_DATA_OUT_L, tem, 2);  // positive accel self test
#else
    _i2c_read(LSM6DSM_ADDRESS, LSM6DSM_FIFO_DATA_OUT_L, tem, 2);     // positive accel self test
#endif
    s16 data = (tem[1] << 8) | tem[0];
    return data;
}

s16 lsm6dsm_fifo_reads(E_xyz_axis_t axis) {
    s16 acc[AW_AXIS_CNT] = {0};
    acc[AW_X_AXIS]       = lsm6dsm_fifo_read() - _acc_bias[AW_X_AXIS];
    acc[AW_Y_AXIS]       = lsm6dsm_fifo_read() - _acc_bias[AW_Y_AXIS];
    acc[AW_Z_AXIS]       = lsm6dsm_fifo_read() - _acc_bias[AW_Z_AXIS];
    return acc[axis];
}


