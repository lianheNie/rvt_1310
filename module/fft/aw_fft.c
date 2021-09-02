/*
 * aw_fft.c
 *
 *  Created on: 2020Äê10ÔÂ19ÈÕ
 *      Author: admin
 */

#include "aw_fft.h"
#include "bmi160.h"
#include "lsm6dsm.h"
#include "util_delay.h"
#include <ti/drivers/timer/GPTimerCC26XX.h>
#include "aw_config.h"
#ifdef IS_USE_MQTT
#include <aw_mqtt.h>
#endif
#include "i2c_driver.h"
#define numSamples LSM6DSM_FIFO_DEPTH / 3  // 1024
static int16_t              _zArray[numSamples];
static bmi160_sensor_data_t _acc_data;
// msp_fft_q15_params params = {numSamples, true, msp_cmplx_bitrev_table_2048_ui16,
//                             msp_cmplx_twiddle_table_2048_q15, msp_split_table_2048_q15};

void aw_fft_test() {
    // msp_fft_q15(&params, _zArray);
}
void aw_fft_init() {
#ifdef IS_USE_LSM6DSM
    lsm6dsm_init();
#else
    bmi160_init_no_imu();
#endif
    //
}
#ifndef IS_USE_ACC_FIFO
static GPTimerCC26XX_Handle _hTimer                = NULL;
static u16                  _data_cnt              = 0;
static u8                   _timerCallbackReached  = 0;
static u8                   _samplesStartupCounter = 0;
static void _timerCallback(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask) {
    if (_samplesStartupCounter < 15) {
        _samplesStartupCounter++;
    } else {
        _data_cnt++;
    }
    _timerCallbackReached = 1;
    if (_data_cnt >= numSamples) {
        _samplesStartupCounter = 0;
        GPTimerCC26XX_stop(_hTimer);
        GPTimerCC26XX_close(_hTimer);
    }
}
#endif
#define _AW_FFT_PEROID_US 500  // 200us 2KHz
#define _AW_CPU_FREQ_MHZ  48
#ifdef IS_USE_BMI160
static bmi160_sensor_data_t _acc_data;
#endif
s16* aw_fft_calibration() {
#ifdef IS_USE_LSM6DSM
    lsm6dsm_spi_open();
    s16* vla = lsm6dsm_calibration();
    lsm6dsm_spi_close();
    return vla;
#else
    return NULL;
#endif
}
void aw_fft_process() {
#ifndef IS_USE_ACC_FIFO
    GPTimerCC26XX_Params timerParams;
    GPTimerCC26XX_Params_init(&timerParams);
    timerParams.width           = GPT_CONFIG_16BIT;
    timerParams.mode            = GPT_MODE_PERIODIC_UP;
    timerParams.debugStallMode  = GPTimerCC26XX_DEBUG_STALL_OFF;
    _hTimer                     = GPTimerCC26XX_open(0, &timerParams);
    GPTimerCC26XX_Value loadVal = _AW_CPU_FREQ_MHZ * _AW_FFT_PEROID_US;
    GPTimerCC26XX_setLoadValue(_hTimer, loadVal);
    GPTimerCC26XX_registerInterrupt(_hTimer, _timerCallback, GPT_INT_TIMEOUT);
#else

#endif
    lsm6dsm_spi_open();
#ifdef IS_USE_LSM6DSM
    lsm6dsm_accel_enable(1);
#else
    bmi160_accel_enable(1);
#endif

    aw_delay_ms(1000);
#ifndef IS_USE_ACC_FIFO
    GPTimerCC26XX_start(_hTimer);
    while (_data_cnt < numSamples)  // Wait for timer interrupt for each sample
    {
        // aw_delay_us(1);
        if (_timerCallbackReached) {
#ifdef IS_USE_LSM6DSM
            s16 acc = lsm6dsm_read_accel(AW_Z_AXIS);
#else
            bmi160_read_accel(&_acc_data);
#endif

            if (_data_cnt < numSamples) {
#ifdef IS_USE_LSM6DSM
                _zArray[_data_cnt] = acc;
#else
                _zArray[_data_cnt] = _acc_data.z;
#endif
            }
            _timerCallbackReached = 0;
        }
    }
#else
    while (lsm6dsm_fifo_full() < LSM6DSM_FIFO_DEPTH) {
        aw_delay_ms(200);
    }
    u16 fifo_data_cnt = lsm6dsm_fifo_full();
    if (fifo_data_cnt >= LSM6DSM_FIFO_DEPTH) {
        u16 i = 0;
        for (i = 0; i < numSamples; i++) {
            s16 acc_z  = lsm6dsm_read_accel(AW_Y_AXIS);
            _zArray[i] = acc_z;
        }
    }

#endif
#ifdef IS_USE_LSM6DSM
    lsm6dsm_accel_enable(0);
#else
    bmi160_accel_enable(0);
#endif

    aw_delay_us(10);
    lsm6dsm_spi_close();
#ifndef IS_USE_ACC_FIFO
    _data_cnt = 0;
#endif

#ifdef IS_USE_MQTT
    u16 i      = 0;
    u8  j      = 0;
    u16 offset = 0;
    u16 off    = 200;
    for (j = 0; j < 3; j++) {
        aw_mqtt_pub_add_f("{v:[");
        for (i = 0; i < off; i++) {
            if (i > 0) {
                aw_mqtt_pub_add_f(",");
            }
            aw_mqtt_pub_add_f("%hX", _zArray[i + offset]);
        }
        aw_mqtt_pub_add_f("],i:%d,c:%d}", offset, off);
        aw_mqtt_pub_flush(AW_PUB_MODAL, QOS0_MOST_ONECE, NO_RETAIN,
                          aw_mqtt_topic_get_f("" AW_DEV_RV "/%s/" AW_NOTIFY "", AW_MQTT_DEV_ID));
        offset = offset + off;
    }
    aw_mqtt_pub_add_f("{v:[");
    for (i = offset; i < numSamples; i++) {
        if (i > offset) {
            aw_mqtt_pub_add_f(",");
        }
        aw_mqtt_pub_add_f("%hX", _zArray[i]);
    }
    aw_mqtt_pub_add_f("],i:%d,c:%d}", offset, (numSamples - offset));
    aw_mqtt_pub_flush(AW_PUB_MODAL, QOS0_MOST_ONECE, NO_RETAIN,
                      aw_mqtt_topic_get_f("" AW_DEV_RV "/%s/" AW_NOTIFY "", AW_MQTT_DEV_ID));
    u16 cnt = 0;
    for (cnt = 0; cnt < numSamples; cnt++) {
        _zArray[cnt] = 0;
    }
#endif
}
