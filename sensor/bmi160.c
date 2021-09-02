/*
 * bmi160.c
 *
 *  Created on: 2020Äê7ÔÂ31ÈÕ
 *      Author: admin
 */
#include "bmi160.h"
#include "i2c_driver.h"
#include "util_printf.h"
#include "util_delay.h"
#include "util_def.h"
#include "io_driver.h"
#include "Board.h"

static bmi160_dev_t _dev_bmi160;
static imu_data_t   _imu_data;
static int8_t       bmi160_i2c_routine(bmi160_dev_t *dev_bmi160);
static int8_t       bmi160_imu_init(bmi160_dev_t *dev_bmi160);
// static int8_t       bmi160_imu_normal_init(bmi160_dev_t *dev_bmi160);
static int8_t bmi160_imu_lowpower_init(bmi160_dev_t *dev_bmi160);
static int8_t bmi160_readDatas(bmi160_dev_t *dev_bmi160, imu_data_t *imu_data);
static int8_t bmi160_any_motion_int_init(bmi160_dev_t *dev_bmi160);

static PIN_Handle _int1PinHandle = NULL;
static PIN_State  _int1PinState;
#define INT1_PIN IOID_0  // IOID_22 IOID_0
static PIN_Config      _int1PinTable[] = {INT1_PIN | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
                                     PIN_TERMINATE};
static void            int1CallbackFxn(PIN_Handle handle, PIN_Id pinId);
static AwCallBack_u8_t _int1CB = NULL;
static int8_t          int1Pin_init(AwCallBack_u8_t int1Cb);

int8_t bmi160_init(AwCallBack_u8_t int1Cb) {
    int8_t rslt = AW_ERR;
    rslt        = bmi160_i2c_routine(&_dev_bmi160);
    rslt        = bmi160_driver_init(&_dev_bmi160);
    if (rslt == AW_OK) {
        // util_printf("BMI160 chip id=0x%0x\r\n", _dev_bmi160.chip_id);
    } else {
        // util_printf("BMI160 communication err\r\n");
    }
    rslt = bmi160_imu_init(&_dev_bmi160);
    if (rslt == AW_OK) {
        // util_printf("BMI160 low power mode init ok\r\n");
    } else {
        // util_printf("BMI160 low power mode err\r\n");
    }
    rslt = bmi160_any_motion_int_init(&_dev_bmi160);
    if (rslt == AW_OK) {
        // util_printf("BMI160 any-motion int init ok\r\n");
    } else {
        // util_printf("BMI160 any-motion int init err\r\n");
    }
    rslt = bmi160_readDatas(&_dev_bmi160, &_imu_data);
    if (rslt == AW_OK) {
        // util_printf("BMI160 read imu data: accX=%d,accY=%d,accZ=%d,gyrX=%d,gyrY=%d,gyrZ=%d,\r\n",
        //           _imu_data.accel.x, _imu_data.accel.y, _imu_data.accel.z, _imu_data.gyro.x,
        //           _imu_data.gyro.y, _imu_data.gyro.z);
    }
    rslt = int1Pin_init(int1Cb);
    if (rslt == AW_OK) {
        // util_printf("BMI160 int 1 init ok\r\n");
    } else {
        // util_printf("BMI160 int 1 init err\r\n");
    }
    return rslt;
}
static s8 _bmi160_acc_init() {
    int8_t rslt                 = BMI160_OK;
    _dev_bmi160.accel_cfg.odr   = BMI160_ACCEL_ODR_1600HZ;
    _dev_bmi160.accel_cfg.range = BMI160_ACCEL_RANGE_4G;
    _dev_bmi160.accel_cfg.bw    = BMI160_ACCEL_BW_NORMAL_AVG4;
    _dev_bmi160.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
    rslt                        = bmi160_set_sens_conf(&_dev_bmi160);
    return rslt;
}
int8_t bmi160_init_no_imu() {
    int8_t rslt = AW_ERR;
    rslt        = bmi160_i2c_routine(&_dev_bmi160);
    rslt        = bmi160_driver_init(&_dev_bmi160);
    rslt        = _bmi160_acc_init();
    rslt        = bmi160_accel_enable(0);
    rslt        = bmi160_gyro_enable(0);

    return rslt;
}

static int8_t bmi160_i2c_routine(bmi160_dev_t *dev_bmi160) {
    AW_S8_VAULE rslt      = AW_ERR;
    dev_bmi160->id        = BMI160_I2C_ADDR_VDD;
    dev_bmi160->interface = BMI160_I2C_INTF;
    dev_bmi160->write     = i2c_write;
    dev_bmi160->read      = i2c_read;
    dev_bmi160->delay_ms  = aw_delay_ms;
    return rslt;
}
static int8_t bmi160_imu_init(bmi160_dev_t *dev_bmi160) {
    AW_S8_VAULE rslt = AW_ERR;
    // rslt             = bmi160_imu_normal_init(dev_bmi160);
    rslt = bmi160_imu_lowpower_init(dev_bmi160);
    return rslt;
}

// int8_t bmi160_imu_normal_init(bmi160_dev_t *dev_bmi160) {
//    int8_t rslt = BMI160_OK;
//    /* Select the Output data rate, range of accelerometer sensor */
//    dev_bmi160->accel_cfg.odr   = BMI160_ACCEL_ODR_1600HZ;
//    dev_bmi160->accel_cfg.range = BMI160_ACCEL_RANGE_2G;
//    dev_bmi160->accel_cfg.bw    = BMI160_ACCEL_BW_NORMAL_AVG4;
//
//    /* Select the power mode of accelerometer sensor */
//    dev_bmi160->accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
//
//    /* Select the Output data rate, range of Gyroscope sensor */
//    dev_bmi160->gyro_cfg.odr   = BMI160_GYRO_ODR_3200HZ;
//    dev_bmi160->gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
//    dev_bmi160->gyro_cfg.bw    = BMI160_GYRO_BW_NORMAL_MODE;
//
//    /* Select the power mode of Gyroscope sensor */
//    dev_bmi160->gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;
//
//    /* Set the sensor configuration */
//    rslt = bmi160_set_sens_conf(dev_bmi160);
//    return rslt;
//}

static int8_t bmi160_imu_lowpower_init(bmi160_dev_t *dev_bmi160) {
    int8_t rslt = BMI160_OK;
    /* Select the Output data rate, range of accelerometer sensor */
    dev_bmi160->accel_cfg.odr   = BMI160_ACCEL_ODR_6_25HZ;
    dev_bmi160->accel_cfg.range = BMI160_ACCEL_RANGE_2G;
    dev_bmi160->accel_cfg.bw    = BMI160_ACCEL_BW_NORMAL_AVG4;  // BMI160_ACCEL_BW_RES_AVG16;

    /* Select the power mode of accelerometer sensor */

#ifdef IS_USE_NB_POWERDOWN
    dev_bmi160->accel_cfg.power = BMI160_ACCEL_SUSPEND_MODE;
#else
    dev_bmi160->accel_cfg.power = BMI160_ACCEL_LOWPOWER_MODE;
#endif

    /* Select the Output data rate, range of Gyroscope sensor */
    dev_bmi160->gyro_cfg.odr   = BMI160_GYRO_ODR_100HZ;
    dev_bmi160->gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    dev_bmi160->gyro_cfg.bw    = BMI160_GYRO_BW_NORMAL_MODE;

    /* Select the power mode of Gyroscope sensor */
    dev_bmi160->gyro_cfg.power = BMI160_GYRO_SUSPEND_MODE;

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(dev_bmi160);
    return rslt;
}
int8_t bmi160_gyro_enable(uint8_t enable) {
    int8_t rslt = BMI160_OK;
    if (enable == 1) {
        _dev_bmi160.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;
    } else if (enable == 0) {
        _dev_bmi160.gyro_cfg.power = BMI160_GYRO_SUSPEND_MODE;
    }
    /*  Set the Power mode  */
    rslt = bmi160_set_power_mode(&_dev_bmi160);
    return rslt;
}
int8_t bmi160_accel_enable(uint8_t enable) {
    int8_t rslt = BMI160_OK;
    if (enable == 1) {
        _dev_bmi160.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
    } else if (enable == 0) {
        _dev_bmi160.accel_cfg.power = BMI160_ACCEL_SUSPEND_MODE;
    }
    /*  Set the Power mode  */
    rslt = bmi160_set_power_mode(&_dev_bmi160);
    return rslt;
}
static int8_t bmi160_readDatas(bmi160_dev_t *dev_bmi160, imu_data_t *imu_data) {
    int8_t rslt = BMI160_OK;
    /* To read only Accel data */
    rslt = bmi160_get_sensor_data(BMI160_ACCEL_SEL, &imu_data->accel, NULL, dev_bmi160);

    /* To read only Gyro data */
    rslt = bmi160_get_sensor_data(BMI160_GYRO_SEL, NULL, &imu_data->gyro, dev_bmi160);

    /* To read both Accel and Gyro data */
    bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &imu_data->accel, &imu_data->gyro,
                           dev_bmi160);

    /* To read Accel data along with time */
    rslt = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_TIME_SEL), &imu_data->accel, NULL,
                                  dev_bmi160);

    /* To read Gyro data along with time */
    rslt = bmi160_get_sensor_data((BMI160_GYRO_SEL | BMI160_TIME_SEL), NULL, &imu_data->gyro,
                                  dev_bmi160);

    /* To read both Accel and Gyro data along with time*/
    bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL), &imu_data->accel,
                           &imu_data->gyro, dev_bmi160);
    return rslt;
}

int8_t bmi160_read_accel(bmi160_sensor_data_t *accel) {
    int8_t rslt = BMI160_OK;
    rslt        = bmi160_get_sensor_data(BMI160_ACCEL_SEL, accel, NULL, &_dev_bmi160);
    return rslt;
}

int8_t bmi160_read_gyro(bmi160_sensor_data_t *gyro) {
    int8_t rslt = BMI160_OK;
    rslt        = bmi160_get_sensor_data(BMI160_GYRO_SEL, NULL, gyro, &_dev_bmi160);
    return rslt;
}

static struct bmi160_acc_any_mot_int_cfg _acc_any_motion_int;
static int8_t                            bmi160_any_motion_int_init(bmi160_dev_t *dev_bmi160) {
    int8_t                  rslt = BMI160_OK;
    struct bmi160_int_settg int_config;

    /* Select the Interrupt channel/pin */
    int_config.int_channel = BMI160_INT_CHANNEL_1;  // Interrupt channel/pin 1

    /* Select the Interrupt type */
    int_config.int_type = BMI160_ACC_ANY_MOTION_INT;  // Choosing Any motion interrupt
    /* Select the interrupt channel/pin settings */
    int_config.int_pin_settg.output_en =
        BMI160_ENABLE;  // Enabling interrupt pins to act as output pin
    int_config.int_pin_settg.output_mode =
        BMI160_DISABLE;  // Choosing push-pull mode for interrupt pin
    int_config.int_pin_settg.output_type = BMI160_DISABLE;  // Choosing active low output
    int_config.int_pin_settg.edge_ctrl = BMI160_ENABLE;  // Choosing edge triggered output
    int_config.int_pin_settg.input_en = BMI160_DISABLE;  // Disabling interrupt pin to act as input
    int_config.int_pin_settg.latch_dur = BMI160_LATCH_DUR_NONE;  // non-latched output

    _acc_any_motion_int.anymotion_en =
        BMI160_ENABLE;  // 1- Enable the any-motion, 0- disable any-motion
    _acc_any_motion_int.anymotion_x = BMI160_ENABLE;  // Enabling x-axis for any motion interrupt
    _acc_any_motion_int.anymotion_y = BMI160_ENABLE;  // Enabling y-axis for any motion interrupt
    _acc_any_motion_int.anymotion_z = BMI160_ENABLE;  // Enabling z-axis for any motion interrupt
    _acc_any_motion_int.anymotion_dur = 0;  // any-motion duration
    _acc_any_motion_int.anymotion_thr =
        20;  // (2-g range) -> (slope_thr) * 3.91 mg, (4-g range) -> (slope_thr) * 7.81 mg, (8-g
             // range) ->(slope_thr) * 15.63 mg, (16-g range) -> (slope_thr) * 31.25 mg
    int_config.int_type_cfg.acc_any_motion_int = _acc_any_motion_int;
    /* Set the Any-motion interrupt */
    bmi160_set_int_config(&int_config,
                          dev_bmi160); /* sensor is an instance of the structure bmi160_dev  */
    return rslt;
}

int8_t bmi160_acc_any_motion_int_enable(uint8_t enable) {
    int8_t rslt = BMI160_OK;
    if (enable == 1) {
        _acc_any_motion_int.anymotion_en =
            BMI160_ENABLE;  // 1- Enable the any-motion, 0- disable any-motion
        _acc_any_motion_int.anymotion_x =
            BMI160_ENABLE;  // Enabling x-axis for any motion interrupt
        _acc_any_motion_int.anymotion_y =
            BMI160_ENABLE;  // Enabling y-axis for any motion interrupt
        _acc_any_motion_int.anymotion_z =
            BMI160_ENABLE;                      // Enabling z-axis for any motion interrupt
        _acc_any_motion_int.anymotion_dur = 0;  // any-motion duration
        _acc_any_motion_int.anymotion_thr =
            20;  // (2-g range) -> (slope_thr) * 3.91 mg, (4-g range) -> (slope_thr) * 7.81 mg, (8-g
                 // range) ->(slope_thr) * 15.63 mg, (16-g range) -> (slope_thr) * 31.25 mg
    } else if (enable == 0) {
        _acc_any_motion_int.anymotion_en =
            BMI160_DISABLE;  // 1- Enable the any-motion, 0- disable any-motion
        _acc_any_motion_int.anymotion_x =
            BMI160_DISABLE;  // Enabling x-axis for any motion interrupt
        _acc_any_motion_int.anymotion_y =
            BMI160_DISABLE;  // Enabling y-axis for any motion interrupt
        _acc_any_motion_int.anymotion_z =
            BMI160_DISABLE;                     // Enabling z-axis for any motion interrupt
        _acc_any_motion_int.anymotion_dur = 0;  // any-motion duration
        _acc_any_motion_int.anymotion_thr =
            20;  // (2-g range) -> (slope_thr) * 3.91 mg, (4-g range) -> (slope_thr) * 7.81 mg, (8-g
                 // range) ->(slope_thr) * 15.63 mg, (16-g range) -> (slope_thr) * 31.25 mg
    }

    rslt = enable_accel_any_motion_int(&_acc_any_motion_int, &_dev_bmi160);
    return rslt;
}

static int8_t int1Pin_init(AwCallBack_u8_t int1Cb) {
    _int1CB = int1Cb;
    return io_input_init(&_int1PinHandle, &_int1PinState, _int1PinTable, &int1CallbackFxn);
}
static void int1CallbackFxn(PIN_Handle handle, PIN_Id pinId) {
    if (_int1CB != NULL) {
        _int1CB(pinId);
    }
}
