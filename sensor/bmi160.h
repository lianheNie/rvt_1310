/*
 * bmi160.h
 *
 *  Created on: 2020Äê7ÔÂ31ÈÕ
 *      Author: admin
 */

#ifndef BMI160_H_
#define BMI160_H_

#include "driver_comm.h"
#include "bmi160_driver.h"
#include "io_driver.h"
int8_t bmi160_init(AwCallBack_u8_t int1Cb);
int8_t bmi160_init_no_imu();
int8_t bmi160_read_accel(bmi160_sensor_data_t* accel);
int8_t bmi160_read_gyro(bmi160_sensor_data_t* gyro);
int8_t bmi160_gyro_enable(uint8_t enable);
int8_t bmi160_accel_enable(uint8_t enable);
int8_t bmi160_acc_any_motion_int_enable(uint8_t enable);
#endif /* BMI160_H_ */
