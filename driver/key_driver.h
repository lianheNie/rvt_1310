/*
 * key_driver.h
 *
 *  Created on: 2020Äê8ÔÂ1ÈÕ
 *      Author: admin
 */

#ifndef KEY_DRIVER_H_
#define KEY_DRIVER_H_
#include "driver_comm.h"
#include "Board.h"
#include "io_driver.h"
#define KEY_1 Board_PIN_BUTTON0
#define KEY_2 Board_PIN_BUTTON1
int8_t key_init(AwCallBack_u8_t keyCb);
uint32_t key_read(uint8_t pinId);
#endif /* KEY_DRIVER_H_ */
