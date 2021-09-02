/*
 * aw_led.h
 *
 *  Created on: 2020Äê9ÔÂ23ÈÕ
 *      Author: admin
 */

#ifndef MODULE_AW_LED_H_
#define MODULE_AW_LED_H_
#include "aw_config.h"
#include "util_def.h"
void aw_led_init();
void aw_set_led(u8 val);
void aw_toggle_led();
void aw_blink_led(u8 cnt, u16 ms);
void aw_led_enable(bool is_enable);
#endif /* MODULE_AW_LED_H_ */
