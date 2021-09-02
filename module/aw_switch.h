/*
 * aw_remoteSwitch.h
 *
 *  Created on: 2020年9月23日
 *      Author: admin
 */

#ifndef MODULE_AW_SWITCH_H_
#define MODULE_AW_SWITCH_H_
#include <iot/easylink/aw_easylink_def.h>
#include "aw_config.h"
#include "util_def.h"

#ifdef IS_USE_SERVO_SWITCH
#define SW_CH_CNT 2  //云台开关 单路正负开关模块
#else
#define SW_CH_CNT 3  //电源模块    //三路开关模块
#endif

#if defined(IS_USE_SWITCH3_SENSOR)
#define SW_CH1_PIN_ID IOID_4  // IOID_8  IOID_4     #define CH1_12V_PIN_ID IOID_8
#define SW_CH2_PIN_ID IOID_5  // IOID_0  IOID_5     #define CH2_5V_PIN_ID  IOID_0
#define SW_CH3_PIN_ID IOID_7  // IOID_1  IOID_7     #define CH3_4V_PIN_ID  IOID_1
#elif defined(IS_USE_SERVO_SWITCH)
#define SW_CH1_PIN_ID IOID_4  // IOID_8  IOID_4     #define CH1_12V_PIN_ID IOID_8
#define SW_CH2_PIN_ID IOID_5  // IOID_0  IOID_5     #define CH2_5V_PIN_ID  IOID_0
#else
#define SW_CH1_PIN_ID IOID_8  // IOID_8  IOID_4     #define CH1_12V_PIN_ID IOID_8
#define SW_CH2_PIN_ID IOID_0  // IOID_0  IOID_5     #define CH2_5V_PIN_ID  IOID_0
#define SW_CH3_PIN_ID IOID_1  // IOID_1  IOID_7     #define CH3_4V_PIN_ID  IOID_1
#endif

s8                aw_sw_init(E_sw_sts_t sts);
bool              sw_set_chs_delay(E_sw_chs_t* chs, u8 cnt, E_sw_sts_t sts, u16 ms);
bool              sw_set_chs_sts(E_sw_sts_t* sts, u16 ms);
const E_sw_sts_t* sw_get_chs();
bool              sw_set_chs(E_sw_chs_t* chs, u8 cnt, E_sw_sts_t sts);
bool              sw_set_ch(E_sw_chs_t ch, E_sw_sts_t sts);
#endif /* MODULE_AW_SWITCH_H_ */
