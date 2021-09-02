/*
 * aw_oled.h
 *
 *  Created on: 2020年10月2日
 *      Author: admin
 */

#ifndef MODULE_AW_OLED_H_
#define MODULE_AW_OLED_H_
#include "util_def.h"
typedef enum {
    AW_OLED_F6x8 = 1,  //字体1
    AW_OLED_F8X16,     //字体2
} AW_OLED_FONT_t;
typedef enum {
    AW_OLED_Clear = 1,  //清屏
    AW_OLED_NO_Clear,   //不清屏
} AW_OLED_Clear_t;
void aw_oled_init();
void aw_oled_test();
s8   aw_oled_printf(AW_OLED_Clear_t clear, u8 x, u8 y, AW_OLED_FONT_t font,
                    const char *__restrict _format, ...);
s8   aw_oled_single_printf(AW_OLED_FONT_t font, const char *__restrict _format, ...);
s8   aw_oled_put_f(char **buf, const char *__restrict _format, ...);
s8   aw_oled_flush(AW_OLED_Clear_t clear, u8 x, u8 y, AW_OLED_FONT_t font);
#endif /* MODULE_AW_OLED_H_ */
