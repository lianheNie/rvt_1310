/*
 * aw_oled.c
 *
 *  Created on: 2020年10月2日
 *      Author: admin
 */

#include "aw_oled.h"
#include "i2c_driver.h"
#include "util_delay.h"
#include <stdio.h>
#include <string.h>
#include "aw_oled_def.h"
#define AW_I2C_OLED_ADDRESS 0x3C  // 0x3C

#define _i2c_read            i2c_read
#define _i2c_write           i2c_write
#define _delay_ms            aw_delay_ms
#define AW_OLED_STR_BUF_SIZE 128

#define _OLED_CMD_REG  0x00
#define _OLED_DATA_REG 0x40
static char _str_buf[AW_OLED_STR_BUF_SIZE] = {'\0'};

static void _write_cmd(u8 cmd) { _i2c_write(AW_I2C_OLED_ADDRESS, _OLED_CMD_REG, &cmd, 1); }
static void _write_data(u8 data) { _i2c_write(AW_I2C_OLED_ADDRESS, _OLED_DATA_REG, &data, 1); }
static u8   _str_len = 0;
static void _oled_int(void) {
    _delay_ms(100);
    _write_cmd(0xAE);  // display off
    _write_cmd(0x20);  // Set Memory Addressing Mode
    _write_cmd(0x10);  // 00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page
                       // Addressing Mode (RESET);11,Invalid
    _write_cmd(0xb0);  // Set Page Start Address for Page Addressing Mode,0-7
    _write_cmd(0xc8);  // Set COM Output Scan Direction
    _write_cmd(0x00);  //---set low column address
    _write_cmd(0x10);  //---set high column address
    _write_cmd(0x40);  //--set start line address
    _write_cmd(0x81);  //--set contrast control register
    _write_cmd(0xff);  // 0x00~0xff
    _write_cmd(0xa1);  //--set segment re-map 0 to 127
    _write_cmd(0xa6);  //--set normal display
    _write_cmd(0xa8);  //--set multiplex ratio(1 to 64)
    _write_cmd(0x3F);  //
    _write_cmd(0xa4);  // 0xa4,Output follows RAM content;0xa5,Output ignores RAM content
    _write_cmd(0xd3);  //-set display offset
    _write_cmd(0x00);  //-not offset
    _write_cmd(0xd5);  //--set display clock divide ratio/oscillator frequency
    _write_cmd(0xf0);  //--set divide ratio
    _write_cmd(0xd9);  //--set pre-charge period
    _write_cmd(0x22);  //
    _write_cmd(0xda);  //--set com pins hardware configuration
    _write_cmd(0x12);
    _write_cmd(0xdb);  //--set vcomh
    _write_cmd(0x20);  // 0x20,0.77xVcc
    _write_cmd(0x8d);  //--set DC-DC enable
    _write_cmd(0x14);  //
    _write_cmd(0xaf);  //--turn on oled panel
}

static void _oled_set_pos(u8 x, u8 y) {
    _write_cmd(0xb0 + y);
    _write_cmd(((x & 0xf0) >> 4) | 0x10);
    _write_cmd((x & 0x0f) | 0x01);
}

static void _oled_fill(u8 fill_Data) {
    u8 m, n;
    for (m = 0; m < 8; m++) {
        _write_cmd(0xb0 + m);  // page0-page1
        _write_cmd(0x00);      // low column start address
        _write_cmd(0x10);      // high column start address
        for (n = 0; n < 128; n++) {
            _write_data(fill_Data);
        }
    }
}

static void _oled_clear(void) { _oled_fill(0x00); }

static void _oled_on(void) {
    _write_cmd(0X8D);
    _write_cmd(0X14);
    _write_cmd(0XAF);
}

static void _oled_off(void) {
    _write_cmd(0X8D);
    _write_cmd(0X10);
    _write_cmd(0XAE);
}

static s8 _oled_print_str(u8 x, u8 y, const char *ch, AW_OLED_FONT_t font) {
    u8 c = 0, i = 0, j = 0;
    switch (font) {
        case AW_OLED_F6x8: {
            while (ch[j] != '\0') {
                c = ch[j] - 32;
                if (x > 126) {
                    x = 0;
                    y = y + 1;
                }
                _oled_set_pos(x, y);
                for (i = 0; i < 6; i++) _write_data(F6x8[c][i]);
                x += 6;
                j++;
            }
        } break;
        case AW_OLED_F8X16: {
            while (ch[j] != '\0') {
                c = ch[j] - 32;
                if (x > 120) {
                    x = 0;
                    y = y + 2;
                }
                _oled_set_pos(x, y);
                for (i = 0; i < 8; i++) _write_data(F8X16[c * 16 + i]);
                _oled_set_pos(x, y + 1);
                for (i = 0; i < 8; i++) _write_data(F8X16[c * 16 + i + 8]);
                x += 8;
                j++;
            }
        } break;
    }
    return 0;
}

static void _oled_print_chinese(u8 x, u8 y, u8 N) {
    u8  wm    = 0;
    u32 adder = 32 * N;
    _oled_set_pos(x, y);
    for (wm = 0; wm < 16; wm++) {
        _write_data(F16x16[adder]);
        adder += 1;
    }
    _oled_set_pos(x, y + 1);
    for (wm = 0; wm < 16; wm++) {
        _write_data(F16x16[adder]);
        adder += 1;
    }
}

void _oled_draw_bmp(u8 x0, u8 y0, u8 x1, u8 y1, const u8 *BMP) {
    unsigned int j = 0;
    u8           x, y;
    if (y1 % 8 == 0)
        y = y1 / 8;
    else
        y = y1 / 8 + 1;
    for (y = y0; y < y1; y++) {
        _oled_set_pos(x0, y);
        for (x = x0; x < x1; x++) {
            _write_data(BMP[j++]);
        }
    }
}

#define _CENTER_X_OFFSET 32
s8 aw_oled_printf(AW_OLED_Clear_t clear, u8 x, u8 y, AW_OLED_FONT_t font,
                  const char *__restrict _format, ...) {
    _str_buf[0] = '\0';
    va_list args;
    va_start(args, _format);
    vsnprintf(_str_buf, AW_OLED_STR_BUF_SIZE, _format, args);
    va_end(args);
    if (clear == AW_OLED_Clear) {
        _oled_clear();
    }
    s8 res      = _oled_print_str(x, y, _str_buf, font);
    _str_buf[0] = '\0';
    return res;
}

s8 aw_oled_single_printf(AW_OLED_FONT_t font, const char *__restrict _format, ...) {
    _str_buf[0] = '\0';
    va_list args;
    va_start(args, _format);
    vsnprintf(_str_buf, AW_OLED_STR_BUF_SIZE, _format, args);
    va_end(args);
    u8 len = strlen(_str_buf);
    if (len < _str_len && _str_len > 0) {
        _oled_clear();
        _str_len = 0;
    } else {
        _str_len = len;
    }
    s8 res      = _oled_print_str(0, 0, _str_buf, font);
    _str_buf[0] = '\0';
    return res;
}
s8 aw_oled_put_f(char **buf, const char *__restrict _format, ...) {
    int     res = 0;
    int     len = strlen(_str_buf);
    va_list args;
    va_start(args, _format);
    int cnt = vsnprintf(&_str_buf[len], AW_OLED_STR_BUF_SIZE - len, _format, args);
    va_end(args);
    if ((cnt + len) > AW_OLED_STR_BUF_SIZE) {
        res = -1;
    }
    if (0 == res) {
        *buf = _str_buf;
    } else {
        *buf = NULL;
    }
    return res;
}
s8 aw_oled_flush(AW_OLED_Clear_t clear, u8 x, u8 y, AW_OLED_FONT_t font) {
    u8 len = strlen(_str_buf);
    if (len > _str_len && _str_len > 0) {
        _oled_clear();
        _str_len = 0;
    } else {
        _str_len = len;
    }
    if (clear == AW_OLED_Clear && _str_len > 0) {
        _str_len = 0;
        _oled_clear();
    }
    s8 res      = _oled_print_str(x, y, _str_buf, font);
    _str_buf[0] = '\0';
    return res;
}
static void _oled_print_logo() {
    u8 i = 0;
    for (i = 0; i < 4; i++) {
        _oled_print_chinese(_CENTER_X_OFFSET + i * 16, 0, i);  //显示中文字符
    }
}
void aw_oled_init() {
    _oled_int();
    _oled_fill(0x00);
    _oled_print_logo();
}
void aw_oled_test() {
    aw_oled_init();
    _delay_ms(2000);
    _oled_draw_bmp(0, 0, 128, 8, BMP1);
    _delay_ms(2000);
    _oled_clear();
    _delay_ms(1000);
    _oled_off();
    _delay_ms(1000);
    _oled_on();
    _delay_ms(1000);
    _oled_print_logo();
    int cnt = 0;
    while (1) {
        aw_oled_printf(AW_OLED_NO_Clear, 0, 2, AW_OLED_F8X16, "cnt:%d", cnt++);
        // aw_oled_printf(AW_OLED_NO_Clear, 0, 4, AW_OLED_F8X16, "cnt2:%d", cnt2++);
        _delay_ms(100);
        if (cnt > 100) {
            cnt = 0;
        }
    }
}
