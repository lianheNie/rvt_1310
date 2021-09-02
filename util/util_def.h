/*
 * util_def.h
 *
 *  Created on: 2020Äê7ÔÂ31ÈÕ
 *      Author: admin
 */
#ifndef UTIL_DEF_H_
#define UTIL_DEF_H_
#include <stdint.h>
/************************************************/
/*unsigned integer types*/
typedef uint8_t  u8;  /**< used for unsigned 8bit */
typedef uint16_t u16; /**< used for unsigned 16bit */
typedef uint32_t u32; /**< used for unsigned 32bit */
typedef uint64_t u64; /**< used for unsigned 64bit */

/*signed integer types*/
typedef int8_t  s8;  /**< used for signed 8bit */
typedef int16_t s16; /**< used for signed 16bit */
typedef int32_t s32; /**< used for signed 32bit */
typedef int64_t s64; /**< used for signed 64bit */

typedef int8_t  AW_S8_VAULE; /**< used for signed 8bit */
typedef uint8_t AW_U8_VAULE; /**< used for signed 8bit */
/// typedef uint8_t AW_BOOL; /**< used for signed 8bit */
typedef void (*AwCallback_u32_t)(u32 arg);
typedef void (*AwCallback_s32_t)(s32 arg);
typedef void (*AwCallBack_u16_t)(s16 arg);
typedef void (*AwCallBack_s16_t)(s16 arg);
typedef void (*AwCallBack_u8_t)(u8 arg);
typedef void (*AwCallBack_s8_t)(s8 arg);
typedef void (*AwCallBack_void_t)(void);
typedef void (*AwCallBack_float_t)(float arg);
typedef int8_t (*AwCallBack_char_t)(const char *str);
typedef int8_t (*aw_i2c_com_fptr_t)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data,
                                    uint16_t len);

typedef void (*aw_delay_fptr_t)(u32 period);

struct _aw_i2c_dev_t {
    /*! Chip Id */
    uint16_t chip_id;
    /*! Device Id */
    uint8_t id;
    /*! Read function pointer */
    aw_i2c_com_fptr_t read;
    /*! Write function pointer */
    aw_i2c_com_fptr_t write;
    /*!  Delay function pointer */
    aw_delay_fptr_t delay_ms;
};
typedef struct _aw_i2c_dev_t aw_i2c_dev_t;
/////////////////////////////////////////////////////

/* ------------------------------------------------------------------------------------------------
 *                                             Macros
 * ------------------------------------------------------------------------------------------------
 */

#ifndef BV
#define AW_BV(n) (1 << (n))
#endif

#ifndef BF
#define AW_BF(x, b, s) (((x) & (b)) >> (s))
#endif

#ifndef MIN
#define AW_MIN(n, m) (((n) < (m)) ? (n) : (m))
#endif

#ifndef MAX
#define AW_MAX(n, m) (((n) < (m)) ? (m) : (n))
#endif

#ifndef ABS
#define AW_ABS(n) (((n) < 0) ? -(n) : (n))
#endif

/* takes a byte out of a uint32 : var - uint32,  ByteNum - byte to take out (0 - 3) */
#define AW_BREAK_UINT32(var, ByteNum) (uint8)((uint32)(((var) >> ((ByteNum)*8)) & 0x00FF))

#define AW_BUILD_UINT32(Byte0, Byte1, Byte2, Byte3)                        \
    ((uint32)((uint32)((Byte0)&0x00FF) + ((uint32)((Byte1)&0x00FF) << 8) + \
              ((uint32)((Byte2)&0x00FF) << 16) + ((uint32)((Byte3)&0x00FF) << 24)))

#define AW_BUILD_UINT16(loByte, hiByte) ((uint16)(((loByte)&0x00FF) + (((hiByte)&0x00FF) << 8)))

#define AW_HI_UINT16(a) (((a) >> 8) & 0xFF)
#define AW_LO_UINT16(a) ((a)&0xFF)

#define AW_BUILD_UINT8(hiByte, loByte) ((uint8)(((loByte)&0x0F) + (((hiByte)&0x0F) << 4)))

#define AW_HI_UINT8(a) (((a) >> 4) & 0x0F)
#define AW_LO_UINT8(a) ((a)&0x0F)

// Write the 32bit value of 'val' in little endian format to the buffer pointed
// to by pBuf, and increment pBuf by 4
#define AW_UINT32_TO_BUF_LITTLE_ENDIAN(pBuf, val)     \
    do {                                              \
        *(pBuf)++ = ((((uint32)(val)) >> 0) & 0xFF);  \
        *(pBuf)++ = ((((uint32)(val)) >> 8) & 0xFF);  \
        *(pBuf)++ = ((((uint32)(val)) >> 16) & 0xFF); \
        *(pBuf)++ = ((((uint32)(val)) >> 24) & 0xFF); \
    } while (0)

// Return the 32bit little-endian formatted value pointed to by pBuf, and increment pBuf by 4
#define AW_BUF_TO_UINT32_LITTLE_ENDIAN(pBuf) \
    (((pBuf) += 4), BUILD_UINT32((pBuf)[-4], (pBuf)[-3], (pBuf)[-2], (pBuf)[-1]))

#ifndef GET_BIT
#define AW_GET_BIT(DISCS, IDX) (((DISCS)[((IDX) / 8)] & BV((IDX) % 8)) ? TRUE : AW_FALSE)
#endif
#ifndef SET_BIT
#define AW_SET_BIT(DISCS, IDX) (((DISCS)[((IDX) / 8)] |= BV((IDX) % 8)))
#endif
#ifndef CLR_BIT
#define AW_CLR_BIT(DISCS, IDX) (((DISCS)[((IDX) / 8)] &= (BV((IDX) % 8) ^ 0xFF)))
#endif
#define AW_TOSTRING(s) #s
#define AW_STR(p)      AW_TOSTRING(p)
#define AW_st(x) \
    do {         \
        x        \
    } while (__LINE__ == -1)

typedef uint8_t AW_Status_t;

// Data types
typedef int32_t  AW_int24;
typedef uint32_t AW_uint24;
#define AW_BOOL _Bool
#define bool _Bool
#define AW_NULL 0L
////////////////////////////////////////////////////////////
#define AW_TRUE  1
#define AW_FALSE 0

#define true 1
#define false 0
#define AW_ERR      -1
#define AW_NULL_ERR -2
#define AW_OK       0
#define AW_SUCCESS  0
typedef enum { AW_SW_CH1, AW_SW_CH2, AW_SW_CH3, AW_SW_CNT } E_sw_chs_t;
typedef enum { AW_X_AXIS, AW_Y_AXIS, AW_Z_AXIS, AW_AXIS_CNT } E_xyz_axis_t;
typedef enum { AW_SW_NEG = -1, AW_SW_OFF, AW_SW_ON, AW_SW_NULL } E_sw_sts_t;
typedef struct _Easylink_Sensor_t {
    u16 src_id;
    u8  type;
    u16 id;
    s16 Xval;
    s16 Yval;
    s8  rssi;
    s16 Zval;
    s16 temp;
    s16 axisArray[2046 / 3];
} Aw_Sensor_t;
//typedef struct _Easylnk_Sensor_t {
//    u16 src_id;
//    u8  type;
//    u16 id;
//    s16 temp;
//    s16 axisArray[2046 / 3];
//}  Aw_Sensor_t;

#endif /* UTIL_DEF_H_ */
