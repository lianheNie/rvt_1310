/*
 * ads1120.c
 *
 *  Created on: 2020Äê11ÔÂ6ÈÕ
 *      Author: admin
 */

#include "ads1120.h"
#include "util_delay.h"
#include <math.h>
#include <ti/drivers/PIN.h>
#include <ti/devices/cc13x0/driverlib/ioc.h>
/* Command Definitions */
#define ADS1120_TEMP_SCALE   100
#define ADS1120_CMD_RDATA    0x10
#define ADS1120_CMD_RREG     0x23
#define ADS1120_CMD_WREG     0x43
#define ADS1120_CMD_SYNC     0x08
#define ADS1120_CMD_SHUTDOWN 0x02
#define ADS1120_CMD_RESET    0x06
/* ADS1120 Register Definitions */
#define ADS1120_0_REGISTER 0x00
#define ADS1120_1_REGISTER 0x01
#define ADS1120_2_REGISTER 0x02
#define ADS1120_3_REGISTER 0x03

#define RTD_3_Wire_0_REG_VAL \
    0x04  // 0x66  0x01  // AINP = AIN1, AINN = AIN0, gain = 4, PGA disenabled
          //
#define RTD_3_Wire_1_REG_VAL 0x00
// 0x00  // 0x00-single-shot mode  0x04-continuous conversion mode  //DR = 20 SPS,  normal mode

#define RTD_3_Wire_2_REG_VAL \
    0x55  // 0x15  0x55   //Internal 2.048-V reference/External reference (REFP0, REFN0),
          // simultaneous 50-Hz and 60-Hz rejection, IDAC = 1000/500 ?A
#define RTD_3_Wire_2_REG_INTERNAL_VAL \
    0x15  // 0x15  0x55   //Internal 2.048-V reference/External reference (REFP0, REFN0),

#define RTD_3_Wire_2_REG_EXTERNAL_VAL \
    0x55  // 0x15  0x55   //Internal 2.048-V reference/External reference (REFP0, REFN0),
#define RTD_3_Wire_3_REG_VAL \
    0x70  // 0x70  0x72   //IDAC1 = AIN2, IDAC2 = AIN3  DRDY mode  Only the dedicated DRDY/DOUT/DRDY
          // and DRDY
//#define RTD_A 3.9083e-3
//#define RTD_B -5.775e-7

const float RTD_A          = 3.9083E-3;
const float RTD_B          = -5.775E-7;
const float RTD_R0         = 100;
const float RTD_RES_OFFSET = 1.23;
#define CC1310_ADS_SCLK IOID_24 /* RF1.16 */
#ifdef IS_USE_7X7_PACKAGE
#define CC1310_ADS_CS IOID_25
#else
#define CC1310_ADS_CS IOID_8
#endif
#define CC1310_ADS_CLK  IOID_26
#define CC1310_ADS_DOUT IOID_27 /* RF1.20 */
#define CC1310_ADS_DIN  IOID_28 /* RF1.18 */
#define CC1310_ADS_DRDY IOID_29
static PIN_Handle _tempPinHandle;
static PIN_State  _tempPinState;
static PIN_Config _tempPinTable[] = {
    CC1310_ADS_CS | PIN_GPIO_OUTPUT_EN | PIN_DRVSTR_MIN | PIN_PUSHPULL | PIN_GPIO_HIGH,
#ifdef IS_USE_7X7_PACKAGE
    CC1310_ADS_CLK | PIN_GPIO_OUTPUT_EN | PIN_PUSHPULL | PIN_GPIO_LOW,

    CC1310_ADS_DRDY | PIN_INPUT_EN | PIN_PULLUP,
#endif
    PIN_TERMINATE /* Terminate list */
};
s8 ads1120_spi_open() {
    s8 res = 0;
    spi_open(SPI_POL0_PHA1);
    return res;
}
s8 ads1120_spi_close() {
    s8 res = 0;
    spi_close();
    return res;
}
static void _ads1120_cs_low() { PIN_setOutputValue(_tempPinHandle, CC1310_ADS_CS, 0); }
static void _ads1120_cs_heigh() { PIN_setOutputValue(_tempPinHandle, CC1310_ADS_CS, 1); }
static s8   _ads1120_reset() {
    s8 res = 0;
    _ads1120_cs_low();
    res = spi_write_cmd(ADS1120_CMD_RESET);
    _ads1120_cs_heigh();
    return res;
}
static int32_t GetADCRange() { return 0x00007fff; }
#define AD_DEFREFRES 390.0f
#define AD_GAIN      4
#define ADC_V_MAX    ((AD_DEFREFRES * 2 * 0.8) / 1000)
static float Convert2Res(int32_t adc) {
    int32_t adcRange = GetADCRange();
    float   res      = (adc * (AD_DEFREFRES * 2) / (AD_GAIN) / adcRange);
    return (res - RTD_RES_OFFSET);
}
static float Convert2Res_internal(s16 adc) {
    int32_t adcRange = GetADCRange();
    float   res      = ((float)adc * 2.048 * 2000) / adcRange;
    return res;
}
static float Convert2Temp(float res) {
    return (-RTD_A + sqrt((RTD_A * RTD_A) - (4 * RTD_B * (1 - (res / RTD_R0))))) / (2 * RTD_B);
}
s8 ads1120_init() {
    s8 res        = 0;
    u8 ww_regs[4] = {RTD_3_Wire_0_REG_VAL, RTD_3_Wire_1_REG_VAL, RTD_3_Wire_2_REG_VAL,
                     RTD_3_Wire_3_REG_VAL};
    u8 rr_regs[4] = {0, 0, 0, 0};

    _tempPinHandle = PIN_open(&_tempPinState, _tempPinTable);
#ifdef IS_USE_7X7_PACKAGE
    PIN_setOutputValue(_tempPinHandle, CC1310_ADS_CLK, 0);
#endif
    ads1120_spi_open();
    _ads1120_reset();
    aw_delay_ms(1000);
    _ads1120_cs_low();
    spi_write(ADS1120_CMD_WREG, ww_regs, sizeof(ww_regs));
    spi_read(ADS1120_CMD_RREG, rr_regs, sizeof(rr_regs));
    spi_write_cmd(ADS1120_CMD_SHUTDOWN);
    _ads1120_cs_heigh();
    ads1120_spi_close();
    return res;
}
#define ADS1120_MIN_TEMP_VAL -100
s8 ads1120_set_internal_source(s16* temp_val) {
    s8 res        = 0;
    u8 ww_regs[4] = {0x01, 0x00, 0x15, 0x70};
    u8 rr_regs[4] = {0, 0, 0, 0};
    ads1120_spi_open();
    _ads1120_cs_low();
    spi_write(ADS1120_CMD_WREG, ww_regs, sizeof(ww_regs));
    _ads1120_cs_heigh();
    aw_delay_ms(1);
    _ads1120_cs_low();
    spi_read(ADS1120_CMD_RREG, rr_regs, sizeof(rr_regs));
    _ads1120_cs_heigh();
    aw_delay_ms(1);

    s16 val = 0;
    ads1120_read_raw(&val);
    float ress = Convert2Res_internal(val);
    float temp = Convert2Temp(ress);
    *temp_val=temp;
    if (temp < ADS1120_MIN_TEMP_VAL) {
        res = -1;
    }
    ads1120_spi_close();
    return res;
}
s8 ads1120_set_external_source() {
    s8 res        = 0;
    u8 ww_regs[4] = {RTD_3_Wire_0_REG_VAL, RTD_3_Wire_1_REG_VAL, RTD_3_Wire_2_REG_EXTERNAL_VAL,
                     RTD_3_Wire_3_REG_VAL};
    u8 rr_regs[4] = {0, 0, 0, 0};
    ads1120_spi_open();

    _ads1120_cs_low();
    spi_write(ADS1120_CMD_WREG, ww_regs, sizeof(ww_regs));
    _ads1120_cs_heigh();
    aw_delay_ms(1);
    _ads1120_cs_low();
    spi_read(ADS1120_CMD_RREG, rr_regs, sizeof(rr_regs));
    _ads1120_cs_heigh();
    aw_delay_ms(100);
    ads1120_spi_close();
    return res;
}
s8 ads1120_close() {
    s8 res = 0;
    _ads1120_cs_low();
    res = spi_write_cmd(ADS1120_CMD_SHUTDOWN);
    _ads1120_cs_heigh();
    return res;
}
#ifdef IS_USE_7X7_PACKAGE
#define _ADS1120_TIMEROUT 50
s8 _ads1120_convdone() {
    s8 cnt = 0;
    while (PIN_getInputValue(CC1310_ADS_DRDY) != 0) {
        if (cnt++ > _ADS1120_TIMEROUT) {
            //æ­¤æ—¶å·²è¶…æ—¶ã??
            return -1;
        }
        aw_delay_ms(100);
    }
    return 0;
}
#endif
s8 ads1120_read_raw(s16* val) {
    s8 res     = 0;
    u8 vals[2] = {0};
    _ads1120_cs_low();
    res = spi_write_cmd(ADS1120_CMD_SYNC);
    _ads1120_cs_heigh();
#ifdef IS_USE_7X7_PACKAGE
    if (_ads1120_convdone() != 0) {
        return -1;
    }
#else
    aw_delay_ms(1000);
#endif
    _ads1120_cs_low();
    res = spi_read(ADS1120_CMD_RDATA, vals, sizeof(vals));
    _ads1120_cs_heigh();
    *val = vals[0] << 8 | vals[1];

    return res;
}
s8 ads1120_read_temp(s16* val) {
    s8  res        = 0;
    s16 rawDataU16 = 0;
    ads1120_spi_open();
    res              = ads1120_read_raw(&rawDataU16);
    float ress       = Convert2Res(rawDataU16);
    float tempF      = Convert2Temp(ress);
    s16   tempData16 = (s16)(tempF * ADS1120_TEMP_SCALE);
    *val             = tempData16;
    ads1120_close();
    ads1120_spi_close();
    return res;
}
s8 ads1120_test() {
    s8  res  = 0;
    s16 temp = 0;
    res      = ads1120_init();
    u8 cnt   = 0;
    while (cnt++ < 10000) {
        aw_delay_ms(1000);
        res = ads1120_read_temp(&temp);
    }
    return res;
}
