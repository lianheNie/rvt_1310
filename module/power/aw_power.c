/*
 * aw_powerctl.c
 *
 *  Created on: 2020年9月23日
 *      Author: admin
 */

#include <aw_power.h>
#include "util_delay.h"
#include "Board.h"
#include "io_driver.h"
#define _SWITCH_PIN_ID IOID_8  // IOID_8
#define POWER_PIN_ID   IOID_9  // IOID_9
#define POWER_ON       1
#define POWER_OFF      0
static PIN_Handle _ioPowerPinHandle;
static PIN_State _ioPowerPinState;
static PIN_Config _ioPowerPinTable[] = {
        POWER_PIN_ID | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL
                | PIN_DRVSTR_MAX,
        _SWITCH_PIN_ID | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_POSEDGE,
        PIN_TERMINATE };

static AwCallBack_void_t _switchCb = NULL;
static void _switchIntCb(PIN_Handle handle, PIN_Id pinId)
{
    if (NULL != _switchCb)
    {
        _switchCb();
    }
}
static s8 _io_power_init(AwCallBack_void_t switchCb)
{
    _switchCb = switchCb;
    _ioPowerPinHandle = PIN_open(&_ioPowerPinState, _ioPowerPinTable);
    if (_ioPowerPinHandle == NULL)
    {
        return -1;
    }
    if (PIN_registerIntCb(_ioPowerPinHandle, _switchIntCb) != 0)
    {
        return -1;
    }
    PIN_setOutputValue(_ioPowerPinHandle, POWER_PIN_ID, POWER_ON);
    return 0;
}

bool aw_power_switch_set(AW_power_ctl_t val)
{
    if (_ioPowerPinHandle == NULL)
    {
        return false;
    }
    PIN_setOutputValue(_ioPowerPinHandle, POWER_PIN_ID, (u32) val);
    return true;
}
u32 aw_power_switch_get()
{
    return PIN_getInputValue(_SWITCH_PIN_ID);
}
void aw_power_switch_init(AwCallBack_void_t switchCb)
{
    _io_power_init(switchCb);
}

#include "scs/scif.h"
#include "scs/ex_include_tirtos.h"
static u16 aw_to_millivolts(int32_t adcValue)
{
    adcValue =
    AUXADCAdjustValueForGainAndOffset(
            adcValue, AUXADCGetAdjustmentGain(AUXADC_REF_FIXED),
            AUXADCGetAdjustmentOffset(AUXADC_REF_FIXED));
    return AUXADCValueToMicrovolts(AUXADC_FIXED_REF_VOLTAGE_NORMAL, adcValue)
            / 1000;
}  // millivoltsToAdcValue

static void scCtrlReadyCallback(void)
{
}  // scCtrlReadyCallback
#define BV(n) (1 << (n))
static void _adc_init(AwCallBack_void_t adcCb)
{
    scifOsalInit();
    scifOsalRegisterCtrlReadyCallback(scCtrlReadyCallback);
    scifOsalRegisterTaskAlertCallback(adcCb);
    scifInit(&scifDriverSetup);
    scifStartRtcTicksNow(65536 * 1 * 2);  // 2*60s
    scifStartTasksNbl(BV(SCIF_ADC_WINDOW_MONITOR_TASK_ID));
}

static bool _is_adcFirst = true;
static u8 _is_adcFirstCnt = 0;
static u8 _lowCnt = 0;
static bool _lowFlag = false;
#define _POWER_ADC_RATIO   2.03f     // ADC 电阻分压系数
#define _POWER_ADC_PERIOD  (2 * 60)  // 2*60秒
#define _POWER_ADC_LOW_VAL 3400      //低电压
#define _POWER_ADC_LOW_CNT 10        // 检测低电压次数
static u16 _power_adc_mv = 0;
u16 aw_power_adc_get()
{
    return _power_adc_mv;
}
bool aw_power_adc_update(u16 *mV)
{
    scifClearAlertIntSource();  // Clear the ALERT interrupt source
    u16 milliVolt = aw_to_millivolts(
            scifTaskData.adcWindowMonitor.output.adcValue);
    milliVolt = (u16) (milliVolt * _POWER_ADC_RATIO);
    if (NULL != mV)
        *mV = milliVolt;
    _power_adc_mv = milliVolt;
    if (milliVolt < _POWER_ADC_LOW_VAL)
    {  // 3.0v
        if (_lowFlag)
        {
            _lowCnt++;
            if (_lowCnt >= _POWER_ADC_LOW_CNT)
            {
                return true;
            }
        }
        else
        {
            _lowCnt = 0;
            _lowFlag = true;
        }
    }
    else
    {
        _lowFlag = false;
        _lowCnt = 0;
    }
    scifAckAlertEvents();  // Acknowledge the alert event
    if (_is_adcFirst)
    {
        _is_adcFirstCnt++;
        if (_is_adcFirstCnt > 5)
        {
            _is_adcFirstCnt = 0;
            _is_adcFirst = false;
            scifStopRtcTicks();
            scifStopTasksNbl(BV(SCIF_ADC_WINDOW_MONITOR_TASK_ID));
            scifUninit();
            aw_delay_ms(1);
            scifInit(&scifDriverSetup);
            scifStartRtcTicksNow(65536 * 2 * 60);  // 2*60s
            scifStartTasksNbl(BV(SCIF_ADC_WINDOW_MONITOR_TASK_ID));
        }
    }
    return false;
}

void aw_power_adc_init(AwCallBack_void_t adcCb)
{
    _adc_init(adcCb);
}

void aw_power_init(AwCallBack_void_t switchCb, AwCallBack_void_t adcCb)
{
    _io_power_init(switchCb);
    _adc_init(adcCb);
}
