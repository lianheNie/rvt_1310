/*
 * task_app.c
 *
 *  Created on: 2020年9月25日
 *      Author: admin
 */
#include <xdc/runtime/Error.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/BIOS.h>
#include <stdio.h>
#include <string.h>
#include "uart_driver.h"
#include "util_printf.h"
#include "util_other.h"
#include "util_delay.h"
#include "util_def.h"
#include <string.h>
#include <stdio.h>
#include <aw_config.h>
#include <sys_ctrl.h>
#include <aon_batmon.h>

/////////////////////////////////////
#define TIMER_EVENT     0x0800
static u16              _events = 0;
static Semaphore_Handle _appSem = NULL;
static s16 _axisArray[2046 / 3];

static bool _app_sem_init() {
    Semaphore_Params params;
    Error_Block      eb;
    Semaphore_Params_init(&params);
    Error_init(&eb);
    _appSem = Semaphore_create(0, &params, &eb);
    if (_appSem == NULL) {
        // System_abort("Semaphore creation failed");
    }
    return true;
}

#include <EasyLink.h>
#include "aw_easylink.h"
static Easylink_TX_t _easylink_tx_pkt;
extern bool          easylink_txCB(Easylink_TX_t *txPacket);

#include "lsm6dsm.h"
#include "aw_temper.h"

static u16 _sensor_cnt = 0;
static s8  _process_sensor(s16 type,s16 temp,s16 Xval,s16 Yval,s16 Zval,s16 axisArray[] ) {
    Aw_Sensor_t _sensor;
    u16 i = 0;
    _sensor.Yval = Yval;
    _sensor.id   = AW_EASYLINK_SELF_ADDRESS;
    _sensor.src_id = _sensor.id;
    _sensor.type = type;
    _sensor.Zval = Zval;
    _sensor.Xval = Xval;
    _sensor.temp = temp;
    for(i = 0; i < 30; i++) {
        _sensor.axisArray[i] = axisArray[i];
    }

    _easylink_tx_pkt.len    = aw_sensorToPayload(&_sensor, _easylink_tx_pkt.payload);
    _easylink_tx_pkt.dstAdd = 66; //9;//aw_easylink_pair_get();
    easylink_txCB(&_easylink_tx_pkt);

    return 0;
}


#include "util_timer.h"
#define _TIMEOUT 10 * 1000  // 10s
static Clock_Struct _clockS;

static void _timeout_cb(u32 arg) {
  _events |= TIMER_EVENT;
  if (NULL != _appSem) {
    Semaphore_post(_appSem);
  }
}

static void _timer_init() {
  Timer_construct(&_clockS, _timeout_cb, _TIMEOUT, _TIMEOUT,
                  false, 0);
  Timer_start(&_clockS);  //开定时器
}

static void _proc_timer_event() {
    s16 tempt = 0;
    aw_temper_read(&tempt);
//    _process_sensor(10,tempt,++_sensor_cnt);
    //E_xyz_axis_t val;
    //s16 val = 0;
    lsm6dsm_spi_open();
    lsm6dsm_accel_enable(1);
    aw_delay_ms(500);
    while(lsm6dsm_fifo_full() < 2046) {
        aw_delay_ms(100);
    }
      //E_xyz_axis_t axis;
    u16 fifo_data_cnt = lsm6dsm_fifo_full();
    if(fifo_data_cnt >= 2046) {
        u16 i = 0;
       for(i = 0;i < 30; i += 3) {
           s16 acc_x = lsm6dsm_read_accel(AW_X_AXIS);
           _axisArray[i] = acc_x;
           s16 acc_y = lsm6dsm_read_accel(AW_Y_AXIS);
           _axisArray[i+1] = acc_y;
           s16 acc_z= lsm6dsm_read_accel(AW_Z_AXIS);
           _axisArray[i+2] = acc_z;
       }
    }
   //s16 xAcc= lsm6dsm_read_accel(AW_X_AXIS);
   //s16 yAcc= lsm6dsm_read_accel(AW_Y_AXIS);
   //s16 zAcc= lsm6dsm_read_accel(AW_Z_AXIS);
//    s16 acc = lsm6dsm_read_accel((E_xyz_axis_t)val);
   lsm6dsm_accel_enable(0);
   aw_delay_us(10);
   lsm6dsm_spi_close();
   _process_sensor(10,tempt,0,0,0,_axisArray);

   //_process_sensor(10,tempt,xAcc,yAcc,zAcc);
  // _process_sensor(10,yAcc,++_sensor_cnt);
  //_process_sensor(10,zAcc,++_sensor_cnt);
//    int x = 0,y = 0,z = 0;
//    lsm6dsm_read_accel();
}

static void _proc_events() {
    if(_events & TIMER_EVENT){
        _proc_timer_event();
        Util_clearEvent(&_events, TIMER_EVENT);
    }
}

static void _app_init() {
    _app_sem_init();
    _timer_init(); //定时器初始化
    lsm6dsm_init(); //振动传感器初始化
    aw_temper_init(); //温度传感器初始化
}
static void _app_process() {
    while (1) {
        Semaphore_pend(_appSem, BIOS_WAIT_FOREVER);
        _proc_events();
    }
}

static void _app_thread(UArg arg0, UArg arg1) {
    _app_init();
    _app_process();
}

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#define _TASK_STACK_SIZE 1024
static Task_Struct _task_struct;
static Char        _task_stack[_TASK_STACK_SIZE];
void               aw_app_task_init() {
    Task_Params taskParams;
    Task_Params_init(&taskParams);
    taskParams.stackSize = _TASK_STACK_SIZE;
    taskParams.stack     = &_task_stack;
    taskParams.priority  = 2;
    Task_construct(&_task_struct, (Task_FuncPtr)_app_thread, &taskParams, NULL);
}
