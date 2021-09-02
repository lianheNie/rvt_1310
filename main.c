/*
 * Copyright (c) 2016-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,

 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <xdc/std.h>
#include <xdc/runtime/System.h>
/* BIOS module Headers */
#include <ti/sysbios/BIOS.h>
#include <ti/drivers/Board.h>
#include "i2c_driver.h"
#include "spi_driver.h"
#include <aw_config.h>
/*
 *  ======== main ========
 */
#ifndef IS_USE_SENSOR_TEST
#if AW_DEV_TYPE != AW_devtype_rs_3 && AW_DEV_TYPE != AW_devtype_rs_np_1 && \
    AW_DEV_TYPE != AW_devtype_gateway_rs_np_1
static s8 _driver_init() {
    s8 rslt = 0;
//#ifndef IS_USE_LSM6DSM_SPI
//    rslt += i2c_init();
//#endif
//#ifdef IS_USE_RT
//    rslt += spi_init();
//#endif
    rslt += spi_init();
    return rslt;
}
#endif

#ifdef IS_USE_IMU
extern void aw_imu_task_init();
#endif
#ifdef IS_USE_EASYLINK
extern void aw_easylink_task_init();
#endif

#ifdef IS_USE_MQTT
#ifdef IS_USE_AT_CLIENT
extern void aw_at_client_task_init();
#endif
#endif
//#include "task_app.h"
extern void aw_app_task_init();

#else
extern void aw_hc_sr04_task_test_init();
#endif


static s8 _driver_init(){
    s8 rslt = 0;
    rslt += spi_init();
    return rslt;
}


int main(void) {
    /* Call driver init functions */
    Board_init();
    _driver_init();

#if AW_DEV_TYPE != AW_devtype_rs_3 && AW_DEV_TYPE != AW_devtype_rs_np_1 && \
    AW_DEV_TYPE != AW_devtype_gateway_rs_np_1
    _driver_init();
#endif

#ifdef IS_USE_IMU
    aw_imu_task_init();
#endif

#ifdef IS_USE_MQTT
#ifdef IS_USE_AT_CLIENT
    aw_at_client_task_init();  // AT客户端线程初始化
#endif
#endif

#ifdef IS_USE_EASYLINK
    aw_easylink_task_init(); //433初始化
#endif
    aw_app_task_init(); //主线程初始化

#ifdef IS_USE_C_TEST  // C语言测试
    {
        extern int main_C_test();
        main_C_test();
    }
#endif

    BIOS_start();
    return (0);
}
