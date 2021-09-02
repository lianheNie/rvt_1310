/*
 * util_delay.c
 *
 *  Created on: 2020Äê7ÔÂ31ÈÕ
 *      Author: admin
 */
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>

void aw_delay_ms(uint32_t ms)
{
   Task_sleep((ms * 1000 )/ Clock_tickPeriod);
}

void aw_delay_us(uint32_t us){
    Task_sleep(us/ Clock_tickPeriod);
}



