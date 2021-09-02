/*
 * util_timer.c
 *
 *  Created on: 2020Äê8ÔÂ1ÈÕ
 *      Author: admin
 */
#include <string.h>

#include <stdbool.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "util_timer.h"

/******************************************************************************
 Constants and Typedefs
 *****************************************************************************/

/*! Adjustment for the timers */
#define TIMER_MS_ADJUSTMENT     100

/* RTOS queue for profile/app messages. */
typedef struct _queueRec_
{
    Queue_Elem _elem;    /* queue element */
    uint8_t *pData;      /* pointer to app data */
} queueRec_t;

/******************************************************************************
 Public Functions
 *****************************************************************************/

/*!
 Initialize a TIRTOS Timer/Clock instance.

 Public function defined in mac_util.h
 */
Clock_Handle Timer_construct(Clock_Struct *pClock, Clock_FuncPtr clockCB,
                                 uint32_t clockDuration, uint32_t clockPeriod,
                                 uint8_t startFlag,
                                 UArg arg)
{
    Clock_Params clockParams;

    /* Convert clockDuration in milliseconds to ticks. */
    uint32_t clockTicks = clockDuration * (1000 / Clock_tickPeriod);

    /* Setup parameters. */
    Clock_Params_init(&clockParams);

    /* Setup argument. */
    clockParams.arg = arg;

    /* If period is 0, this is a one-shot timer. */
    clockParams.period = clockPeriod * (1000 / Clock_tickPeriod);

    /*
     Starts immediately after construction if true, otherwise wait for a
     call to start.
     */
    clockParams.startFlag = startFlag;

    /*/ Initialize clock instance. */
    Clock_construct(pClock, clockCB, clockTicks, &clockParams);

    return Clock_handle(pClock);
}

/*!
 Start a timer/clock.

 Public function defined in mac_util.h
 */
void Timer_start(Clock_Struct *pClock)
{
    Clock_Handle handle = Clock_handle(pClock);

    /* Start clock instance */
    Clock_start(handle);
}

/*!
 Determine if a timer/clock is currently active.

 Public function defined in mac_util.h
 */
bool Timer_isActive(Clock_Struct *pClock)
{
    Clock_Handle handle = Clock_handle(pClock);

    /* Start clock instance */
    return Clock_isActive(handle);
}

/*!
 Stop a timer/clock.

 Public function defined in mac_util.h
 */
void Timer_stop(Clock_Struct *pClock)
{
    Clock_Handle handle = Clock_handle(pClock);

    /* Start clock instance */
    Clock_stop(handle);
}

/*!
 * @brief   Set a Timer/Clock timeout.
 *
 * @param   timeOut - Timeout value in milliseconds
 */
void Timer_setTimeout(Clock_Handle handle, uint32_t timeout)
{
    Clock_setTimeout(handle, (timeout * TIMER_MS_ADJUSTMENT));
}

/*!
 * @brief   Get a Timer/Clock timeout.
 *
 * @param   handle - clock handle
 *
 * @return   timeOut - Timeout value in milliseconds
 */
uint32_t Timer_getTimeout(Clock_Handle handle)
{
    uint32_t timeout;

    timeout = Clock_getTimeout(handle);

    return (timeout / TIMER_MS_ADJUSTMENT);
}



