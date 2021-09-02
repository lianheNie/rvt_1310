/*
 * util_timer.h
 *
 *  Created on: 2020Äê8ÔÂ1ÈÕ
 *      Author: admin
 */

#ifndef UTIL_TIMER_H_
#define UTIL_TIMER_H_
/******************************************************************************
 Includes
 *****************************************************************************/
#include <ti/sysbios/knl/Clock.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

/*!
 \defgroup Timer Functions
 <BR>
 Timer/Clock functions.
 <BR>
 */

/******************************************************************************
 Constants and definitions
 *****************************************************************************/

/*!
 * \ingroup TimerClock
 * @{
 */

/******************************************************************************
 Function Prototypes
 *****************************************************************************/

/*!
 * @brief   Initialize a TIRTOS Clock instance.
 *
 * @param   pClock        - pointer to clock instance structure.
 * @param   clockCB       - callback function upon clock expiration.
 * @param   clockDuration - longevity of clock timer in milliseconds
 * @param   clockPeriod   - duration of a periodic clock, used continuously
 *                          after clockDuration expires.
 * @param   startFlag     - TRUE to start immediately, FALSE to wait.
 * @param   arg           - argument passed to callback function.
 *
 * @return  Clock_Handle  - a handle to the clock instance.
 */
extern Clock_Handle Timer_construct(Clock_Struct *pClock,
                                        Clock_FuncPtr clockCB,
                                        uint32_t clockDuration,
                                        uint32_t clockPeriod,
                                        uint8_t startFlag,
                                        UArg arg);

/*!
 * @brief   Start a Timer/Clock.
 *
 * @param   pClock - pointer to clock struct
 */
extern void Timer_start(Clock_Struct *pClock);

/*!
 * @brief   Determine if a clock is currently active.
 *
 * @param   pClock - pointer to clock struct
 *
 * @return  TRUE or FALSE
 */
extern bool Timer_isActive(Clock_Struct *pClock);

/*!
 * @brief   Stop a Timer/Clock.
 *
 * @param   pClock - pointer to clock struct
 */
extern void Timer_stop(Clock_Struct *pClock);

/*!
 * @brief   Set a Timer/Clock timeout.
 *
 * @param   timeOut - Timeout value in milliseconds
 */
extern void Timer_setTimeout(Clock_Handle handle, uint32_t timeout);

/*!
 * @brief   Get a Timer/Clock timeout.
 *
 * @param   handle - clock handle
 *
 * @return   timeOut - Timeout value in milliseconds
 */
extern uint32_t Timer_getTimeout(Clock_Handle handle);

/*! @} end group TimerClock */

#ifdef __cplusplus
}
#endif





#endif /* UTIL_TIMER_H_ */
