/*
 * io_driver.h
 *
 *  Created on: 2020Äê8ÔÂ1ÈÕ
 *      Author: admin
 */

#ifndef IO_DRIVER_H_
#define IO_DRIVER_H_
#include <ti/drivers/PIN.h>
#include "driver_comm.h"
int8_t io_input_init(PIN_Handle *handle, PIN_State *state, const PIN_Config pinList[],
                     PIN_IntCb callbackFxn);
int8_t io_output_init(PIN_Handle *handle, PIN_State *state, const PIN_Config pinList[]);
#endif /* IO_DRIVER_H_ */
