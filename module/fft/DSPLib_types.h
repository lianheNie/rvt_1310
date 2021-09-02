/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
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
 * --/COPYRIGHT--*/

#ifndef __DSPLIB_TYPES_H__
#define __DSPLIB_TYPES_H__

#include <stdint.h>
#include <stdbool.h>

#ifndef __QMATHLIB_H__ // Define the _q15 type if it is not defined by the QmathLib header file.

/*!
 *  @brief Signed fixed point data type with 1 integer bit and 15 fractional bits.
 */
typedef int16_t _q15;

#endif  //__QMATHLIB_H__

#ifndef __IQMATHLIB_H__ // Define the _iq15 type if it is not defined by the IQmathLib header file.

/*!
 *  @brief Signed fixed point data type with 17 integer bits and 15 fractional bits.
 */
typedef int32_t _iq15;

#endif  //__QMATHLIB_H__

/*!
 *  @brief Signed fixed point data type with 1 integer bit and 31 fractional bits.
 */
typedef int32_t _q31;

/*!
 *  @brief Signed fixed point data type with 33 integer bits and 31 fractional bits.
 */
typedef int64_t _iiq31;

/*!
 *  @brief Unsigned fixed point data type with 1 integer bit and 15 fractional bits.
 */
typedef uint16_t _uq15;

/*!
 *  @brief Unsigned fixed point data type with 1 integer bit and 31 fractional bits.
 */
typedef uint32_t _uq31;

/*!
 *  @brief Enumerated type to return the status of an operation.
 */
typedef enum {
    MSP_SUCCESS,            /*!< Successful operation. */
    MSP_SIZE_ERROR,         /*!< Invalid size, see API for restrictions. */
    MSP_SHIFT_SIZE_ERROR,   /*!< Invalid shift size, see API for restrictions. */
    MSP_TABLE_SIZE_ERROR    /*!< Invalid table size, see API for restrictions. */
} msp_status;

#endif //__DSPLIB_TYPES_H__
