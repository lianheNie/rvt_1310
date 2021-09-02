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

#ifndef __DSPLIB_SUPPORT_H__
#define __DSPLIB_SUPPORT_H__

#include <stdint.h>
#include <stdbool.h>

/*!
 *  @brief Offset used to store table size.
 */
#define DSPLIB_TABLE_OFFSET     2

/*!
 *  @brief Increment size for complex data.
 */
#define CMPLX_INCREMENT (2)

/*!
 *  @brief Access the real portion of complex data.
 */
#define CMPLX_REAL(ptr) ((ptr)[0])

/*!
 *  @brief Access the imaginary portion of complex data.
 */
#define CMPLX_IMAG(ptr) ((ptr)[1])

#ifdef __USE_IQMATHLIB__ // Provide definition for the hidden _IQ31mpy function.

extern int32_t _IQ31mpy(int32_t A, int32_t B);

/*!
 *  @brief Fractional Q15 multiply function with 16-bit arguments and results.
 */
#define __mpyf_w(A, B)  _Q15mpy((A), (B))

/*!
 *  @brief Fractional Q31 multiply function with 32-bit arguments and results.
 */
#define __mpyf_l(A, B)  _IQ31mpy((A), (B))

#else
/*!
 *  @brief Fractional Q15 multiply function with 16-bit arguments and results.
 */
#define __mpyf_w(A, B)  ((int16_t)(((int32_t)(A) * (int32_t)(B)) >> 15))

/*!
 *  @brief Fractional Q31 multiply function with 32-bit arguments and results.
 */
#define __mpyf_l(A, B)  ((int32_t)(((int64_t)(A) * (int64_t)(B)) >> 31))

#endif //__USE_IQMATHLIB__

#endif //__DSPLIB_SUPPORT_H__
