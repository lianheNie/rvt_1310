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

#ifndef __DSPLIB_VECTOR_H__
#define __DSPLIB_VECTOR_H__

/*!
 *  @brief Parameter structure for the vector add functions.
 */
typedef struct msp_add_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
} msp_add_q15_params;

/*!
 *  @brief Parameter structure for the vector subtract functions.
 */
typedef struct msp_sub_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
} msp_sub_q15_params;

/*!
 *  @brief Parameter structure for the vector multiply functions.
 */
typedef struct msp_mpy_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
} msp_mpy_q15_params;

/*!
 *  @brief Parameter structure for the vector multiply and accumulate function.
 */
typedef struct msp_mac_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
} msp_mac_q15_params;

/*!
 *  @brief Parameter structure for the vector negate function.
 */
typedef struct msp_neg_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
} msp_neg_q15_params;

/*!
 *  @brief Parameter structure for the vector absolute value function.
 */
typedef struct msp_abs_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
} msp_abs_q15_params;

/*!
 *  @brief Parameter structure for the vector offset function.
 */
typedef struct msp_offset_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
    _q15        offset;             /*!< Offset to add to each vector element. */
} msp_offset_q15_params;

/*!
 *  @brief Parameter structure for the vector scale function.
 */
typedef struct msp_scale_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
    _q15        scale;              /*!< Q15 fractional scale to multiply each vector element by. */
    uint8_t     shift;              /*!< Unsigned integer value to shift each vector result left by. */
} msp_scale_q15_params;

/*!
 *  @brief Parameter structure for the vector shift function.
 */
typedef struct msp_shift_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
    int8_t      shift;              /*!< Integer value to shift each vector element by. Positive values will shift left and negative values will shift right. */
} msp_shift_q15_params;

/*!
 *  @brief Parameter structure for the signed vector maximum function.
 */
typedef struct msp_max_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
} msp_max_q15_params;

/*!
 *  @brief Parameter structure for the unsigned vector maximum function.
 */
typedef struct msp_max_uq15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
} msp_max_uq15_params;

/*!
 *  @brief Parameter structure for the signed vector minimum function.
 */
typedef struct msp_min_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
} msp_min_q15_params;

/*!
 *  @brief Parameter structure for the unsigned vector minimum function.
 */
typedef struct msp_min_uq15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
} msp_min_uq15_params;

/*!
 *  @brief Parameter structure for the complex vector add functions.
 */
typedef struct msp_cmplx_add_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
} msp_cmplx_add_q15_params;

/*!
 *  @brief Parameter structure for the complex vector subtract functions.
 */
typedef struct msp_cmplx_sub_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
} msp_cmplx_sub_q15_params;

/*!
 *  @brief Parameter structure for the complex vector multiply functions.
 */
typedef struct msp_cmplx_mpy_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
} msp_cmplx_mpy_q15_params;

/*!
 *  @brief Parameter structure for the complex vector multiply and accumulate function.
 */
typedef struct msp_cmplx_mac_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
} msp_cmplx_mac_q15_params;

/*!
 *  @brief Parameter structure for the complex vector conjugate function.
 */
typedef struct msp_cmplx_conj_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
} msp_cmplx_conj_q15_params;

/*!
 *  @brief Parameter structure for the complex vector scale by real function.
 */
typedef struct msp_cmplx_scale_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
    _q15        scale;              /*!< Q15 real fractional scale to multiply each complex vector element by. */
    uint8_t     shift;              /*!< Unsigned integer value to shift each complex vector result left by. */
} msp_cmplx_scale_q15_params;

/*!
 *  @brief Parameter structure for the complex vector shift function.
 */
typedef struct msp_cmplx_shift_q15_params {
    uint16_t    length;             /*!< Length of the source and destination data. */
    int8_t      shift;              /*!< Integer value to shift each complex vector element by. Positive values will shift left and negative values will shift right. */
} msp_cmplx_shift_q15_params;

/*!
 *  @brief Element wise addition of two source vectors.
 *  @param params Pointer to the vector add parameter structure.
 *  @param srcA Pointer to the first source data vector.
 *  @param srcB Pointer to the second source data vector.
 *  @param dst Pointer to the destination data vector.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_add_q15(const msp_add_q15_params *params, const _q15 *srcA, const _q15 *srcB, _q15 *dst);

/*!
 *  @brief Element wise subtraction of two source vectors.
 *  @param params Pointer to the vector add parameter structure.
 *  @param srcA Pointer to the first source data vector.
 *  @param srcB Pointer to the second source data vector.
 *  @param dst Pointer to the destination data vector.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_sub_q15(const msp_sub_q15_params *params, const _q15 *srcA, const _q15 *srcB, _q15 *dst);

/*!
 *  @brief Element wise Q15 multiplication of two source vectors.
 *  @param params Pointer to the vector multiply parameter structure.
 *  @param srcA Pointer to the first source data vector.
 *  @param srcB Pointer to the second source data vector.
 *  @param dst Pointer to the destination data vector.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_mpy_q15(const msp_mpy_q15_params *params, const _q15 *srcA, const _q15 *srcB, _q15 *dst);

/*!
 *  @brief Element wise Q15 multiplication and accumulate (dot product) of two source vectors.
 *  @param params Pointer to the vector multiply and accumulate parameter structure.
 *  @param srcA Pointer to the first source data vector.
 *  @param srcB Pointer to the second source data vector.
 *  @param dst Pointer to the 32-bit destination data vector.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_mac_q15(const msp_mac_q15_params *params, const _q15 *srcA, const _q15 *srcB, _iq15 *result);

/*!
 *  @brief Element wise multiplication of a single source vector with negative one.
 *  @param params Pointer to the vector negate parameter structure.
 *  @param src Pointer to the source data vector.
 *  @param dst Pointer to the destination data vector.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_neg_q15(const msp_neg_q15_params *params, const _q15 *src, _q15 *dst);

/*!
 *  @brief Element wise absolute value of a single source vector.
 *  @param params Pointer to the vector absolute value parameter structure.
 *  @param src Pointer to the source data vector.
 *  @param dst Pointer to the destination data vector.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_abs_q15(const msp_abs_q15_params *params, const _q15 *src, _q15 *dst);

/*!
 *  @brief Element wise addition of a single source vector with an offset.
 *  @param params Pointer to the vector offset parameter structure.
 *  @param src Pointer to the source data vector.
 *  @param dst Pointer to the destination data vector.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_offset_q15(const msp_offset_q15_params *params, const _q15 *src, _q15 *dst);

/*!
 *  @brief Element wise Q15 multiplication of a single source vector with a Q15 scale value and shift left by scale.
 *  @param params Pointer to the vector scale structure.
 *  @param src Pointer to the source data vector.
 *  @param dst Pointer to the destination data vector.
 *  @return This function can return MSP_SUCCESS or MSP_SHIFT_SIZE_ERROR if an invalid shift parameter is given.
 */
extern msp_status msp_scale_q15(const msp_scale_q15_params *params, const _q15 *src, _q15 *dst);

/*!
 *  @brief Element wise left or right shift of a single source vector.
 *  @param params Pointer to the vector shift parameter structure.
 *  @param src Pointer to the source data vector.
 *  @param dst Pointer to the destination data vector.
 *  @return This function can return MSP_SUCCESS or MSP_SHIFT_SIZE_ERROR if an invalid shift parameter is given.
 */
extern msp_status msp_shift_q15(const msp_shift_q15_params *params, const _q15 *src, _q15 *dst);

/*!
 *  @brief Find the signed maximum value of a single source vector.
 *  @param params Pointer to the signed vector maximum parameter structure.
 *  @param src Pointer to the source data vector.
 *  @param max Pointer to the maximum result vector of size one.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_max_q15(const msp_max_q15_params *params, const _q15 *src, _q15 *max);

/*!
 *  @brief Find the unsigned maximum value of a single source vector.
 *  @param params Pointer to the unsigned vector maximum parameter structure.
 *  @param src Pointer to the source data vector.
 *  @param max Pointer to the maximum result vector of size one.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_max_uq15(const msp_max_uq15_params *params, const _uq15 *src, _uq15 *max);

/*!
 *  @brief Find the signed minimum value of a single source vector.
 *  @param params Pointer to the signed vector minimum parameter structure.
 *  @param src Pointer to the source data vector.
 *  @param min Pointer to the minimum result vector of size one.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_min_q15(const msp_min_q15_params *params, const _q15 *src, _q15 *min);

/*!
 *  @brief Find the unsigned minimum value of a single source vector.
 *  @param params Pointer to the unsigned vector minimum parameter structure.
 *  @param src Pointer to the source data vector.
 *  @param min Pointer to the minimum result vector of size one.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_min_uq15(const msp_min_uq15_params *params, const _uq15 *src, _uq15 *min);

/*!
 *  @brief Element wise addition of two complex source vectors without saturation.
 *  @param params Pointer to the complex vector add parameter structure.
 *  @param srcA Pointer to the first complex source data vector.
 *  @param srcB Pointer to the second complex source data vector.
 *  @param dst Pointer to the complex destination data vector.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_cmplx_add_q15(const msp_cmplx_add_q15_params *params, const _q15 *srcA, const _q15 *srcB, _q15 *dst);

/*!
 *  @brief Element wise addition of two complex source vectors with saturation.
 *  @param params Pointer to the complex vector add parameter structure.
 *  @param srcA Pointer to the first complex source data vector.
 *  @param srcB Pointer to the second complex source data vector.
 *  @param dst Pointer to the complex destination data vector.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_cmplx_add_sat_q15(const msp_cmplx_add_q15_params *params, const _q15 *srcA, const _q15 *srcB, _q15 *dst);

/*!
 *  @brief Element wise subtraction of two complex source vectors without saturation.
 *  @param params Pointer to the complex vector add parameter structure.
 *  @param srcA Pointer to the first complex source data vector.
 *  @param srcB Pointer to the second complex source data vector.
 *  @param dst Pointer to the complex destination data vector.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_cmplx_sub_q15(const msp_cmplx_sub_q15_params *params, const _q15 *srcA, const _q15 *srcB, _q15 *dst);

/*!
 *  @brief Element wise subtraction of two complex source vectors with saturation.
 *  @param params Pointer to the complex vector add parameter structure.
 *  @param srcA Pointer to the first complex source data vector.
 *  @param srcB Pointer to the second complex source data vector.
 *  @param dst Pointer to the complex destination data vector.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_cmplx_sub_sat_q15(const msp_cmplx_sub_q15_params *params, const _q15 *srcA, const _q15 *srcB, _q15 *dst);


/*!
 *  @brief Element wise Q15 multiplication of two complex source vectors.
 *  @param params Pointer to the complex vector multiply parameter structure.
 *  @param srcA Pointer to the first complex source data vector.
 *  @param srcB Pointer to the second complex source data vector.
 *  @param dst Pointer to the complex destination data vector.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_cmplx_mpy_q15(const msp_cmplx_mpy_q15_params *params, const _q15 *srcA, const _q15 *srcB, _q15 *dst);

/*!
 *  @brief Element wise Q15 multiplication of a complex source vector with a real source vector.
 *  @param params Pointer to the complex vector multiply parameter structure.
 *  @param srcCmplx Pointer to the complex source data vector.
 *  @param srcReal Pointer to the real source data vector.
 *  @param dst Pointer to the complex destination data vector.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_cmplx_mpy_real_q15(const msp_cmplx_mpy_q15_params *params, const _q15 *srcCmplx, const _q15 *srcReal, _q15 *dst);

/*!
 *  @brief Element wise Q15 multiplication and accumulate (dot product) of two complex source vectors.
 *  @param params Pointer to the complex vector multiply and accumulate parameter structure.
 *  @param srcA Pointer to the first complex source data vector.
 *  @param srcB Pointer to the second complex source data vector.
 *  @param dst Pointer to the 32-bit complex destination data vector.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_cmplx_mac_q15(const msp_cmplx_mac_q15_params *params, const _q15 *srcA, const _q15 *srcB, _iq15 *result);

/*!
 *  @brief Element wise complex conjugate of a single complex source vector.
 *  @param params Pointer to the complex vector conjugate parameter structure.
 *  @param src Pointer to the complex source data vector.
 *  @param dst Pointer to the complex destination data vector.
 *  @return This function will always return MSP_SUCCESS.
 */
extern msp_status msp_cmplx_conj_q15(const msp_cmplx_conj_q15_params *params, const _q15 *src, _q15 *dst);

/*!
 *  @brief Element wise Q15 multiplication of a single complex source vector with a Q15 real scale value and shift left by scale.
 *  @param params Pointer to the complex vector scale structure.
 *  @param src Pointer to the complex source data vector.
 *  @param dst Pointer to the complex destination data vector.
 *  @return This function can return MSP_SUCCESS or MSP_SHIFT_SIZE_ERROR if an invalid shift parameter is given.
 */
extern msp_status msp_cmplx_scale_q15(const msp_cmplx_scale_q15_params *params, const _q15 *src, _q15 *dst);

/*!
 *  @brief Element wise left or right shift of a single complex source vector.
 *  @param params Pointer to the complex vector shift parameter structure.
 *  @param src Pointer to the complex source data vector.
 *  @param dst Pointer to the complex destination data vector.
 *  @return This function can return MSP_SUCCESS or MSP_SHIFT_SIZE_ERROR if an invalid shift parameter is given.
 */
extern msp_status msp_cmplx_shift_q15(const msp_cmplx_shift_q15_params *params, const _q15 *src, _q15 *dst);

#endif //__DSPLIB_VECTOR_H__
