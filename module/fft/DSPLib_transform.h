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

#ifndef __DSPLIB_TRANSFORM_H__
#define __DSPLIB_TRANSFORM_H__

/*!
 *  @brief Parameter structure for _q15 complex bit-reversal.
 */
typedef struct msp_cmplx_bitrev_q15_params {
    uint16_t                length;             /*!< Length of the source data. */
    const uint16_t          *bitReverseTable;   /*!< Pointer to the bit-reversal table descriptor to use with size greater than or equal to source length. */
} msp_cmplx_bitrev_q15_params;

/*!
 *  @brief Parameter structure for _q15 real FFT functions.
 */
typedef struct msp_fft_q15_params {
    uint16_t                length;             /*!< Length of the source data. */
    bool                    bitReverse;         /*!< Perform bit-reversal of input first. */
    const uint16_t          *bitReverseTable;   /*!< Pointer to the bit-reversal table descriptor to use with size greater than or equal to half the source length. This is not required if bitReverse is false. */
    const _q15              *twiddleTable;      /*!< Pointer to the twiddle coefficient table descriptor to use with size greater than or equal to half the source length. */
    const _q15              *splitTable;        /*!< Pointer to the split coefficient table with size greater than or equal to the source length. */
} msp_fft_q15_params;

/*!
 *  @brief Parameter structure for _q15 real inverse FFT functions.
 */
typedef struct msp_ifft_q15_params {
    uint16_t                length;             /*!< Length of the source data. */
    bool                    bitReverse;         /*!< Perform bit-reversal of input first. */
    const uint16_t          *bitReverseTable;   /*!< Pointer to the bit-reversal table descriptor to use with size greater than or equal to half the source length. This is not required if bitReverse is false. */
    const _q15              *twiddleTable;      /*!< Pointer to the twiddle coefficient table descriptor to use with size greater than or equal to half the source length. */
    const _q15              *splitTable;        /*!< Pointer to the split coefficient table with size greater than or equal to the source length. */
} msp_ifft_q15_params;

/*!
 *  @brief Parameter structure for _q15 complex FFT functions.
 */
typedef struct msp_cmplx_fft_q15_params {
    uint16_t                length;             /*!< Length of the source data. */
    bool                    bitReverse;         /*!< Perform bit-reversal of input first. */
    const uint16_t          *bitReverseTable;   /*!< Pointer to the bit-reversal table descriptor to use with size greater than or equal to source length. This is not required if bitReverse is false. */
    const _q15              *twiddleTable;      /*!< Pointer to the complex twiddle coefficient table descriptor to use with size greater than or equal to source length. */
} msp_cmplx_fft_q15_params;

/*!
 *  @brief Parameter structure for _q15 complex inverse FFT functions.
 */
typedef struct msp_cmplx_ifft_q15_params {
    uint16_t                length;             /*!< Length of the source data. */
    bool                    bitReverse;         /*!< Perform bit-reversal of input first. */
    const uint16_t          *bitReverseTable;   /*!< Pointer to the bit-reversal table descriptor to use with size greater than or equal to source length. This is not required if bitReverse is false. */
    const _q15              *twiddleTable;      /*!< Pointer to the complex twiddle coefficient table descriptor to use with size greater than or equal to source length. */
} msp_cmplx_ifft_q15_params;

/*!
 *  @brief Parameter structure for performing the split operation.
 */
typedef struct msp_split_q15_params {
    uint16_t                length;             /*!< Length of the source data. */
    const _q15              *splitTable;        /*!< Pointer to the split coefficient table with size greater than or equal to the source length. */
} msp_split_q15_params;

/*!
 *  @brief Parameter structure for performing the inverse split operation.
 */
typedef struct msp_isplit_q15_params {
    uint16_t                length;             /*!< Length of the source data. */
    const _q15              *splitTable;        /*!< Pointer to the split coefficient table with size greater than or equal to the source length. */
} msp_isplit_q15_params;

/*!
 *  @brief Complex bit-reversal function.
 *  @param params Pointer to the complex bit-reversal parameter structure.
 *  @param src Pointer to the complex data array to perform the bit-reversal on.
 *  @return This function can return MSP_SUCCESS or MSP_TABLE_SIZE_ERROR if one of the provided tables is too small.
 */
extern msp_status msp_cmplx_bitrev_q15(const msp_cmplx_bitrev_q15_params *params, _q15 *src);

/*!
 *  @brief Real FFT function without fixed scaling at each stage.
 *  @param params Pointer to the real FFT parameter structure.
 *  @param src Pointer to the real data array to perform the FFT on.
 *  @return This function can return MSP_SUCCESS or MSP_TABLE_SIZE_ERROR if one of the provided tables is too small.
 */
extern msp_status msp_fft_q15(const msp_fft_q15_params *params, _q15 *src);

/*!
 *  @brief Real FFT function with fixed scaling by two at each stage.
 *  @param params Pointer to the real FFT parameter structure.
 *  @param src Pointer to the real data array to perform the FFT on.
 *  @return This function can return MSP_SUCCESS or MSP_TABLE_SIZE_ERROR if one of the provided tables is too small.
 */
extern msp_status msp_fft_scale_q15(const msp_fft_q15_params *params, _q15 *src);

/*!
 *  @brief Real inverse FFT function without fixed scaling at each stage.
 *  @param params Pointer to the real inverse FFT parameter structure.
 *  @param src Pointer to the real data array to perform the inverse FFT on.
 *  @return This function can return MSP_SUCCESS or MSP_TABLE_SIZE_ERROR if one of the provided tables is too small.
 */
extern msp_status msp_ifft_q15(const msp_ifft_q15_params *params, _q15 *src);

/*!
 *  @brief Real FFT function with fixed scaling by two at each stage.
 *  @param params Pointer to the real inverse FFT parameter structure.
 *  @param src Pointer to the real data array to perform the inverse FFT on.
 *  @return This function can return MSP_SUCCESS or MSP_TABLE_SIZE_ERROR if one of the provided tables is too small.
 */
extern msp_status msp_ifft_scale_q15(const msp_ifft_q15_params *params, _q15 *src);

/*!
 *  @brief Complex FFT function without fixed scaling at each stage.
 *  @param params Pointer to the complex FFT parameter structure.
 *  @param src Pointer to the complex data array to perform the FFT on.
 *  @return This function can return MSP_SUCCESS or MSP_TABLE_SIZE_ERROR if one of the provided tables is too small.
 */
extern msp_status msp_cmplx_fft_q15(const msp_cmplx_fft_q15_params *params, _q15 *src);

/*!
 *  @brief Complex FFT function with fixed scaling by two at each stage.
 *  @param params Pointer to the complex FFT parameter structure.
 *  @param src Pointer to the complex data array to perform the FFT on.
 *  @return This function can return MSP_SUCCESS or MSP_TABLE_SIZE_ERROR if one of the provided tables is too small.
 */
extern msp_status msp_cmplx_fft_scale_q15(const msp_cmplx_fft_q15_params *params, _q15 *src);

/*!
 *  @brief Complex inverse FFT function without fixed scaling at each stage.
 *  @param params Pointer to the complex inverse FFT parameter structure.
 *  @param src Pointer to the complex data array to perform the inverse FFT on.
 *  @return This function can return MSP_SUCCESS or MSP_TABLE_SIZE_ERROR if one of the provided tables is too small.
 */
extern msp_status msp_cmplx_ifft_q15(const msp_cmplx_ifft_q15_params *params, _q15 *src);

/*!
 *  @brief Complex inverse FFT function with fixed scaling by two at each stage.
 *  @param params Pointer to the complex inverse FFT parameter structure.
 *  @param src Pointer to the complex data array to perform the inverse FFT on.
 *  @return This function can return MSP_SUCCESS or MSP_TABLE_SIZE_ERROR if one of the provided tables is too small.
 */
extern msp_status msp_cmplx_ifft_scale_q15(const msp_cmplx_ifft_q15_params *params, _q15 *src);

/*!
 *  @brief Split operation for performing the final step of a real FFT.
 *  @param src Pointer to the data array to perform the split operation on.
 *  @param length Length of the data array.
 *  @param splitTable Pointer to the split operation table descriptor.
 *  @return This function can return MSP_SUCCESS or MSP_TABLE_SIZE_ERROR if one of the provided tables is too small.
 */
extern msp_status msp_split_q15(const msp_split_q15_params *params, _q15 *src);

/*!
 *  @brief Inverse split operation for performing the final step of a real inverse FFT.
 *  @param src Pointer to the data array to perform the inverse split operation on.
 *  @param length Length of the data array.
 *  @param splitTable Pointer to the split operation table descriptor.
 *  @return This function can return MSP_SUCCESS or MSP_TABLE_SIZE_ERROR if one of the provided tables is too small.
 */
extern msp_status msp_isplit_q15(const msp_isplit_q15_params *params, _q15 *src);

/*!
 *  @brief Bit reversal lookup table for size length 16.
 */
extern const uint16_t msp_cmplx_bitrev_table_16_ui16[DSPLIB_TABLE_OFFSET+16];

/*!
 *  @brief Bit reversal lookup table for size length 32.
 */
extern const uint16_t msp_cmplx_bitrev_table_32_ui16[DSPLIB_TABLE_OFFSET+32];

/*!
 *  @brief Bit reversal lookup table for size length 64.
 */
extern const uint16_t msp_cmplx_bitrev_table_64_ui16[DSPLIB_TABLE_OFFSET+64];

/*!
 *  @brief Bit reversal lookup table for size length 128.
 */
extern const uint16_t msp_cmplx_bitrev_table_128_ui16[DSPLIB_TABLE_OFFSET+128];

/*!
 *  @brief Bit reversal lookup table for size length 256.
 */
extern const uint16_t msp_cmplx_bitrev_table_256_ui16[DSPLIB_TABLE_OFFSET+256];

/*!
 *  @brief Bit reversal lookup table for size length 512.
 */
extern const uint16_t msp_cmplx_bitrev_table_512_ui16[DSPLIB_TABLE_OFFSET+512];

/*!
 *  @brief Bit reversal lookup table for size length 1024.
 */
extern const uint16_t msp_cmplx_bitrev_table_1024_ui16[DSPLIB_TABLE_OFFSET+1024];

/*!
 *  @brief Bit reversal lookup table for size length 2048.
 */
extern const uint16_t msp_cmplx_bitrev_table_2048_ui16[DSPLIB_TABLE_OFFSET+2048];

/*!
 *  @brief Bit reversal lookup table for size length 4096.
 */
extern const uint16_t msp_cmplx_bitrev_table_4096_ui16[DSPLIB_TABLE_OFFSET+4096];

/*!
*  @brief Twiddle factor table for FFT of size 16.
*/
extern const _q15 msp_cmplx_twiddle_table_16_q15[DSPLIB_TABLE_OFFSET+16];

/*!
*  @brief Twiddle factor table for FFT of size 32.
*/
extern const _q15 msp_cmplx_twiddle_table_32_q15[DSPLIB_TABLE_OFFSET+32];

/*!
*  @brief Twiddle factor table for FFT of size 64.
*/
extern const _q15 msp_cmplx_twiddle_table_64_q15[DSPLIB_TABLE_OFFSET+64];

/*!
*  @brief Twiddle factor table for FFT of size 128.
*/
extern const _q15 msp_cmplx_twiddle_table_128_q15[DSPLIB_TABLE_OFFSET+128];

/*!
*  @brief Twiddle factor table for FFT of size 256.
*/
extern const _q15 msp_cmplx_twiddle_table_256_q15[DSPLIB_TABLE_OFFSET+256];

/*!
*  @brief Twiddle factor table for FFT of size 512.
*/
extern const _q15 msp_cmplx_twiddle_table_512_q15[DSPLIB_TABLE_OFFSET+512];

/*!
*  @brief Twiddle factor table for FFT of size 1024.
*/
extern const _q15 msp_cmplx_twiddle_table_1024_q15[DSPLIB_TABLE_OFFSET+1024];

/*!
*  @brief Twiddle factor table for FFT of size 2048.
*/
extern const _q15 msp_cmplx_twiddle_table_2048_q15[DSPLIB_TABLE_OFFSET+2048];

/*!
*  @brief Twiddle factor table for FFT of size 4096.
*/
extern const _q15 msp_cmplx_twiddle_table_4096_q15[DSPLIB_TABLE_OFFSET+4096];

/*!
*  @brief Split coefficient table for real FFT of size 16.
*/
extern const _q15 msp_split_table_16_q15[DSPLIB_TABLE_OFFSET+8];

/*!
*  @brief Split coefficient table for real FFT of size 32.
*/
extern const _q15 msp_split_table_32_q15[DSPLIB_TABLE_OFFSET+16];

/*!
*  @brief Split coefficient table for real FFT of size 64.
*/
extern const _q15 msp_split_table_64_q15[DSPLIB_TABLE_OFFSET+32];

/*!
*  @brief Split coefficient table for real FFT of size 128.
*/
extern const _q15 msp_split_table_128_q15[DSPLIB_TABLE_OFFSET+64];

/*!
*  @brief Split coefficient table for real FFT of size 256.
*/
extern const _q15 msp_split_table_256_q15[DSPLIB_TABLE_OFFSET+128];

/*!
*  @brief Split coefficient table for real FFT of size 512.
*/
extern const _q15 msp_split_table_512_q15[DSPLIB_TABLE_OFFSET+256];

/*!
*  @brief Split coefficient table for real FFT of size 1024.
*/
extern const _q15 msp_split_table_1024_q15[DSPLIB_TABLE_OFFSET+512];

/*!
*  @brief Split coefficient table for real FFT of size 2048.
*/
extern const _q15 msp_split_table_2048_q15[DSPLIB_TABLE_OFFSET+1024];

/*!
*  @brief Split coefficient table for real FFT of size 4096.
*/
extern const _q15 msp_split_table_4096_q15[DSPLIB_TABLE_OFFSET+2048];

#endif //__DSPLIB_TRANSFORM_H__
