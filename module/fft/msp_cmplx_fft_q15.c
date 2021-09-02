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

#include <stdint.h>
#include <stdbool.h>

#include "DSPLib.h"

#define STAGE1_STEP     (2)
#define STAGE2_STEP     (STAGE1_STEP*2)
#define STAGE3_STEP     (STAGE2_STEP*2)
#define STAGE4_STEP     (STAGE3_STEP*2)

/*
 * Abstracted radix-2 butterfly operations without fixed scaling by two.
 */
static inline void msp_cmplx_btfly_q15(int16_t *srcA, int16_t *srcB, const _q15 *coeff);
static inline void msp_cmplx_btfly_c0_q15(int16_t *srcA, int16_t *srcB);
static inline void msp_cmplx_btfly_c1_q15(int16_t *srcA, int16_t *srcB);

/*
 * Perform in-place radix-2 DFT of the input signal using an algorithm optimized
 * for MSP430 without fixed scaling by two at each stage.
 *
 * This optimized algorithm takes advantage of the fact that the first two
 * stages of the complex FFT will always have constants of +/- 0/1 and can be
 * reduced to add and subtract operations, removing a costly multiply operation.
 * This method greatly reduces execution time and energy consumption for small
 * complex FFT sizes.
 */
msp_status msp_cmplx_fft_q15(const msp_cmplx_fft_q15_params *params, int16_t *src)
{
    uint16_t i, j;                      // loop counters
    uint16_t ui16Length;                // src length
    uint16_t ui16Step;                  // step size
    uint16_t ui16TwidIndex;             // twiddle table index
    uint16_t ui16TwidIncrement;         // twiddle table increment
    int16_t *pi16Src;                   // temp source pointer
    const _q15 *pq15TwidTable;          // twiddle table pointer
    msp_status status;                  // Status of the operation
    
    /* Save input length to local. */
    ui16Length = params->length;
    
    /* Check that the provided table is the correct length. */
    if (*(uint16_t *)params->twiddleTable < ui16Length) {
        return MSP_TABLE_SIZE_ERROR;
    }
    
    /* Bit reverse the order of the inputs. */
    if(params->bitReverse) {
        /* Create and initialize a bit reversal params structure. */
        msp_cmplx_bitrev_q15_params paramsBitRev;
        paramsBitRev.length = params->length;
        paramsBitRev.bitReverseTable = params->bitReverseTable;
        
        /* Perform bit reversal on source data. */
        status = msp_cmplx_bitrev_q15(&paramsBitRev, src);
        
        /* Check if the operation was not successful. */
        if (status !=  MSP_SUCCESS) {
            return status;
        }
    }
    
    /* Stage 1. */
    if (STAGE1_STEP <= ui16Length) {
        for (j = 0; j < ui16Length; j += STAGE1_STEP) {
            pi16Src = src + j*2;
            msp_cmplx_btfly_c0_q15(&pi16Src[0], &pi16Src[0+STAGE1_STEP]);
        }
    }
    
    /* Stage 2. */
    if (STAGE2_STEP <= ui16Length) {
        for (j = 0; j < ui16Length; j += STAGE2_STEP) {
            pi16Src = src + j*2;
            msp_cmplx_btfly_c0_q15(&pi16Src[0], &pi16Src[0+STAGE2_STEP]);
            msp_cmplx_btfly_c1_q15(&pi16Src[2], &pi16Src[2+STAGE2_STEP]);
        }
    }
    
    /* Initialize step size, twiddle angle increment and twiddle table pointer. */
    ui16Step = STAGE3_STEP;
    ui16TwidIncrement = (*(uint16_t*)params->twiddleTable)/STAGE3_STEP;
    pq15TwidTable = &params->twiddleTable[DSPLIB_TABLE_OFFSET];
    
    /* If MPY32 is available save control context and set to fractional mode. */
#if defined(__MSP430_HAS_MPY32__)
    uint16_t ui16MPYState = MPY32CTL0;
    MPY32CTL0 = MPYFRAC;
#endif
    
    /* Stage 3 -> log2(ui16Step). */
    while (ui16Step <= ui16Length) {
        /* Reset the twiddle angle index. */
        ui16TwidIndex = 0;
        
        for (i = 0; i < (ui16Step/2); i++) {            
            /* Perform butterfly operations on complex pairs. */
            for (j = i; j < ui16Length; j += ui16Step) {
                pi16Src = src + j*2;
                msp_cmplx_btfly_q15(pi16Src, pi16Src + ui16Step, &pq15TwidTable[ui16TwidIndex]);
            }
            
            /* Increment twiddle table index. */
            ui16TwidIndex += ui16TwidIncrement;
        }
        /* Double the step size and halve the increment factor. */
        ui16Step *= 2;
        ui16TwidIncrement = ui16TwidIncrement/2;
    }
    
    /* Restore MPY32 control context. */
#if defined(__MSP430_HAS_MPY32__)
    MPY32CTL0 = ui16MPYState;
#endif
    
    return MSP_SUCCESS;
}

/*
 * Abstracted helper functions for a radix-2 butterfly operation. The following
 * operation is performed:
 *     A = A + coeff*B
 *     B = A - coeff*B
 */
static inline void msp_cmplx_btfly_q15(int16_t *srcA, int16_t *srcB, const _q15 *coeff)
{
    /* Load coefficients. */
    _q15 tempR = CMPLX_REAL(coeff);
    _q15 tempI = CMPLX_IMAG(coeff);
    
#if defined(__MSP430_HAS_MPY32__)
    /* Calculate real and imaginary parts of coeff*B. */
    MPYS = tempI;
    OP2 = CMPLX_REAL(srcB);
    MACS = tempR;
    OP2 = CMPLX_IMAG(srcB);
    MPYS = -tempI;
    tempI = RESHI;
    OP2 = CMPLX_IMAG(srcB);
    MACS = tempR;
    OP2 = CMPLX_REAL(srcB);
    tempR = RESHI;
#else
    /* Calculate real and imaginary parts of coeff*B. */
    tempR =  __mpyf_w(CMPLX_REAL(coeff), CMPLX_REAL(srcB));
    tempR -= __mpyf_w(CMPLX_IMAG(coeff), CMPLX_IMAG(srcB));
    tempI =  __mpyf_w(CMPLX_REAL(coeff), CMPLX_IMAG(srcB));
    tempI += __mpyf_w(CMPLX_IMAG(coeff), CMPLX_REAL(srcB));
#endif

    /* B = A - coeff*B */
    CMPLX_REAL(srcB) = CMPLX_REAL(srcA) - tempR;
    CMPLX_IMAG(srcB) = CMPLX_IMAG(srcA) - tempI;
    
    /* A = A + coeff*B */
    CMPLX_REAL(srcA) = CMPLX_REAL(srcA) + tempR;
    CMPLX_IMAG(srcA) = CMPLX_IMAG(srcA) + tempI;
}

/*
 * Simplified radix-2 butterfly operation for e^(-2*pi*(0/4)). This abstracted
 * helper function takes advantage of the fact the the twiddle coefficients are
 * positive and negative one for a multiplication by e^(-2*pi*(0/4)). The
 * following operation is performed:
 *     A = A + (1+0j)*B
 *     B = A - (1+0j)*B
 */
static inline void msp_cmplx_btfly_c0_q15(int16_t *srcA, int16_t *srcB)
{
    int16_t tempR = CMPLX_REAL(srcB);
    int16_t tempI = CMPLX_IMAG(srcB);
    
    /* B = A - (1+0j)*B */
    CMPLX_REAL(srcB) = CMPLX_REAL(srcA) - tempR;
    CMPLX_IMAG(srcB) = CMPLX_IMAG(srcA) - tempI;
    
    /* A = A + (1+0j)*B */
    CMPLX_REAL(srcA) = CMPLX_REAL(srcA) + tempR;
    CMPLX_IMAG(srcA) = CMPLX_IMAG(srcA) + tempI;
}

/*
 * Simplified radix-2 butterfly operation for e^(-2*pi*(1/4)). This abstracted
 * helper function takes advantage of the fact the the twiddle coefficients are
 * positive and negative one for a multiplication by e^(-2*pi*(1/4)). The
 * following operation is performed:
 *     A = A + (0-1j)*B
 *     B = A - (0-1j)*B
 */
static inline void msp_cmplx_btfly_c1_q15(int16_t *srcA, int16_t *srcB)
{
    int16_t tempR = CMPLX_REAL(srcB);
    int16_t tempI = CMPLX_IMAG(srcB);
    
    /* B = A - (0-1j)*B */
    CMPLX_REAL(srcB) = CMPLX_REAL(srcA) - tempI;
    CMPLX_IMAG(srcB) = CMPLX_IMAG(srcA) + tempR;
    
    /* A = A + (0-1j)*B */
    CMPLX_REAL(srcA) = CMPLX_REAL(srcA) + tempI;
    CMPLX_IMAG(srcA) = CMPLX_IMAG(srcA) - tempR;
}
