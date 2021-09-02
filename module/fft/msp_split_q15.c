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

/*
 * This function performs the last stage of a real FFT of size M in place.
 * Before this step is performed, a size M/2 complex FFT must be performed
 * on the input data with bit reversal. This algorithm has been optimized
 * to use a single coefficient lookup table and a reduced number of multiply
 * operations. The complex result G(k) where k = 0,1,2...N-1 and N = M/2 is
 * calculated as follows:
 *
 *     G(k) = 0.5*(X(k) + X*(N-k)) - 0.5*j*(e^-j2*pi*k/2N)*(X(k) - X*(N-k))
 *
 * This can be optimized by calculating G(N-k) in parallel using the following
 * simplification:
 *
 *     G(N-k) = 0.5*(X(N-k) + X*(N-(N-k))) - 0.5*j*(e^-j2*pi*(N-k)/2N)*(X(N-k) - X*(N-(N-k)))
 *     G(N-k) = 0.5*(X(N-k) + X*(k)) - 0.5*j*(e^-j2*pi*(N-k)/2N)*(X(N-k) - X*(k))
 *     G(N-k) = 0.5*(X(N-k) + X*(k)) - 0.5*j*-(e^-j2*pi*(N-k)/2N)*(X*(k) - X(N-k))
 *     G(N-k) = 0.5*(X(k) + X*(N-k))* - 0.5*j*((e^-j2*pi*k/2N)*(X(k) - X*(N-k)))*
 *
 * The common components of G(k) and G(N-k) can be calculated once as local
 * variables A and B:
 *
 *     A(k) = 0.5*(X(k) + X*(N-k))
 *     B(k) = 0.5*(e^-j2*pi*k/2N)*(X(k) - X*(N-k))
 *
 * Finally, performing substitutions to the G(k) and G(N-k) equations yields
 * the following optimized equations:
 * 
 *     G(k) = A(k) - j*B(k)
 *     G(N-k) = A*(k) - j*B*(k)
 * 
 * Reference: http://www.ti.com/lit/an/spra291/spra291.pdf
 */
msp_status msp_split_q15(const msp_split_q15_params *params, _q15 *src)
{
    uint16_t ui16Length;            // src length
    uint16_t ui16TableLen;          // Coefficient table length
    uint16_t ui16CoeffOffset;       // Coefficient table increment
    int16_t i16Xr;                  // Temp variable
    int16_t i16Xi;                  // Temp variable
    int16_t i16BrTmp;               // Temp variable
    int16_t i16BiTmp;               // Temp variable
    int16_t i16ArTmp;               // Temp variable
    int16_t i16AiTmp;               // Temp variable
    int16_t *pi16SrcK;              // Source pointer to X(k)
    int16_t *pi16SrcNK;             // Source pointer to X(N-k)
    const int16_t *pi16Coeff;       // Coefficient pointer
    
    /* Check that the provided table is the correct length. */
    if (*(uint16_t *)params->splitTable < params->length) {
        return MSP_TABLE_SIZE_ERROR;
    }
    
    /* 
     * Calculate the first result bin (DC component).
     *
     *     X(N) = X(0)
     *     G(0) = 0.5*(X(0) + X*(0)) - j*0.5*(e^-j*0)*(X(0) - X*(0))
     *     G(0) = Xr(0) + Xi(0)
     */
    CMPLX_REAL(src) = CMPLX_REAL(src) + CMPLX_IMAG(src);
    CMPLX_IMAG(src) = 0;
    
    /* Initialize Src(k) and Src(N/2-k) pointers when k=1. */
    pi16SrcK = src + CMPLX_INCREMENT;
    pi16SrcNK = src + params->length - CMPLX_INCREMENT;
    
    /* Calculate coefficient table offset. */
    ui16Length = params->length;
    ui16CoeffOffset = 2;
    ui16TableLen = *(uint16_t *)params->splitTable;
    while (ui16Length < ui16TableLen) {
        ui16CoeffOffset *= 2;
        ui16Length *= 2;
    }
    
    /* Initialize coefficient pointer to index k=1. */
    pi16Coeff = &params->splitTable[DSPLIB_TABLE_OFFSET] + ui16CoeffOffset;
    
    /*
     * Initialize length of split operations to perform. G(k) and G(N/2-k) are
     * calculated in the same loop iteration so only half of the N/2 iterations
     * are required, N/4. The last iteration where k = N/2-k will be calculated
     * separately.
     */
    ui16Length = (params->length >> 2) - 1;
    
    /* If MPY32 is available save control context and set to fractional mode. */
#if defined(__MSP430_HAS_MPY32__)
    uint16_t ui16MPYState = MPY32CTL0;
    MPY32CTL0 = MPYFRAC;
#endif
    
    /* Loop through and perform all of the split operations. */
    while(ui16Length--) {
        /* Calculate X(k) - X*(N-k) to local temporary variables. */
        i16Xr = CMPLX_REAL(pi16SrcK) - CMPLX_REAL(pi16SrcNK);
        i16Xi = CMPLX_IMAG(pi16SrcK) + CMPLX_IMAG(pi16SrcNK);
        
#if defined(__MSP430_HAS_MPY32__)
        /* Br(k) = RE{0.5*(e^-j2*pi*k/2N)*(Xr(k) - Xr(N-k))} */
        MPYS = i16Xr;
        OP2 = CMPLX_REAL(pi16Coeff);
        MACS = i16Xi;
        OP2 = -CMPLX_IMAG(pi16Coeff);
        i16BrTmp = RESHI;
        
        /* Bi(k) = IM{0.5*(e^-j2*pi*k/2N)*(Xi(k) + Xi(N-k))} */
        MPYS = i16Xr;
        OP2 = CMPLX_IMAG(pi16Coeff);
        MACS = i16Xi;
        OP2 = CMPLX_REAL(pi16Coeff);
        i16BiTmp = RESHI;
#else
        /* Br(k) = RE{0.5*(e^-j2*pi*k/2N)}*(Xr(k) - Xr(N-k)) */
        i16BrTmp =  __mpyf_w(i16Xr, CMPLX_REAL(pi16Coeff));
        i16BrTmp -= __mpyf_w(i16Xi, CMPLX_IMAG(pi16Coeff));
        
        /* Bi(k) = IM{0.5*(e^-j2*pi*k/2N)}*(Xi(k) + Xi(N-k)) */
        i16BiTmp =  __mpyf_w(i16Xr, CMPLX_IMAG(pi16Coeff));
        i16BiTmp += __mpyf_w(i16Xi, CMPLX_REAL(pi16Coeff));
#endif
        
        /*
         * Ar(k) = 0.5*(Xr(k) + Xr(N-k))
         * Ai(k) = 0.5*(Xi(k) - Xi(N-k))
         */
        i16ArTmp = (CMPLX_REAL(pi16SrcK) + CMPLX_REAL(pi16SrcNK)) >> 1;
        i16AiTmp = (CMPLX_IMAG(pi16SrcK) - CMPLX_IMAG(pi16SrcNK)) >> 1;
        
        /*
         * Gr(k) = Ar(k) + Bi(k)
         * Gi(k) = Ai(k) - Br(k)
         * Gr(N-k) = Ar(k) - Bi(k)
         * Gi(N-k) = -(Ai(k) + Br(k))
         */
        CMPLX_REAL(pi16SrcK) = i16ArTmp + i16BiTmp;
        CMPLX_IMAG(pi16SrcK) = i16AiTmp - i16BrTmp;
        CMPLX_REAL(pi16SrcNK) = i16ArTmp - i16BiTmp;
        CMPLX_IMAG(pi16SrcNK) = -(i16AiTmp + i16BrTmp);
        
        /* Update pointers. */
        pi16SrcK += CMPLX_INCREMENT;
        pi16SrcNK -= CMPLX_INCREMENT;
        pi16Coeff += ui16CoeffOffset;
    }
    
    /* Restore MPY32 control context. */
#if defined(__MSP430_HAS_MPY32__)
    MPY32CTL0 = ui16MPYState;
#endif
    
    /* 
     * Calculate the last result bin where k = N/2-k.
     *
     *     X(k) = X(N-k)
     *     G(k) = 0.5*(X(k) + X*(k)) - j*0.5*(e^-j*pi/2)*(X(k) - X*(k))
     *     G(k) = 0.5(2*Xr(k)) - j*0.5*(-j)*(2*j*Xi(k))
     *     G(k) = Xr(k) - j*Xi(k)
     */
    CMPLX_REAL(pi16SrcK) = CMPLX_REAL(pi16SrcK);
    CMPLX_IMAG(pi16SrcK) = -CMPLX_IMAG(pi16SrcK);
    
    return MSP_SUCCESS;
}
