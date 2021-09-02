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
 * Perform an in-place bit reversal of the complex input array using a lookup
 * table.
 */
msp_status msp_cmplx_bitrev_q15(const msp_cmplx_bitrev_q15_params *params, _q15 *src)
{
    uint16_t i, j;                      // loop counters
    uint16_t ui16Length;                // src length
    int16_t i16BitRev;                  // index bit reversal
    uint16_t ui16TableLen;              // Bit reversal table length
    const uint16_t *pui16Table;         // Bit reversal table
    uint32_t ui32Temp;                  // Temporary storage
    uint32_t *pui32Src;                 // Treat complex data pairs as 32-bit data
    
    /* Initialize source pointer and length. */
    pui32Src = (uint32_t *)src;
    ui16Length = params->length;
    
    /* Check for a null bit reversal pointer. */
    if (params->bitReverseTable == 0) {
        /* Perform the bit reversal without using a lookup table. */
        // for (i = 0; i < ui16Length; i++) {
            // i16BitRev = i;
            // i16BitRev = (((i16BitRev & 0xaaaa) >> 1) | ((i16BitRev & 0x5555) << 1));
            // i16BitRev = (((i16BitRev & 0xcccc) >> 2) | ((i16BitRev & 0x3333) << 2));
            // i16BitRev = (((i16BitRev & 0xf0f0) >> 4) | ((i16BitRev & 0x0f0f) << 4));
            // i16BitRev = __swap_bytes(i16BitRev);
            
            // if (i < i16BitRev) {
                // /* Swap inputs. */
                // ui32Temp = pui32Src[i];
                // pui32Src[i] = pui32Src[i16BitRev];
                // pui32Src[i16BitRev] = ui32Temp;
            // }
        // }
    }
    else{
        /* Initialize table length and pointer. */
        ui16TableLen = *(uint16_t *)params->bitReverseTable;
        pui16Table = &params->bitReverseTable[DSPLIB_TABLE_OFFSET];
        
        if (ui16Length < ui16TableLen) {
            /* In-place bit-reversal using a larger table length than source data length. */
            for (i = 0; i < ui16Length; i++) {
                i16BitRev = pui16Table[i];
                /* Shift the index to the correct length. */
                for (j = ui16Length; j < ui16TableLen; j *= 2) {
                    i16BitRev >>= 1;
                }
                if (i < i16BitRev) {
                    /* Swap inputs. */
                    ui32Temp = pui32Src[i];
                    pui32Src[i] = pui32Src[i16BitRev];
                    pui32Src[i16BitRev] = ui32Temp;
                }
            }
        }
        else if (ui16Length == ui16TableLen) {
            /* Basic in-place bit reversal with correct table length. */
            for (i = 0; i < ui16Length; i++) {
                i16BitRev = pui16Table[i];
                if (i < i16BitRev) {
                    /* Swap inputs. */
                    ui32Temp = pui32Src[i];
                    pui32Src[i] = pui32Src[i16BitRev];
                    pui32Src[i16BitRev] = ui32Temp;
                }
            }
        }
        else {
            return MSP_TABLE_SIZE_ERROR;
        }
    }
    
    return MSP_SUCCESS;
}
