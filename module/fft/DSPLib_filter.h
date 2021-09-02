#ifndef __DSPLIB_FILTER_H__
#define __DSPLIB_FILTER_H__

/*!
 *  @brief Parameter structure for the Goertzel algorithm filter.
 */
typedef struct msp_goertzel_q15_params {
    uint16_t    length;             /*!< Length of the source data. */
    _q15        cosCoeff;           /*!< Cos coefficient used for calculating the result. */
    _q15        sinCoeff;           /*!< Sin coefficient used for calculating the result. */
    int16_t     *output;            /*!< Complex vector of size one to store the result. */
} msp_goertzel_q15_params;

/*!
 *  @brief Goertzel algorithm filter to calculate a single frequency of the DFT.
 *  @param params Pointer to the Goertzel parameter structure.
 *  @param src Pointer to the source data to filter.
 *  @return none.
 */
extern msp_status msp_goertzel_q15(const msp_goertzel_q15_params *params, const _q15 *src, _q15 *dst);

#endif //__DSPLIB_FILTER_H__
