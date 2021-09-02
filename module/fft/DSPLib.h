#ifndef __DSPLIB_H__
#define __DSPLIB_H__

#ifdef __USE_IQMATHLIB__ // Include and use QmathLib and IQmathLib libraries

#include "QmathLib.h"
#include "IQmathLib.h"

#endif  //__USE_IQMATHLIB__

#include "DSPLib_types.h"               // Include DSPLib type definitions
#include "DSPLib_support.h"             // Include DSPLib support functions
#include "DSPLib_vector.h"              // Include DSPLib vector functions
#include "DSPLib_matrix.h"              // Include DSPLib matrix functions
#include "DSPLib_filter.h"              // Include DSPLib filter functions
#include "DSPLib_transform.h"           // Include DSPLib transform functions
#include "DSPLib_utility.h"             // Include DSPLib utility functions

#endif //__DSPLIB_H__
