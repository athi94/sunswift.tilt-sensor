/*
 * rt_nonfinite.h
 *
 * Embedded MATLAB Coder code generation for M-function 'KalmanUpdate'
 *
 * C source code generated on: Sun May 20 12:50:56 2012
 *
 */

#ifndef __RT_NONFINITE_H__
#define __RT_NONFINITE_H__

#ifdef _MSC_VER
#include <float.h>
#endif
#include <stddef.h>
#include "rtwtypes.h"

extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;
extern void rt_InitInfAndNaN(size_t realSize);
extern boolean_T rtIsInf(real_T value);
extern boolean_T rtIsInfF(real32_T value);
extern boolean_T rtIsNaN(real_T value);
extern boolean_T rtIsNaNF(real32_T value);

#endif
/* End of Embedded MATLAB Coder code generation (rt_nonfinite.h) */
