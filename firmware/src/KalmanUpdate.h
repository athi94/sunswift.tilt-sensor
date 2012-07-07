/*
 * KalmanUpdate.h
 *
 * Code generation for function 'KalmanUpdate'
 *
 * C source code generated on: Sat Jul 07 10:16:21 2012
 *
 */

#ifndef __KALMANUPDATE_H__
#define __KALMANUPDATE_H__
/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"

#include "rtwtypes.h"
#include "KalmanUpdate_types.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern void KalmanUpdate(const real_T X[3], const real_T P[9], const real_T w[3], const real_T acc[3], real_T dt, const real_T Q[9], const real_T R[9], real_T Xnew[3], real_T Pnew[9]);
#endif
/* End of code generation (KalmanUpdate.h) */
