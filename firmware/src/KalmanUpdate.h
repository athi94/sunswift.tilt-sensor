/*
 * KalmanUpdate.h
 *
 * Embedded MATLAB Coder code generation for M-function 'KalmanUpdate'
 *
 * C source code generated on: Sun May 20 12:50:56 2012
 *
 */

#ifndef __KALMANUPDATE_H__
#define __KALMANUPDATE_H__
/* Include files */
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "rtwtypes.h"
#include "rt_nonfinite.h"

/* Type Definitions */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
void KalmanUpdate(real_T *eml_X, real_T *eml_P, real_T *eml_w, real_T *eml_acc, real_T eml_dt, real_T *eml_Xnew, real_T *eml_Pnew);
void KalmanUpdate_initialize(void);
void KalmanUpdate_terminate(void);

#endif
/* End of Embedded MATLAB Coder code generation (KalmanUpdate.h) */
