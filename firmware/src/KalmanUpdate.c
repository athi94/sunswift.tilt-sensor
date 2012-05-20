/*
 * KalmanUpdate.c
 *
 * Embedded MATLAB Coder code generation for M-function 'KalmanUpdate'
 *
 * C source code generated on: Sun May 20 12:50:56 2012
 *
 */

/* Include files */
#include "KalmanUpdate.h"

/* Type Definitions */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void KalmanUpdate(real_T *eml_X, real_T *eml_P, real_T *eml_w, real_T *eml_acc,
 real_T eml_dt, real_T *eml_Xnew, real_T *eml_Pnew)
{
  int32_T eml_i;
  int32_T eml_p2;
  boolean_T eml_Q[9];
  real_T eml_b_X[3];
  real_T eml_b_w[3];
  real_T eml_c_X[3];
  real_T eml_c_w[3];
  real_T eml_F[9];
  real_T eml_x[9];
  int32_T eml_p3;
  real_T eml_absx11;
  real_T eml_iS[9];
  static boolean_T eml_bv0[9] = { true, false, false, false, true, false, false,
    false, true };
  real_T eml_S[9];
  real_T eml_absx21;
  real_T eml_absx31;
  int32_T eml_itmp;
  real_T eml_dv0[3];
  real_T eml_dv1[3];
  real_T eml_dv2[9];
  real_T eml_dv3[9];
  real_T eml_dv4[9];
  real_T eml_dv5[9];
  real_T eml_dv6[9];
  /*  Q - noise in process model, need to find actual values */
  for(eml_i = 0; eml_i < 3; eml_i++) {
    for(eml_p2 = 0; eml_p2 < 3; eml_p2++) {
      eml_Q[eml_p2 + 3 * eml_i] = false;
    }
    eml_Q[eml_i + 3 * eml_i] = true;
    /*  R - noise in the measurements */
    /*  prediction step */
    eml_b_X[eml_i] = eml_X[eml_i];
    eml_b_w[eml_i] = eml_w[eml_i];
    /*  update the covariance */
    eml_c_X[eml_i] = eml_X[eml_i];
    eml_c_w[eml_i] = eml_w[eml_i];
  }
  eml_F[0] = (eml_c_w[1] * cos(eml_c_X[0]) - eml_c_w[2] * sin(eml_c_X[0])) *
    tan(eml_c_X[1]);
  eml_F[3] = pow(1.0 / cos(eml_c_X[1]), 2.0) * (eml_c_w[2] * cos(eml_c_X[0]) +
   eml_c_w[1] * sin(eml_c_X[0]));
  eml_F[6] = 0.0;
  eml_F[1] = (-eml_c_w[2]) * cos(eml_c_X[0]) - eml_c_w[1] * sin(eml_c_X[0]);
  eml_F[4] = 0.0;
  eml_F[7] = 0.0;
  eml_F[2] = 1.0 / cos(eml_c_X[1]) * (eml_c_w[1] * cos(eml_c_X[0]) - eml_c_w[2]
    * sin(eml_c_X[0]));
  eml_F[5] = 1.0 / cos(eml_c_X[1]) * (eml_c_w[2] * cos(eml_c_X[0]) + eml_c_w[1]
    * sin(eml_c_X[0])) * tan(eml_c_X[1]);
  eml_F[8] = 0.0;
  for(eml_i = 0; eml_i < 3; eml_i++) {
    for(eml_p2 = 0; eml_p2 < 3; eml_p2++) {
      eml_x[eml_i + 3 * eml_p2] = 0.0;
      for(eml_p3 = 0; eml_p3 < 3; eml_p3++) {
        eml_x[eml_i + 3 * eml_p2] += eml_F[eml_i + 3 * eml_p3] * eml_P[eml_p3 +
          3 * eml_p2];
      }
    }
  }
  for(eml_i = 0; eml_i < 3; eml_i++) {
    for(eml_p2 = 0; eml_p2 < 3; eml_p2++) {
      eml_absx11 = 0.0;
      for(eml_p3 = 0; eml_p3 < 3; eml_p3++) {
        eml_absx11 += eml_x[eml_i + 3 * eml_p3] * eml_F[eml_p2 + 3 * eml_p3];
      }
      eml_Pnew[eml_i + 3 * eml_p2] = eml_absx11 + (real_T)eml_Q[eml_i + 3 *
        eml_p2];
    }
  }
  /*  Kalman update */
  /*  calculate 'innovation': actual measurement - expected measurement */
  for(eml_i = 0; eml_i < 3; eml_i++) {
    eml_c_w[eml_i] = eml_X[eml_i];
    eml_c_X[eml_i] = eml_X[eml_i];
  }
  eml_F[0] = 0.0;
  eml_F[3] = -cos(eml_c_X[1]);
  eml_F[6] = 0.0;
  eml_F[1] = cos(eml_c_X[0]) * cos(eml_c_X[1]);
  eml_F[4] = (-sin(eml_c_X[0])) * sin(eml_c_X[1]);
  eml_F[7] = 0.0;
  eml_F[2] = (-cos(eml_c_X[1])) * sin(eml_c_X[0]);
  eml_F[5] = (-cos(eml_c_X[0])) * sin(eml_c_X[1]);
  eml_F[8] = 0.0;
  for(eml_i = 0; eml_i < 3; eml_i++) {
    for(eml_p2 = 0; eml_p2 < 3; eml_p2++) {
      eml_iS[eml_i + 3 * eml_p2] = 0.0;
      for(eml_p3 = 0; eml_p3 < 3; eml_p3++) {
        eml_iS[eml_i + 3 * eml_p2] += eml_F[eml_i + 3 * eml_p3] *
          eml_Pnew[eml_p3 + 3 * eml_p2];
      }
    }
  }
  for(eml_i = 0; eml_i < 3; eml_i++) {
    for(eml_p2 = 0; eml_p2 < 3; eml_p2++) {
      eml_absx11 = 0.0;
      for(eml_p3 = 0; eml_p3 < 3; eml_p3++) {
        eml_absx11 += eml_iS[eml_i + 3 * eml_p3] * eml_F[eml_p2 + 3 * eml_p3];
      }
      eml_S[eml_i + 3 * eml_p2] = (real_T)eml_bv0[eml_i + 3 * eml_p2] +
        eml_absx11;
    }
  }
  for(eml_i = 0; eml_i < 3; eml_i++) {
    for(eml_p2 = 0; eml_p2 < 3; eml_p2++) {
      eml_x[eml_p2 + 3 * eml_i] = eml_S[eml_p2 + 3 * eml_i];
      eml_iS[eml_p2 + 3 * eml_i] = 0.0;
    }
  }
  eml_i = 0;
  eml_p2 = 3;
  eml_p3 = 6;
  eml_absx11 = fabs(eml_S[0]);
  eml_absx21 = fabs(eml_S[1]);
  eml_absx31 = fabs(eml_S[2]);
  if((eml_absx21 > eml_absx11) && (eml_absx21 > eml_absx31)) {
    eml_i = 3;
    eml_p2 = 0;
    eml_x[0] = eml_S[1];
    eml_x[1] = eml_S[0];
    eml_absx11 = eml_x[3];
    eml_x[3] = eml_x[4];
    eml_x[4] = eml_absx11;
    eml_absx11 = eml_x[6];
    eml_x[6] = eml_x[7];
    eml_x[7] = eml_absx11;
  } else if(eml_absx31 > eml_absx11) {
    eml_i = 6;
    eml_p3 = 0;
    eml_x[0] = eml_S[2];
    eml_x[2] = eml_S[0];
    eml_absx11 = eml_x[3];
    eml_x[3] = eml_x[5];
    eml_x[5] = eml_absx11;
    eml_absx11 = eml_x[6];
    eml_x[6] = eml_x[8];
    eml_x[8] = eml_absx11;
  }
  eml_x[1] /= eml_x[0];
  eml_x[2] /= eml_x[0];
  eml_x[4] -= eml_x[1] * eml_x[3];
  eml_x[5] -= eml_x[2] * eml_x[3];
  eml_x[7] -= eml_x[1] * eml_x[6];
  eml_x[8] -= eml_x[2] * eml_x[6];
  if(fabs(eml_x[5]) > fabs(eml_x[4])) {
    eml_itmp = eml_p2;
    eml_p2 = eml_p3;
    eml_p3 = eml_itmp;
    eml_absx11 = eml_x[1];
    eml_x[1] = eml_x[2];
    eml_x[2] = eml_absx11;
    eml_absx11 = eml_x[4];
    eml_x[4] = eml_x[5];
    eml_x[5] = eml_absx11;
    eml_absx11 = eml_x[7];
    eml_x[7] = eml_x[8];
    eml_x[8] = eml_absx11;
  }
  eml_x[5] /= eml_x[4];
  eml_x[8] -= eml_x[5] * eml_x[7];
  eml_absx11 = (eml_x[5] * eml_x[1] - eml_x[2]) / eml_x[8];
  eml_absx21 = (-(eml_x[1] + eml_x[7] * eml_absx11)) / eml_x[4];
  eml_iS[eml_i] = ((1.0 - eml_x[3] * eml_absx21) - eml_x[6] * eml_absx11) /
    eml_x[0];
  eml_iS[eml_i + 1] = eml_absx21;
  eml_iS[eml_i + 2] = eml_absx11;
  eml_absx11 = (-eml_x[5]) / eml_x[8];
  eml_absx21 = (1.0 - eml_x[7] * eml_absx11) / eml_x[4];
  eml_iS[eml_p2] = (-(eml_x[3] * eml_absx21 + eml_x[6] * eml_absx11)) / eml_x[0];
  eml_iS[eml_p2 + 1] = eml_absx21;
  eml_iS[eml_p2 + 2] = eml_absx11;
  eml_absx11 = 1.0 / eml_x[8];
  eml_absx21 = (-eml_x[7]) * eml_absx11 / eml_x[4];
  eml_iS[eml_p3] = (-(eml_x[3] * eml_absx21 + eml_x[6] * eml_absx11)) / eml_x[0];
  eml_iS[eml_p3 + 1] = eml_absx21;
  eml_iS[eml_p3 + 2] = eml_absx11;
  /*  Kalman gain */
  eml_dv0[0] = eml_b_w[0] + (eml_b_w[1] * sin(eml_b_X[0]) + eml_b_w[2] *
    cos(eml_b_X[0]) * tan(eml_b_X[1]));
  eml_dv0[1] = eml_b_w[1] * cos(eml_b_X[0]) - eml_b_w[2] * sin(eml_b_X[0]);
  eml_dv0[2] = (eml_b_w[1] * sin(eml_b_X[0]) + eml_b_w[2] * cos(eml_b_X[0])) /
    cos(eml_b_X[1]);
  eml_dv1[0] = -sin(eml_c_w[1]);
  eml_dv1[1] = cos(eml_c_w[1]) * sin(eml_c_w[0]);
  eml_dv1[2] = cos(eml_c_w[1]) * cos(eml_c_w[0]);
  for(eml_p3 = 0; eml_p3 < 3; eml_p3++) {
    for(eml_i = 0; eml_i < 3; eml_i++) {
      eml_dv2[eml_p3 + 3 * eml_i] = 0.0;
      for(eml_p2 = 0; eml_p2 < 3; eml_p2++) {
        eml_dv2[eml_p3 + 3 * eml_i] += eml_Pnew[eml_p3 + 3 * eml_p2] *
          eml_F[eml_i + 3 * eml_p2];
      }
    }
    for(eml_i = 0; eml_i < 3; eml_i++) {
      eml_dv3[eml_p3 + 3 * eml_i] = 0.0;
      for(eml_p2 = 0; eml_p2 < 3; eml_p2++) {
        eml_dv3[eml_p3 + 3 * eml_i] += eml_dv2[eml_p3 + 3 * eml_p2] *
          eml_iS[eml_p2 + 3 * eml_i];
      }
    }
  }
  for(eml_i = 0; eml_i < 3; eml_i++) {
    eml_b_X[eml_i] = eml_w[eml_i] - eml_dv1[eml_i];
  }
  for(eml_i = 0; eml_i < 3; eml_i++) {
    eml_absx11 = 0.0;
    for(eml_p2 = 0; eml_p2 < 3; eml_p2++) {
      eml_absx11 += eml_dv3[eml_i + 3 * eml_p2] * eml_b_X[eml_p2];
    }
    eml_Xnew[eml_i] = (eml_X[eml_i] + eml_dt * eml_dv0[eml_i]) + eml_absx11;
  }
  /*  update the expected value */
  for(eml_p3 = 0; eml_p3 < 3; eml_p3++) {
    for(eml_i = 0; eml_i < 3; eml_i++) {
      eml_dv4[eml_p3 + 3 * eml_i] = 0.0;
      for(eml_p2 = 0; eml_p2 < 3; eml_p2++) {
        eml_dv4[eml_p3 + 3 * eml_i] += eml_Pnew[eml_p3 + 3 * eml_p2] *
          eml_F[eml_i + 3 * eml_p2];
      }
    }
    for(eml_i = 0; eml_i < 3; eml_i++) {
      eml_dv5[eml_p3 + 3 * eml_i] = 0.0;
      for(eml_p2 = 0; eml_p2 < 3; eml_p2++) {
        eml_dv5[eml_p3 + 3 * eml_i] += eml_dv4[eml_p3 + 3 * eml_p2] *
          eml_iS[eml_p2 + 3 * eml_i];
      }
    }
    for(eml_i = 0; eml_i < 3; eml_i++) {
      eml_dv6[eml_p3 + 3 * eml_i] = 0.0;
      for(eml_p2 = 0; eml_p2 < 3; eml_p2++) {
        eml_dv6[eml_p3 + 3 * eml_i] += eml_dv5[eml_p3 + 3 * eml_p2] *
          eml_F[eml_p2 + 3 * eml_i];
      }
    }
  }
  for(eml_i = 0; eml_i < 3; eml_i++) {
    for(eml_p2 = 0; eml_p2 < 3; eml_p2++) {
      eml_absx11 = 0.0;
      for(eml_p3 = 0; eml_p3 < 3; eml_p3++) {
        eml_absx11 += eml_dv6[eml_i + 3 * eml_p3] * eml_Pnew[eml_p3 + 3 *
          eml_p2];
      }
      eml_x[eml_i + 3 * eml_p2] = eml_Pnew[eml_i + 3 * eml_p2] - eml_absx11;
    }
  }
  for(eml_i = 0; eml_i < 3; eml_i++) {
    for(eml_p2 = 0; eml_p2 < 3; eml_p2++) {
      eml_Pnew[eml_p2 + 3 * eml_i] = eml_x[eml_p2 + 3 * eml_i];
    }
  }
}

void KalmanUpdate_initialize(void)
{
  rt_InitInfAndNaN(8U);
}

void KalmanUpdate_terminate(void)
{
}

/* End of Embedded MATLAB Coder code generation (KalmanUpdate.c) */
