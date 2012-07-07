/*
 * KalmanUpdate.c
 *
 * Code generation for function 'KalmanUpdate'
 *
 * C source code generated on: Sat Jul 07 10:16:21 2012
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "KalmanUpdate.h"
#include "mldivide.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
static real_T rt_powd_snf(real_T u0, real_T u1);

/* Function Definitions */
static real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  real_T d1;
  real_T d2;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    d1 = fabs(u0);
    d2 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d1 == 1.0) {
        y = rtNaN;
      } else if (d1 > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d2 == 0.0) {
      y = 1.0;
    } else if (d2 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

/*
 * function [Xnew, Pnew] = KalmanUpdate(X,P,w,acc,dt,Q,R)
 */
void KalmanUpdate(const real_T X[3], const real_T P[9], const real_T w[3], const
                  real_T acc[3], real_T dt, const real_T Q[9], const real_T R[9],
                  real_T Xnew[3], real_T Pnew[9])
{
  real_T H[9];
  real_T K[9];
  real_T F[9];
  int32_T i;
  int32_T i0;
  int32_T i1;
  real_T d0;
  real_T b_Pnew[9];
  real_T dv0[9];
  real_T b_w[3];
  real_T dv1[3];
  real_T b_acc[3];
  real_T b_H[9];

  /*  prediction step */
  /* 'KalmanUpdate:4' Xnew = processModel(X,w,dt); */
  /* 'KalmanUpdate:21' f = X + dt*[ (w(1)+( w(2)*sin(X(1))+w(3)*cos(X(1))*tan(X(2))));  */
  /* 'KalmanUpdate:22'                (w(2)*cos(X(1))-w(3)*sin(X(1))); */
  /* 'KalmanUpdate:23'                (w(2)*sin(X(1))+w(3)*cos(X(1)))/cos(X(2)) ]; */
  /* 'KalmanUpdate:5' F = processModelJacobian(X,w,dt); */
  /* 'KalmanUpdate:27' F = eye(3,3) +dt*[ ( w(2)*cos(X(1))-w(3)*sin(X(1)))*tan(X(2)) (sec(X(2)))^2*(w(3)*cos(X(1))+w(2)*sin(X(1)))          0; */
  /* 'KalmanUpdate:28'                      (-w(3)*cos(X(1))-w(2)*sin(X(1)))                      0                                          0; */
  /* 'KalmanUpdate:29'                        sec(X(2))*(w(2)*cos(X(1))-w(3)*sin(X(1))) sec(X(2))*(w(3)*cos(X(1))+w(2)*sin(X(1)))*tan(X(2)) 0 ]; */
  memset(&H[0], 0, 9U * sizeof(real_T));
  K[0] = (w[1] * cos(X[0]) - w[2] * sin(X[0])) * tan(X[1]);
  K[3] = rt_powd_snf(1.0 / cos(X[1]), 2.0) * (w[2] * cos(X[0]) + w[1] * sin(X[0]));
  K[6] = 0.0;
  K[1] = -w[2] * cos(X[0]) - w[1] * sin(X[0]);
  K[4] = 0.0;
  K[7] = 0.0;
  K[2] = 1.0 / cos(X[1]) * (w[1] * cos(X[0]) - w[2] * sin(X[0]));
  K[5] = 1.0 / cos(X[1]) * (w[2] * cos(X[0]) + w[1] * sin(X[0])) * tan(X[1]);
  K[8] = 0.0;
  for (i = 0; i < 3; i++) {
    H[i + 3 * i] = 1.0;
    for (i0 = 0; i0 < 3; i0++) {
      F[i0 + 3 * i] = H[i0 + 3 * i] + dt * K[i0 + 3 * i];
    }
  }

  /* 'KalmanUpdate:6' Pnew = F*P*F' + Q; */
  for (i0 = 0; i0 < 3; i0++) {
    for (i = 0; i < 3; i++) {
      K[i0 + 3 * i] = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        K[i0 + 3 * i] += F[i0 + 3 * i1] * P[i1 + 3 * i];
      }
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (i = 0; i < 3; i++) {
      d0 = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        d0 += K[i0 + 3 * i1] * F[i + 3 * i1];
      }

      Pnew[i0 + 3 * i] = d0 + Q[i0 + 3 * i];
    }
  }

  /*  Kalman update */
  /*  calculate 'innovation': actual measurement - expected measurement */
  /* 'KalmanUpdate:10' z = acc - observationModel(X); */
  /* 'KalmanUpdate:35' h = [-sin(X(2)); cos(X(2))*sin(X(1)); cos(X(2))*cos(X(1)) ]; */
  /* 'KalmanUpdate:11' H = observationModelJacobian(X); */
  /* 'KalmanUpdate:39' H = [ 0                    -cos(X(2))          0; */
  /* 'KalmanUpdate:40'         cos(X(1))*cos(X(2)) -sin(X(1))*sin(X(2)) 0; */
  /* 'KalmanUpdate:41'        -cos(X(2))*sin(X(1)) -cos(X(1))*sin(X(2)) 0 ]; */
  H[0] = 0.0;
  H[3] = -cos(X[1]);
  H[6] = 0.0;
  H[1] = cos(X[0]) * cos(X[1]);
  H[4] = -sin(X[0]) * sin(X[1]);
  H[7] = 0.0;
  H[2] = -cos(X[1]) * sin(X[0]);
  H[5] = -cos(X[0]) * sin(X[1]);
  H[8] = 0.0;

  /* 'KalmanUpdate:13' S = R + H*Pnew*H' ; */
  for (i0 = 0; i0 < 3; i0++) {
    for (i = 0; i < 3; i++) {
      K[i0 + 3 * i] = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        K[i0 + 3 * i] += H[i0 + 3 * i1] * Pnew[i1 + 3 * i];
      }
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (i = 0; i < 3; i++) {
      d0 = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        d0 += K[i0 + 3 * i1] * H[i + 3 * i1];
      }

      F[i0 + 3 * i] = R[i0 + 3 * i] + d0;
    }
  }

  /* 'KalmanUpdate:14' K = Pnew*H'/S ; */
  for (i0 = 0; i0 < 3; i0++) {
    for (i = 0; i < 3; i++) {
      K[i + 3 * i0] = F[i0 + 3 * i];
      b_Pnew[i0 + 3 * i] = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        b_Pnew[i0 + 3 * i] += Pnew[i + 3 * i1] * H[i0 + 3 * i1];
      }
    }
  }

  mldivide(K, b_Pnew, dv0);

  /*  Kalman gain */
  /* 'KalmanUpdate:16' Xnew = Xnew+K*z; */
  b_w[0] = w[0] + (w[1] * sin(X[0]) + w[2] * cos(X[0]) * tan(X[1]));
  b_w[1] = w[1] * cos(X[0]) - w[2] * sin(X[0]);
  b_w[2] = (w[1] * sin(X[0]) + w[2] * cos(X[0])) / cos(X[1]);
  dv1[0] = -sin(X[1]);
  dv1[1] = cos(X[1]) * sin(X[0]);
  dv1[2] = cos(X[1]) * cos(X[0]);
  for (i0 = 0; i0 < 3; i0++) {
    for (i = 0; i < 3; i++) {
      K[i + 3 * i0] = dv0[i0 + 3 * i];
    }

    b_acc[i0] = acc[i0] - dv1[i0];
  }

  for (i0 = 0; i0 < 3; i0++) {
    d0 = 0.0;
    for (i = 0; i < 3; i++) {
      d0 += K[i0 + 3 * i] * b_acc[i];
    }

    Xnew[i0] = (X[i0] + dt * b_w[i0]) + d0;
  }

  /*  update the expected value */
  /* 'KalmanUpdate:17' Pnew = Pnew-S\H*Pnew ; */
  memcpy(&b_H[0], &H[0], 9U * sizeof(real_T));
  mldivide(F, b_H, H);
  for (i0 = 0; i0 < 3; i0++) {
    for (i = 0; i < 3; i++) {
      d0 = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        d0 += H[i0 + 3 * i1] * Pnew[i1 + 3 * i];
      }

      b_Pnew[i0 + 3 * i] = Pnew[i0 + 3 * i] - d0;
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (i = 0; i < 3; i++) {
      Pnew[i + 3 * i0] = b_Pnew[i + 3 * i0];
    }
  }

  /*  update the covariance */
}

/* End of code generation (KalmanUpdate.c) */
