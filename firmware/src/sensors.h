#include <arch/i2c.h>

#define Pi 3.141592653589793 // The glorious Pi!
#define APPLY_GYRO_OFFSET 1

/*  Initialisation functions for the 3 sensors
 *      Gyroscope      (ITG-3200)
 *      Accelerometer  (BLA180)
 *      Magnetometer   (HMC???) 
 */
void gyroinit(void);
void accinit(void);
void maginit(void);

/*  Reading functions for the 3 sensors
 *  Each function is given a pointer of how many averages to take (n^2)
 *  and also a pointer which points to where the outputs should go
 *  these correspond to each of the 3 orthogonal axes (x, y, z) and also a 
 *  temperature readout if the sensor provides one (t).
 */
void readgyro(uint8_t n, int32_t *gyroxout, int32_t *gyroyout, int32_t *gyrozout, int32_t *gyrotout);
void readmag(uint8_t n, int32_t *magxout, int32_t *magyout, int32_t *magzout);
void readacc(uint8_t n, int32_t *accxout, int32_t *accyout, int32_t *acczout, int32_t *acctout);

/*  Converts the bare outputs of the sensors into human readable, floating point outputs
 *  The inputs are typically the data provided earlier as 3 orthogonal axes and the output
 *  will typically be a degree (accelero and mag) or an omega (angular velocity) value (gyro)
 */

void acc2deg(int32_t accx, int32_t accy, int32_t accz, float *AccXDeg, float *AccYDeg);
void gyro2omega(int32_t gyroxraw, int32_t gyroyraw, int32_t gyrozraw, float *xomega, float *yomega, float *zomega);
void mag2bearing(int32_t magxraw, int32_t magyraw, int32_t magzraw, float *bearing, float *theta);

void clrbuf(void);
