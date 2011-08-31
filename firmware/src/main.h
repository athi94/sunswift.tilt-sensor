/*
 * main.h
 *
 *  Created on: Feb 10, 2011
 *      Author: charith
 */

#ifndef MAIN_H_
#define MAIN_H_
#endif /* MAIN_H_ */

float CompFilter(float PrevAngle, float GyroOmega, float AccelAngle, float dT, float Offset);
float kalman_update(float gyro_rate, float accel_angle, float dT);

#define SAMPLE_COUNT 512
#define APPLY_GYRO_OFFSET 1 //Edit needed (Check if sensors are working)
#define ENABLE_KALMAN 1
#define ENABLE_COMPLIMENTARY 1
#define ENABLE_CAN 1

#define Pi 3.141592653589793 // The glorious Pi!

unsigned long iapCommand[5] = {0};
unsigned long iapResult[4] = {0};

typedef void (*IAP)(unsigned int [], unsigned int []);
IAP iap_entry;

//IAP (In Application Programming aka - WRITING TO FLASH stuff):
#define IAP_LOCATION 0x1fff1ff1 //<- Defines where to send the commands to
//See page 312 of "LPC111x/LPC11Cxx User manual"

void clrbuf(void);
void i2cdebug(void);
void readsensors(void);

void acc2deg(int32_t accx, int32_t accy, int32_t accz, float *AccXDeg, float *AccYDeg);
void gyro2omega(int32_t gyroxraw, int32_t gyroyraw, int32_t gyrozraw, float *xomega, float *yomega, float *zomega);
void mag2bearing(int32_t magxraw, int32_t magyraw, int32_t magzraw, float *bearing, float *theta);

void readgyro(uint8_t n, int32_t *gyroxout, int32_t *gyroyout, int32_t *gyrozout, int32_t *gyrotout);
void readmag(uint8_t n, int32_t *magxout, int32_t *magyout, int32_t *magzout);
void readacc(uint8_t n, int32_t *accxout, int32_t *accyout, int32_t *acczout, int32_t *acctout);
uint32_t addrgen(uint16_t Ext, uint16_t Pri, uint16_t MsgType, uint16_t NodAddr, uint16_t NodTyp);

void delay(uint32_t delayms);
uint32_t gpioGetValue (uint32_t portNum, uint32_t bitPos);

void gyroinit(void);
void accinit(void);
void maginit(void);

//Debug Stuff:
void PrintUint(uint32_t Input);

 uint32_t cyc; // Cycle variable for everaging
 int16_t cycp; // Averaging period

 int32_t gyrointx=0; // Gyro X Integral
 int32_t gyrointy=0; // Gyro Y Integral
 int32_t gyrointz=0; // Gyro Z Integral

extern volatile uint32_t I2CCount;
extern volatile uint8_t I2CMasterBuffer[BUFSIZE];
extern volatile uint8_t I2CSlaveBuffer[BUFSIZE];
extern volatile uint32_t I2CMasterState;
extern volatile uint32_t I2CReadLength, I2CWriteLength;

typedef volatile uint32_t REG32;
#define pREG32 (REG32 *)
#define GPIO_GPIO0DATA			(*(pREG32 (0x50003FFC)))    // Port data register
#define GPIO_GPIO1DATA			(*(pREG32 (0x50013FFC)))    // Port data register
#define GPIO_GPIO2DATA			(*(pREG32 (0x50023FFC)))    // Port data register
#define GPIO_GPIO3DATA			(*(pREG32 (0x50033FFC)))    // Port data register

#if 1
void PrintUint(uint32_t Input){

  uint16_t i=0;
	uint32_t ShiftChar=1;
	uint32_t OutputVar=0;
	char UARTBUFF[40];

	for (i=0; i<32 ; i++)
	{
		OutputVar=( (Input & (ShiftChar << (31-i))) != 0);
		sprintf(UARTBUFF, "%u", OutputVar);
		UARTSend( &UARTBUFF[0], 1 );

			if((i+1)%8 == 0){
				sprintf(UARTBUFF, " ");
				UARTSend( &UARTBUFF[0], 1 );
				//for( i = 0; UARTBUFF[i] !='\0'; i++){
				//UARTSend( &UARTBUFF[i], 1 );
			}
		 /*for( i = 0; UARTBUFF[i] !='\0'; i++){
		  UARTSend( &UARTBUFF[i], 1 );
		}*/
	}

	  sprintf(UARTBUFF, "\r\n");
	  for( i = 0; UARTBUFF[i] !='\0'; i++){
		  UARTSend( &UARTBUFF[i], 1 ); //(uint8_t *)
	  }

}
#endif

uint32_t addrgen(uint16_t Ext, uint16_t Pri, uint16_t MsgType, uint16_t NodAddr, uint16_t NodTyp){

	return (Ext << 30) | (Pri << 26) | ( MsgType << 18) | (NodAddr << 10) | NodTyp;

}

void clrbuf(void){ // Clear the receive buffers
int16_t i=0;

	for ( i = 0; i < BUFSIZE; i++ )
	  {
		I2CSlaveBuffer[i] = 0x00;
		//UARTBUFF[i]=0x00;
	  }


}

void i2cdebug(void){
	int8_t i2ctemp; // Gyro Temperature (Temporary variable)
	clrbuf();


    // Acquiring Gyro Data
    I2CWriteLength = 2;
  	I2CReadLength = 1;
  	I2CMasterBuffer[0] = 0xD0; // Gyro Address + Write bit
  	I2CMasterBuffer[1] = 0x00; // Start read register
  	I2CMasterBuffer[2] = 0xD0 | RD_BIT; // Gyro Address + Read bit to receive data
  	I2CEngine();

  	// Print Temp Data
  	//gyrottemp=I2CSlaveBuffer[0];
  	printf("i2Tmp: %d\n", I2CSlaveBuffer[0]); // Prints the values into the console

}

void readsensors(void){



}

void acc2deg(int32_t accx, int32_t accy, int32_t accz, float *AccXDeg, float *AccYDeg){


	float accxrad; // Calculated Tilt Angle from Accelero (XZ Plane)
	float accyrad; // Calculated Tilt Angle from Accelero (YZ Plane)

	//accxrad = -acos(accx/sqrt(pow(accz, 2)+pow(accx, 2))); //In good old radians :(
	//accyrad = -acos(accy/sqrt(pow(accz, 2)+pow(accy, 2))); //Not Currently used

	accxrad = atan2(accx, accz);
	accyrad = atan2(accy, accz);

	  *AccXDeg = accxrad*180/Pi;//+90; //PUT BxK in
	  *AccYDeg = accyrad*180/Pi;//+90; //Rename this stuffs!!
}

void gyro2omega(int32_t gyroxraw, int32_t gyroyraw, int32_t gyrozraw, float *xomega, float *yomega, float *zomega){
//Scales the gyro readings to degrees per second
	float scale = 14.375; //Scale from datasheet
	*xomega = (((float) gyroxraw )/scale);
	*yomega = (((float) gyroyraw )/scale);
	*zomega = (((float) gyrozraw )/scale);

}

void mag2bearing(int32_t magxraw, int32_t magyraw, int32_t magzraw, float *bearing, float *theta){


	*bearing = atan2(magyraw, magxraw)*180/Pi;


}

void readgyro(uint8_t n, int32_t *gyroxout, int32_t *gyroyout, int32_t *gyrozout, int32_t *gyrotout)
{

	// if n> 12 (4096 cycles) disallow?
	//uint8_t n=0;

	int16_t i=0;
	int16_t j=0;
	uint32_t IntState=0; //Stores the state of the interrupt pin
	uint16_t cycles;
	int16_t gyrottemp; // Gyro Temperature (Temporary variable)
	int16_t gyroxtemp; // Gyro X (Temporary variable)
	int16_t gyroytemp; // Gyro Y (Temporary variable)
	int16_t gyroztemp; // Gyro Z (Temporary variable)
	int32_t gyrosumt=0;
	int32_t gyrosumx=0;
	int32_t gyrosumy=0;
	int32_t gyrosumz=0;
	float gyroxslope=0;
	float gyroxconst=0;
	float gyroyslope=0;
	float gyroyconst=0;
	float gyrozslope=0;
	float gyrozconst=0;
	float gyroxoffset=0;
	float gyroyoffset=0;
	float gyrozoffset=0;


	cycles=pow(2, n);


#if APPLY_GYRO_OFFSET //APPLY_GYRO_OFFSET, !INDICATE_SENSORS_WORKING
	//These are the values of the zero position varying with temp
	gyroxslope=-0.009979581;
	gyroxconst=63.72678208+16.9;
	gyroyslope=0.01187758;
	gyroyconst=-34.95626176;
	gyrozslope=0.009735277;
	gyrozconst=79.63944424-17.4;
#endif


	  for ( i = 0; i < cycles; i++ ) //TODO: n*n
	  {
		    clrbuf();

		    // Acquiring Gyro Data
		    I2CWriteLength = 2;
		  	I2CReadLength = 8;
		  	I2CMasterBuffer[0] = 0xD0; // Gyro Address + Write bit
		  	I2CMasterBuffer[1] = 0x1B; // Start read register
		  	I2CMasterBuffer[2] = 0xD0 | RD_BIT; // Gyro Address + Read bit to receive data
	    	// Put if statement here!
		  	I2CEngine();

/*		  	 for ( j = 0; j < 2; j++ ) //TODO: n*n
		  		  {
		  		IntState=gpioGetValue (0, 6);
		  		if ( IntState )
		  		{
		  			GPIOSetValue(2,7,1); //Turn off Yellow LED to indicate Sensors clear
		  			j=100000;

				}else{
					GPIOSetValue(2,7,0); // Turn on Yellow LED to indicate Sensors busy
		  		}

		      }
*/

		  	// Format Gyro Data
		  	gyrottemp=((I2CSlaveBuffer[0]<<8)|(I2CSlaveBuffer[1]))+13200; //Bit shifts the Temperature and scales
		  	gyroxtemp=((I2CSlaveBuffer[2]<<8)|(I2CSlaveBuffer[3])); // Bit Shifts the Gyro X readings
		  	gyroytemp=((I2CSlaveBuffer[4]<<8)|(I2CSlaveBuffer[5])); // Bit Shifts the Gyro Y readings
		  	gyroztemp=((I2CSlaveBuffer[6]<<8)|(I2CSlaveBuffer[7])); // Bit Shifts the Gyro Z readings
		  	//printf("Gyr T %d, X %d, Y %d, Z %d\n", gyrottemp, gyroxtemp, gyroytemp, gyroztemp); // Prints the values into the console

		  	// Summing Gyro Data
		  	gyrosumx=gyroxtemp+gyrosumx;
		  	gyrosumy=gyroytemp+gyrosumy;
		  	gyrosumz=gyroztemp+gyrosumz;
		  	gyrosumt=gyrottemp+gyrosumt;

		    //clrbuf();

	  }
	  gyrosumx = gyrosumx >> (n); //Changed from n-1 to n, possible problem here?
	  gyrosumy = gyrosumy >> (n);
	  gyrosumz = gyrosumz >> (n);
	  gyrosumt = gyrosumt >> (n);
	  //printf("Gyr T %d, X %d, Y %d, Z %d\n", gyrosumt, gyrosumx, gyrosumy, gyrosumz);
// Scale - 2^16/4000 = LSB per Deg, need timer before integral starts to work

#if APPLY_GYRO_OFFSET
	  gyroxoffset=gyrosumt*gyroxslope+gyroxconst;
	  gyroyoffset=gyrosumt*gyroyslope+gyroyconst;
	  gyrozoffset=gyrosumt*gyrozslope+gyrozconst;
#endif

	  *gyroxout=gyrosumx-gyroxoffset;
	  *gyroyout=-gyrosumy-gyroyoffset;
	  *gyrozout=gyrosumz-gyrozoffset;
	  *gyrotout=gyrosumt;


	  clrbuf();

	  /* Runge Kutta dY calculation, removed and moved away for now
	  *		 for ( i = 0; i < 3; i++ )
	  *			  {
	  *				 //readgyro(8, gyroxout, gyroyout, gyrozout, gyrotout, 0, 0, 0);
	  *				 //printf("Xo: %d Yo: %d Zo: %d\n", gyrox, gyroy, gyroz);
	  *				 GyroXLog[i]=GyroXLog[i+1];
	  *				 GyroYLog[i]=GyroYLog[i+1];
	  *				 GyroZLog[i]=GyroZLog[i+1];
	  *			  }
	  *
	  *		 GyroXLog[3]=gyrox;
	  *		 GyroYLog[3]=gyroy;
	  *		 GyroZLog[3]=gyroz;
	  *
	  *
	  *
	  *		 gdY=(float)0.166666667*(float)(GyroYLog[0]+2*GyroYLog[1]+2*GyroYLog[2]+GyroYLog[3]);
	  */
}

void readmag(uint8_t n, int32_t *magxout, int32_t *magyout, int32_t *magzout)
{
	int16_t i=0;
	uint16_t cycles;
	int16_t magttemp; // Acc Temperature (Temporary variable)
	int16_t magxtemp; // Acc X (Temporary variable)
	int16_t magytemp; // Acc Y (Temporary variable)
	int16_t magztemp; // Acc Z (Temporary variable)
	int32_t magsumt=0;
	int32_t magsumx=0;
	int32_t magsumy=0;
	int32_t magsumz=0;

	clrbuf();
    //Read Magnetometer Data
    	I2CWriteLength = 2;
    	I2CReadLength = 6;
    	I2CMasterBuffer[0] = 0x3C; // Mag Address + Write bit
    	I2CMasterBuffer[1] = 0x03; // Start read register
    	I2CMasterBuffer[2] = 0x3C | RD_BIT; // Mag Address + Read bit to receive data
    	// Put if statement here!
    	I2CEngine();

    	magxtemp=((I2CSlaveBuffer[0]<<8)|(I2CSlaveBuffer[1])); // Bit Shifts the Gyro X readings
    	magytemp=((I2CSlaveBuffer[2]<<8)|(I2CSlaveBuffer[3])); // Bit Shifts the Gyro Y readings
    	magztemp=((I2CSlaveBuffer[4]<<8)|(I2CSlaveBuffer[5])); // Bit Shifts the Gyro Z readings

    	*magxout=magxtemp;
    	*magyout=magytemp;
    	*magzout=magztemp;
}

void readacc(uint8_t n, int32_t *accxout, int32_t *accyout, int32_t *acczout, int32_t *acctout)
{

	// if n> 12 (4096 cycles) disallow?
	//uint8_t n=0;

	int16_t i=0;
	uint16_t cycles;
	int16_t accttemp; // Acc Temperature (Temporary variable)
	int16_t accxtemp; // Acc X (Temporary variable)
	int16_t accytemp; // Acc Y (Temporary variable)
	int16_t accztemp; // Acc Z (Temporary variable)
	int32_t accsumt=0;
	int32_t accsumx=0;
	int32_t accsumy=0;
	int32_t accsumz=0;

	uint16_t accnegchk[3]; //was temp

	cycles=pow(2, n);

	  for ( i = 0; i < cycles; i++ ) //TODO: n*n
	  {
		    clrbuf();

		    //Read Accelero Data
		    	I2CWriteLength = 2;
		    	I2CReadLength = 7;
		    	I2CMasterBuffer[0] = 0x80; // Accelero Address + Write bit
		    	I2CMasterBuffer[1] = 0x02; // Start read register
		    	I2CMasterBuffer[2] = 0x80 | RD_BIT; // Accelero Address + Read bit to receive data
		    	// Put if statement here!
		    	I2CEngine();

		    	// Format Accelero Data
		    	accnegchk[0]=I2CSlaveBuffer[1]&10000000;
		    	accnegchk[1]=I2CSlaveBuffer[3]&10000000;
		    	accnegchk[2]=I2CSlaveBuffer[5]&10000000;
		    	accxtemp=(I2CSlaveBuffer[1]<<8)|(I2CSlaveBuffer[0]);
		    	accxtemp=accxtemp>>2;
		    	accytemp=(I2CSlaveBuffer[3]<<8)|(I2CSlaveBuffer[2]);
		    	accytemp=accytemp>>2;
		    	accztemp=(I2CSlaveBuffer[5]<<8)|(I2CSlaveBuffer[4]);
		    	accztemp=(accztemp>>2);
		    	accttemp=I2CSlaveBuffer[6];

		    	// Take care of 2's complement stuff:
		    		if (accnegchk[0]==0x80){ //10000000
		    			accxtemp = accxtemp | 0xe000;// 1110000000000000;
		    		}else if (accnegchk[0]==0x00){
		    			accxtemp = accxtemp & 0x1fff;// 0001111111111111;
		    		}else{
		    			//error value: error = error | code
		    			//printf("Err Accelero X\n");
		    		}

		    		if (accnegchk[1]==0x80){
		    			accytemp = accytemp | 0xe000;
		    		}else if (accnegchk[1]==0x00){
		    			accytemp = accytemp & 0x1fff;
		    		}else{
		    			//error value: error = error | code
		    			//printf("Err Accelero Y\n");
		    		}

		    		if (accnegchk[2]== 0x80){
		    			accztemp = accztemp | 0xe000;
		    		}else if (accnegchk[2]== 0x00){
		    			accztemp = accztemp & 0x1fff;
		    		}else{
		    			// Error value: error = error | code
		    			// printf("Err Accelero Z\n");
		    		}

		    		//Apply Calibration Offsets
		    		accxtemp=accxtemp+2440-170;
		    		accytemp=accytemp+949;
		    		accztemp=accztemp-3200+2170;

		  	// Summing Gyro Data
		  	accsumx=accxtemp+accsumx;
		  	accsumy=accytemp+accsumy;
		  	accsumz=accztemp+accsumz;
		  	accsumt=accttemp+accsumt;

		    //clrbuf();

	  }
	  accsumx = accsumx >> (n); //Changed from n-1 to n, possible problem here?
	  accsumy = accsumy >> (n);
	  accsumz = accsumz >> (n);
	  accsumt = accsumt >> (n);
	  //printf("Acc T %d, X %d, Y %d, Z %d\n", accsumt, accsumx, accsumy, accsumz);
	  // Scale - 2^16/4000 = LSB per Deg, need timer before integral starts to work
	  *accxout=accsumx;
	  *accyout=accsumy;
	  *acczout=accsumz;
	  *acctout=accsumt;


	  clrbuf();
}

float CompFilter(float PrevAngle, float GyroOmega, float AccelAngle, float dT, float Offset){

	float GyroPredict;
	GyroPredict = PrevAngle + (GyroOmega * dT)/1000-Offset;
	return (0.997 * GyroPredict + 0.003 * AccelAngle+Offset);
	//D_Angle = GyroRate (Change to GyroInput) * ScaleFactor * dT
	//GyroPredict = PrevAngle * D_Angle
	//NewAngle = 0.97 * GyroPredict + 0.03*AccelAngle
	//Increase 0.99 ratio and better LPF
}

void delay(uint32_t delayms){
	uint32_t endtime=timer32_0_counter+delayms;

	while (timer32_0_counter<endtime){


	}


}
#if ENABLE_KALMAN
float kalman_update(float gyro_rate, float accel_angle, float dT)

// Update the State Estimation and compute the Kalman Gain.
// The estimated angle is returned.
{
    // Inputs.
    float u = gyro_rate;
    float y = accel_angle;

    // Output.
    static float x_00 = 0.0;
    static float x_10 = 0.0;

    // Persistant states.
    static float P_00 = 0.001;
    static float P_01 = 0.003;
    static float P_10 = 0.003;
    static float P_11 = 0.003;

    // Constants.

    // These are the delta in seconds between samples.
    const float A_01 = dT/(-1000); // -0.019968;
    const float B_00 = dT/1000; //0.019968;

    // Data read from 1000 samples of the accelerometer had a variance of 0.07701688.
    const float Sz = 50; //Changed this! again from 1 to 50

    // Data read from 1000 samples of the gyroscope had a variance of 0.00025556.
    // XXX The values below were pulled from the autopilot site, but I'm not sure how to
    // XXX plug them into the process noise covariance matrix.  This needs to be
    // XXX further explored.KALMAN
    const float Sw_00 = 0.001;
    const float Sw_01 = 0.003;
    const float Sw_10 = 0.003;
    const float Sw_11 = 0.003;

    // Temp.
    float s_00;
    float inn_00;
    float K_00;
    float K_10;
    float AP_00;
    float AP_01;
    float AP_10;
    float AP_11;
    float APAT_00;
    float APAT_01;
    float APAT_10;
    float APAT_11;
    float KCPAT_00;
    float KCPAT_01;
    float KCPAT_10;
    float KCPAT_11;

    // Update the state estimate by extrapolating current state estimate with input u.
    // x = A * x + B * u
    x_00 += (A_01 * x_10) + (B_00 * u);

    // Compute the innovation -- error between measured value and state.
    // inn = y - c * x
    inn_00 = y - x_00;

    // Compute the covariance of the innovation.
    // s = C * P * C' + Sz
    s_00 = P_00 + Sz;

    // Compute AP matrix for use below.
    // AP = A * P
    AP_00 = P_00 + A_01 * P_10;
    AP_01 = P_01 + A_01 * P_11;
    AP_10 = P_10;
    AP_11 = P_11;

    // Compute the kalman gain matrix.
    // K = A * P * C' * inv(s)
    K_00 = AP_00 / s_00;
    K_10 = AP_10 / s_00;

    // Update the state estimate.
    // x = x + K * inn
    x_00 += K_00 * inn_00;
    x_10 += K_10 * inn_00;

    // Compute the new covariance of the estimation error.
    // P = A * P * A' - K * C * P * A' + Sw
    APAT_00 = AP_00 + (AP_01 * A_01);
    APAT_01 = AP_01;
    APAT_10 = AP_10 + (AP_11 * A_01);
    APAT_11 = AP_11;
    KCPAT_00 = (K_00 * P_00) + (K_00 * P_01) * A_01;
    KCPAT_01 = (K_00 * P_01);
    KCPAT_10 = (K_10 * P_00) + (K_10 * P_01) * A_01;
    KCPAT_11 = (K_10 * P_01);
    P_00 = APAT_00 - KCPAT_00 + Sw_00;
    P_01 = APAT_01 - KCPAT_01 + Sw_01;
    P_10 = APAT_10 - KCPAT_10 + Sw_10;
    P_11 = APAT_11 - KCPAT_11 + Sw_11;

    // Return the estimate.
    return x_00;
}
#endif


void maginit(void){

	clrbuf();
	  I2CWriteLength = 3;
	  I2CReadLength = 0;
	  I2CMasterBuffer[0] = 0x3C; // Mag Address
	  I2CMasterBuffer[1] = 0x02; // Register to change
	  I2CMasterBuffer[2] = 0x00; // Sets Continuous reading
	  I2CEngine();

	  clrbuf();
}

// Gyroscope Initialization function
void gyroinit(void){

	  uint32_t i;

	 //Clearing slave buffer isn't needed but done anyway
	  for ( i = 0; i < BUFSIZE; i++ )
	  {
		I2CSlaveBuffer[i] = 0x00;
	  }

	  // Perform I2C Write of Address, Write Start Reg, Data
	  I2CWriteLength = 3;
	  I2CReadLength = 0;
	  I2CMasterBuffer[0] = 0xD0; // Gyro Address
	  I2CMasterBuffer[1] = 0x3E; // Register
	  I2CMasterBuffer[2] = 0x04; // 1 Sets internal clock source to PLL off x gyro, 4 uses 32.768KHz input
	  I2CEngine();

	  // Perform I2C Write of Address, Write Start Reg, Data
	  I2CWriteLength = 5;
	  I2CReadLength = 0;
	  I2CMasterBuffer[0] = 0xD0; // Gyro Address
	  I2CMasterBuffer[1] = 0x15; // Register
	  I2CMasterBuffer[2] = 0x01; // Value for 0x15, Sampling rate divider
	  I2CMasterBuffer[3] = 0x1a; // Value for 0x16, Range, low pass and sampl freq (1a)
	  I2CMasterBuffer[4] = 0x31; // Value for 0x17, interrupt config
	  I2CEngine();


}

void accinit(void){

	  uint32_t i;
	  uint8_t AccRegVec[28]; // Accelero Register Vector
	  uint8_t AccAndVec[28]; // Accelero And vector for bitwise operationa
	  uint8_t AccOrVec[28]; // Accelero Or vector for bitwise operationa
	  uint8_t AccVerVec[28]; // Accelero Verification yet TODO:Implement this
	//Configuring Accelero
	  //Set ee_w to allow writing to the registers
	  I2CWriteLength = 3;
	  I2CReadLength = 1;
	  I2CMasterBuffer[0] = 0x80; // Gyro Address
	  I2CMasterBuffer[1] = 0x0D; // Register
	  I2CMasterBuffer[2] = 0x10; // New Value;
	  I2CEngine();

	  //for ( i = 0; i < 0x500000; i++ ); //delay

	  //Accelero read initial registers:
	  I2CWriteLength = 2;
	  I2CReadLength = 28;
	  I2CMasterBuffer[0] = 0x80; // Accelero Address
	  I2CMasterBuffer[1] = 0x20; // Register
	  I2CMasterBuffer[2] = 0x80 | RD_BIT; // New Value
	  I2CEngine();

	  AccRegVec[0] = 0x20;
	  AccRegVec[1] = 0x21;
	  AccRegVec[2] = 0x22;
	  AccRegVec[3] = 0x23;
	  AccRegVec[4] = 0x24;
	  AccRegVec[5] = 0x25;
	  AccRegVec[6] = 0x26;
	  AccRegVec[7] = 0x27;
	  AccRegVec[8] = 0x28;
	  AccRegVec[9] = 0x29;
	  AccRegVec[10] = 0x2A;
	  AccRegVec[11] = 0x2B;
	  AccRegVec[12] = 0x2C;
	  AccRegVec[13] = 0x2D;
	  AccRegVec[14] = 0x2E;
	  AccRegVec[15] = 0x2F;
	  AccRegVec[16] = 0x30;
	  AccRegVec[17] = 0x31;
	  AccRegVec[18] = 0x32;
	  AccRegVec[19] = 0x33;
	  AccRegVec[20] = 0x34;
	  AccRegVec[21] = 0x35;
	  AccRegVec[22] = 0x36;
	  AccRegVec[23] = 0x37;
	  AccRegVec[24] = 0x38;
	  AccRegVec[25] = 0x39;
	  AccRegVec[26] = 0x3A;
	  AccRegVec[27] = 0x3B;
	  AccAndVec[0] = 0x5F;
	  AccAndVec[1] = 0x02;
	  AccAndVec[2] = 0xFF;
	  AccAndVec[3] = 0x00;
	  AccAndVec[4] = 0x00;
	  AccAndVec[5] = 0x00;
	  AccAndVec[6] = 0xFF;
	  AccAndVec[7] = 0x00;
	  AccAndVec[8] = 0x00;
	  AccAndVec[9] = 0x00;
	  AccAndVec[10] = 0x00;
	  AccAndVec[11] = 0x00;
	  AccAndVec[12] = 0x00;
	  AccAndVec[13] = 0x00;
	  AccAndVec[14] = 0xFF;
	  AccAndVec[15] = 0xFF;
	  AccAndVec[16] = 0xFD;
	  AccAndVec[17] = 0xFF;
	  AccAndVec[18] = 0xFF;
	  AccAndVec[19] = 0xFF;
	  AccAndVec[20] = 0xFF;
	  AccAndVec[21] = 0xF3;
	  AccAndVec[22] = 0xFF;
	  AccAndVec[23] = 0xFF;
	  AccAndVec[24] = 0xFF;
	  AccAndVec[25] = 0xFF;
	  AccAndVec[26] = 0xFF;
	  AccAndVec[27] = 0xFF;
	  AccOrVec[0] = 0x50;
	  AccOrVec[1] = 0x02;
	  AccOrVec[2] = 0x00;
	  AccOrVec[3] = 0x00;
	  AccOrVec[4] = 0x00;
	  AccOrVec[5] = 0x00;
	  AccOrVec[6] = 0x00;
	  AccOrVec[7] = 0x00;
	  AccOrVec[8] = 0x00;
	  AccOrVec[9] = 0x00;
	  AccOrVec[10] = 0x00;
	  AccOrVec[11] = 0x00;
	  AccOrVec[12] = 0x00;
	  AccOrVec[13] = 0x00;
	  AccOrVec[14] = 0x00;
	  AccOrVec[15] = 0x00;
	  AccOrVec[16] = 0x01;
	  AccOrVec[17] = 0x00;
	  AccOrVec[18] = 0x00;
	  AccOrVec[19] = 0x00;
	  AccOrVec[20] = 0x00;
	  AccOrVec[21] = 0x02;
	  AccOrVec[22] = 0x00;
	  AccOrVec[23] = 0x00;
	  AccOrVec[24] = 0x00;
	  AccOrVec[25] = 0x00;
	  AccOrVec[26] = 0x00;
	  AccOrVec[27] = 0x00;

	  //Accelero modify registers
	  I2CWriteLength = 57;//1+28*2?
	  I2CReadLength = 0; //
	  I2CMasterBuffer[0] = 0x80; // Accelero Address
	  for (i=0; i<=27; i++)
		  {
			  I2CMasterBuffer[2*i+1]= AccRegVec[i];
			  I2CMasterBuffer[2*i+2] = (I2CSlaveBuffer[i] & AccAndVec[i]) | AccOrVec[i];
			  AccVerVec[i]=I2CMasterBuffer[2*i+2]; //Verification Vector - implement later if time permits!
		  }
	  I2CEngine();


	  for ( i = 0; i < 32; i++ ) //clear slave buffers
	  	  {
			  I2CSlaveBuffer[i] = 0x00;
	  	  }

	  for ( i = 0; i < 0x100000; i++ ); //delay

	  //Lock Buffers again
	  I2CWriteLength = 3;
	    I2CReadLength = 0;
	    I2CMasterBuffer[0] = 0x80; // Accelero Address
	    I2CMasterBuffer[1] = 0x0D; // Register
	    I2CMasterBuffer[2] = 0x00; // Disables image writing
	    I2CEngine();
	//Done configuring Accelero

}

uint32_t gpioGetValue (uint32_t portNum, uint32_t bitPos)
{
  //if (!_gpioInitialised) gpioInit();

  uint32_t value = 0;

  switch (portNum)
  {
    case 0:
      value = (GPIO_GPIO0DATA & (1 << bitPos)) ? 1 : 0;
      break;
    case 1:
      value = (GPIO_GPIO1DATA & (1 << bitPos)) ? 1 : 0;
      break;
    case 2:
      value = (GPIO_GPIO2DATA & (1 << bitPos)) ? 1 : 0;
      break;
    case 3:
      value = (GPIO_GPIO3DATA & (1 << bitPos)) ? 1 : 0;
      break;
    default:
      break;
  }

  return value;
}
