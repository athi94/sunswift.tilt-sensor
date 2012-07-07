/* --------------------------------------------------------------------------                                 
    Tilt sensor project
    File name: tilt.c
    Author: Charith Perera
    Description: This is the main tilt sensor project file, it is intended to
                 take care of all the management stuff - reading data, filtering
                 and sending it out over CAN as required. 

    Copyright (C) Charith Perera, 2011

    Created: ?/2011
   -------------------------------------------------------------------------- */

#include <scandal/engine.h>
#include <scandal/message.h>
#include <scandal/leds.h>
#include <scandal/utils.h>
#include <scandal/uart.h>
#include <scandal/stdio.h>
#include <scandal/wdt.h>
#include <scandal/wavesculptor.h>

#include <string.h>

#include <project/driver_config.h>
#include <project/target_config.h>
#include <arch/can.h>
#include <arch/uart.h>
#include <arch/timer.h>
#include <arch/gpio.h>
#include <arch/types.h>
#include <arch/i2c.h>

#include <math.h>

#include "KalmanUpdate.h"
#include "KalmanUpdate_initialize.h"
#include "sensors.h"

#define GRN_LED_PRT 2
#define GRN_LED_BIT 8
#define YEL_LED_PRT 2
#define YEL_LED_BIT 7


void setup_tilt(void) {

  scandal_init();
  GPIO_Init();
  GPIO_SetDir(GRN_LED_PRT,GRN_LED_BIT,1); //Grn LED, Out
  GPIO_SetDir(YEL_LED_PRT,YEL_LED_BIT,1); //Yel LED, Out
    
  I2CInit( (uint32_t)I2CMASTER );
	
  init_timer32PWM(1, 1463, 2);
  enable_timer32(1);

 
  UART_Init(115200); //Init UART at 115200bps

  gyroinit(); // Initialize the Gyro
  accinit(); // Initialize the Accelero
  //maginit(); // Initialise the Magnetometer

  KalmanUpdate_initialize();
}

int main(void){
    setup_tilt();

    GPIO_SetValue(GRN_LED_PRT,GRN_LED_BIT,1);
    GPIO_SetValue(YEL_LED_PRT, YEL_LED_BIT,1);
    
    int32_t gyrot=0; // Gyro Temp Out
    int32_t gyrox=0; // Gyro X Out
    int32_t gyroy=0; // Gyro Y Out
    int32_t gyroz=0; // Gyro Z Out
    
    int32_t acct=0; // Acc Temp Out
    int32_t accx=0; // Acc X Out
    int32_t accy=0; // Acc Y Out
    int32_t accz=0; // Acc Z Out

    real_T dt;
    real_T w[3];
    real_T P[9] = {Pi/100.0,0,0,0,Pi/100.0,0,0,0,Pi/100.0};
    real_T Pnew[9];
    real_T X[9] = {0.0,0.0,0.0};
    real_T Xnew[9];
    real_T acc[3];
    real_T Q[9] = {Pi/10.0,0,0,0,Pi/10.0,0,0,0,Pi/10.0};
    real_T R[9] = {0.1,0,0,0,0.1,0,0,0,0.1};

    sc_time_t t_old = sc_get_timer();

    while(1){

        readgyro(0, &gyrox, &gyroy, &gyroz, &gyrot);
	// MATLAB uses doubles, gyro2omega uses floats. Can we get MATLAB to use floats?
	float wf[3];
        gyro2omega(gyrox, gyroy, gyroz, &wf[0], &wf[1], &wf[2]);
		w[0] = (real_T) wf[0];
		w[1] = (real_T) wf[1];
		w[2] = (real_T) wf[2];
	/* The gyro2omega function takes in the raw gyro data and converts
	   it into degrees per second. */

	// Need to work with rad/s
	w[0]*=Pi/180.0;
	w[1]*=Pi/180.0;
	w[2]*=Pi/180.0;

	readacc(2, &accx, &accy, &accz, &acct);
	acc[0] = (real_T)accx;
	acc[1] = (real_T)accy;
	acc[2] = (real_T)accz;


	sc_time_t t = sc_get_timer();
	dt = (real_T)(t - t_old);
	t_old = t;
	
	real_T accnorm = sqrt( pow(acc[0],2) + pow(acc[1],2) + pow(acc[2],2) );
	acc[0] = acc[0]/accnorm;
	acc[1] = acc[1]/accnorm;
	acc[2] = acc[2]/accnorm;
	
	// Uses the matlab generated code to do the Kalman Update.
	KalmanUpdate(X,P,w,acc,dt/1000.0,Q,R,Xnew,Pnew);
	memcpy(P,Pnew,9*sizeof(real_T));
	memcpy(X,Xnew,3*sizeof(real_T));
	// lpc1114_flash_can_crp.ld
	UART_printf("%1.5f,%1.5f,%1.5f\n",X[0],X[1],X[2]);
	scandal_delay(1000);
    	GPIO_SetValue(YEL_LED_PRT, YEL_LED_BIT,1);
	GPIO_SetValue(YEL_LED_PRT, YEL_LED_BIT,0);
    }
}
