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
}

typedef float tfloat;
void integrateOneStep(tfloat dt, tfloat* w, tfloat* oldPhi, tfloat* newPhi) {
  tfloat sinphix = sin(oldPhi[0]);
  tfloat cosphix = cos(oldPhi[0]);
  tfloat cosphiy = cos(oldPhi[1]);
  tfloat tanphiy = tan(oldPhi[1]);

  newPhi[0] = oldPhi[0] + (w[0] + (w[1]*sinphix + w[2]*cosphix)*tanphiy)*dt;
  newPhi[1] = oldPhi[1] + (w[1]*cosphix - w[2]*sinphix)*dt;
  newPhi[2] = oldPhi[2] + (w[1]*sinphix + w[2]*cosphix)*dt/cosphiy;
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
    
    int32_t magx=0; // Mag X Out
    int32_t magy=0; // Mag Y Out
    int32_t magz=0; // Mag Z Out
    
    float AccXDeg;
    float AccYDeg;
    float MagBearing;
    float MagTheta;

    tfloat oldPhi[3];
    tfloat newPhi[3];
    tfloat w[3];
    tfloat dt;
    sc_time_t t_old = sc_get_timer();

    oldPhi[0] = 0;
    oldPhi[1] = 0;
    oldPhi[2] = 0;


    while(1){

        readgyro(0, &gyrox, &gyroy, &gyroz, &gyrot);
        gyro2omega(gyrox, gyroy, gyroz, &w[0], &w[1], &w[2]);
	/* The gyro2omega function takes in the raw gyro data and converts
	   it into degrees per second. */
	w[0]*=Pi/180.0;
	w[1]*=Pi/180.0;
	w[2]*=Pi/180.0;        


        readacc(2, &accx, &accy, &accz, &acct);
        acc2deg(accx, accy, accz, &AccXDeg, &AccYDeg); 

	sc_time_t t = sc_get_timer();
	dt = (tfloat)(t - t_old);
	t_old = t;

	integrateOneStep(dt/1000.0,w,oldPhi,newPhi);

        UART_printf("GyroInt:%f,%f,%f\n\r",newPhi[0],newPhi[1],newPhi[2]);
	//UART_printf("GyroOmega:%3.3f, %3.3f, %3.3f\n\r", w[0], w[1], w[2]);
	//UART_printf("dt:%3.3f\n\r", dt/1000);
	UART_printf("AccRaw:%3.3f,%3.3f\n\r", AccXDeg, AccYDeg);
	UART_printf("What's Going on?: f:%f, 3f5:%3.5f\n\r", newPhi[0], newPhi[0]);
	memcpy(oldPhi,newPhi,3*sizeof(tfloat));
	scandal_delay(100);
    }
}
