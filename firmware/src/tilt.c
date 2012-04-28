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
    GPIO_Init();
    GPIO_SetDir(GRN_LED_PRT,GRN_LED_BIT,1); //Grn LED, Out
    GPIO_SetDir(YEL_LED_PRT,YEL_LED_BIT,1); //Yel LED, Out
    
	I2CInit( (uint32_t)I2CMASTER );
	
    init_timer32PWM(1, 1463, 2);
    enable_timer32(1);
    UART_Init(115200); //Init UART at 115200bps
    
    
    UART_printf("i??");
    gyroinit(); // Initialize the Gyro
    UART_printf("init_acc");
    accinit(); // Initialize the Accelero
    UART_printf("init_ma");
    //maginit(); // Initialise the Magnetometer
    UART_printf("hello");
    //*/
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
    int32_t gyroy=0; // Gyro Y Out    gyroinit(); // Initialize the Gyro

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

    while(1){
        //TODO: Read Gyro
        readgyro(0, &gyrox, &gyroy, &gyroz, &gyrot);
        gyro2omega(gyrox, gyroy, gyroz, &w[0], &w[1], &w[2]);
        
        //TODO: Accelerometer:
        readacc(0, &accx, &accy, &accz, &acct);
        acc2deg(accx, accy, accz, &AccXDeg, &AccYDeg); 

	integrateOneStep(dt,w,oldPhi,newPhi);
        
        UART_printf("Gyros: X:%f, Y:%f, Z:%f\n\r", w[0],w[1],w[2]);
    }
}
