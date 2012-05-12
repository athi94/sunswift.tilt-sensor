#include <project/driver_config.h>
#include <project/target_config.h>

#include <arch/timer32.h>
#include <arch/gpio.h>
#include <arch/uart.h>
#include <arch/type.h>
#include <arch/can.h>
#include <arch/i2c.h>
#include <math.h>

#include "main.h"

  uint32_t cyc; // Cycle variable for everaging
  int16_t cycp; // Averaging period

 // int32_t gyrointx=0; // Gyro X Integral
 // int32_t gyrointy=0; // Gyro Y Integral
 // int32_t gyrointz=0; // Gyro Z Integral
  int8_t acct; // Accelero Temp Out
  int16_t accx; // Accelero X Out
  int16_t accy; // Accelero Y Out
  int16_t accz; // Accelero Z Out

extern volatile uint32_t I2CCount;
extern volatile uint8_t I2CMasterBuffer[BUFSIZE];
extern volatile uint8_t I2CSlaveBuffer[BUFSIZE];
extern volatile uint32_t I2CMasterState;
extern volatile uint32_t I2CReadLength, I2CWriteLength;

extern volatile uint32_t UARTCount;
extern volatile uint8_t UARTBuffer[BUFSIZE];

extern int32_t SCANDALClockOffset; //TEST

#if ENABLE_CAN
	extern message_object can_buff[MSG_OBJ_MAX];
	extern uint32_t CANRxDone[MSG_OBJ_MAX];
	volatile 
#endif
volatile float Acceleration=0; //To store the acceleration of the car for subtraction by the accelerometer
volatile int32_t Acc_DiffV=0;
volatile int32_t Acc_LastV=0;
volatile uint32_t Acc_DiffT=0;
volatile uint32_t Acc_LastT=0;

void setup_ports(void) {
	// Initializing the GPIO's
	GPIOInit();
	// Setting Port 0.7 as an output; Format:(Port (0), Bit (7), In(0)/Out (1))
	GPIOSetDir(0,7,1);
	GPIOSetDir(2,11,0);
	//gpioGetValue (2, 11);

	GPIOSetDir(2,9,0); //Magnetometer interrupt in
	
	// LEDs
	GPIOSetDir(2,8,1); //Green LED, Out
	//GPIOSetValue(2,8,0); //Green LED, Low (on)
	//GPIOSetValue(2,8,1); //Green LED, High (off)

	GPIOSetDir(2,7,1); //Yel LED, Out
	//GPIOSetValue(2,7,0); //Yel LED, Low (on)
	//GPIOSetValue(2,7,1); //Yel LED, Low (off)
}

/*******************************************************************************
**   Main Function  main()
*******************************************************************************/
//int __main();
int main (void)
{
	setup_ports();
	scandal_init();

	init_timer32PWM(1, 1463, 2);
	enable_timer32(1);


#if ENABLE_CAN

/*
	#define Ext (1 << 30)
	#define Pri (7 << 26)
	#define MsgType (0 << 18)
	#define NodAddr (61 << 10) //Node Address
	#define NodTyp (0 << 0) //Channel ID
*/

	CAN_Init(BITRATE50K16MHZ);
	  uint32_t *add;

	  /* Clear all message buffers allocated for 32 CAN message objects. */
	  add = (uint32_t *)can_buff;
	  while ( add < (uint32_t *)(&can_buff[MSG_OBJ_MAX-1] + 1))
	  {
		*add++ = 0x00;
	  }

	  //Stuff for the tilt code
	  uint32_t NextHBSendTime=timer32_0_counter;
	  uint32_t NextDataSendTime=timer32_0_counter;
	  uint32_t CanTimeStamp;
	  int32_t CanPitch;
	  int32_t CanRoll;

	  //uint32_t TX_Address = ( Ext | Pri | MsgType | NodAddr );

#endif


	uint32_t *gpread;
	gpread=(uint32_t *) 0x50033FFC; // GPIO Read register

	uint32_t *P07IO;
	P07IO=(uint32_t *) 0x40044050; //

	UARTInit(115200); //Init UART at 115200bps

	#if MODEM_TEST
		ModemInit();
	#endif

 /*//	Clock Division for the Gyro, not used
  * TODO: Save the syntax somewhere and remove this from main
		LPC_SYSCON->SYSAHBCLKCTRL |= (1<<6); // Enable GPIO clk
		LPC_IOCON->PIO0_1 &= ~0x3F; // Select clkout function for P0.1
		LPC_IOCON->PIO0_1 |= 0x01;
		LPC_SYSCON->CLKOUTCLKSEL = 0x03; // IRC: 0x00 System osci: 0x01 WTD: 0x02 Main clk: 0x03
		LPC_SYSCON->CLKOUTUEN = 0x01; // Update clock
		LPC_SYSCON->CLKOUTUEN = 0x00; // Toggle update register once
		LPC_SYSCON->CLKOUTUEN = 0x01;
		while ( !(LPC_SYSCON->CLKOUTUEN & 0x01) ); // Wait until updated
		LPC_SYSCON->CLKOUTDIV = 200;//1465; // Divided by 1
*/

	  //GPIO Silkscreen errors for LPCXpresso:
	  // IC 2.4 is connected to USB-DM
	  //PCB 2.4 = IC 3.4
	  //PCB 2.5 = IC 3.5
	  //PCB 2.6 = 2.6, see last page of lpcxpresso schematic, errors listed

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
	float gyroXOmega;
	float gyroYOmega;
	float gyroZOmega;
	float MagBearing;
	float MagTheta;

	uint32_t i; // Counter variable
	uint32_t j;
	char UARTBUFF[128];

	int32_t GyroXLog[4];
	int32_t GyroYLog[4];
	int32_t GyroZLog[4];
	uint32_t clkold=0;
	uint32_t clknew=0;
	float dT; // Time taken for last sampling period



	float kalman_output;
	float CompXState;
	float CompYState;


	uint8_t temp[3]; // Temporary variable for Accelero bit shifting


	if ( I2CInit( (uint32_t)I2CMASTER ) == FALSE )	/* initialize I2c */
		{
			while ( 1 );				/* Fatal error */
		}
	delay(100);
//TODO: Initialisation
	gyroinit(); // Initialize the Gyro
	accinit(); // Initialize the Accelero
	maginit(); // Initialise the Magnetometer


	//for ( i = 0; i < 0x1000000; i++ );
	//for ( i = 0; i < 0x100000; i++ ); //Delay
	delay(100);
	//Initialize CompXState
	 readacc(0, &accx, &accy, &accz, &acct);
	 acc2deg(accx, accy, accz, &AccXDeg, &AccYDeg);
	 CompXState = AccXDeg;
	 CompYState = AccYDeg;


	 //CompXOffset=CompOffset();
	//
	 double YTilt;
	 double gdY;
	  clknew=timer32_0_counter;

	  //Start infinite loop
  while (1)
  {

#if 1	//Debug Section
	  {

		GPIOSetDir(2,9,0);
		maginit();
		delay(100);
		uint32_t BinaryVariable=0;
		uint32_t ND1=0;
		uint32_t ND2=0;
		uint8_t RdNum=0;
		int32_t tmpDataHold=0;
		uint32_t tmpTimeHold=0;

		// LOOK HERE ETIENNE <-----------------------------------------------------------------------

		
		//These two are for setting up the ability to call IAP (In application programming, ie writing to flash)
		IAP iap_entry;
		iap_entry=(IAP) IAP_LOCATION;

		//Sector 7 (there are 8 sectors, 0 up to 7) starts from 0x00007000 so this is just the pointer of the start
		//of sector 7. 
		uint32_t *regReadVal=((pREG32 (0x00007000)));//uint32_t* (0x00007000); (*(pREG32 (0x50003FFC)))

		//The following prepares the sector for writing, erases it, prepares again and writes flashValues into the flashValues
		//Its 0 at the moment so its not flashed every time the chip powers up, that'd be wasteful
		//After its flashed once, it can be read again later so yeah, its been disabled for now
		#if 0
		//Prepare for write
		iapCommand[0]=50;	//command to prepare
		iapCommand[1]=7;	//From sector 7
		iapCommand[2]=7;	//To sector 7
		iap_entry (iapCommand, iapResult); //send the commands
		delay(1);

		//Erase
		iapCommand[0]=52; //Command to erase
		iapCommand[1]=7;	//From sector 7
		iapCommand[2]=7;	//To sector 7
		iapCommand[3]=48000;	//Clock rate in khz 
		iap_entry (iapCommand, iapResult); //send the commands
		delay(120); //wait for it to be done

		//Prepare for write
		iapCommand[0]=50;
		iapCommand[1]=7;
		iapCommand[2]=7;
		iap_entry (iapCommand, iapResult);
		delay(1);

		//Setting up some test values to put to flash (counting down from 64)
		uint32_t flashValues[64];
		for(i=0; i<64; i++){
		    flashValues[i]=64-i;

		}

		//Write
		iapCommand[0]=51;	//Write to flash command
		iapCommand[1]=0x00007000; //Write starting from this flash address
		iapCommand[2]=&flashValues; //Read from this memory address
		iapCommand[3]=256;	//256 bytes is the minimum size we can write to at once
		iapCommand[4]=48000;	//clock speed in khz - 48000
		iap_entry (iapCommand, iapResult);
		//	  0x00007000
		//__enable_irq();
		#endif


		//PrintUint(BinaryVariable); //This function can convert a 32 bit unsigned int and print it as binary over UART
		j=0;
		
	  while ( 1 ) { //Loop, runs at 5Hz
		    delay(100);

#if 0 
		//Printing out the time offset from timesyncing
		FetchData(0, &tmpDataHold, &tmpTimeHold);
		SCANDAL_Send(7, 0, 61, 0, SCANDALClockOffset);
#endif

		sprintf(UARTBUFF, "%3d - IAP: St:%d R1:%d R2:%d R3:%d, RV:%u\r\n", j, iapResult[0], iapResult[1], iapResult[2], iapResult[3], *(regReadVal+(j%64)));
		for( i = 0; UARTBUFF[i] !='\0'; i++)
		{
		    UARTSend( &UARTBUFF[i], 1 ); //(uint8_t *)
		}
		j++;
		
#if 0
		    RdNum = 1;

		    sprintf(UARTBUFF, "Obj%u - %x, %x, %x, %x, %x, D-%d\r\n", RdNum, can_buff[RdNum].id, can_buff[RdNum].data[0], can_buff[RdNum].data[1], can_buff[RdNum].data[2], can_buff[RdNum].data[3], CANRxDone[RdNum]);
		    for( i = 0; UARTBUFF[i] !='\0'; i++)
		    {
			  UARTSend( &UARTBUFF[i], 1 ); //(uint8_t *)
		    }
#endif


#if 0
		    for(j=0; j<20; j++)
		    {
			  if (CANRxDone[j]==1)
			  {

				    RdNum = j;

				    sprintf(UARTBUFF, "Obj%1u - %8x,D %10d,TS %10u\r\n", RdNum, can_buff[RdNum].id, (can_buff[RdNum].data[0]<<16) | can_buff[RdNum].data[1], (can_buff[RdNum].data[2]<<16) | can_buff[RdNum].data[3]);
				    for( i = 0; UARTBUFF[i] !='\0'; i++){
				    UARTSend( &UARTBUFF[i], 1 ); //(uint8_t *)
				    }
				    CANRxDone[j]=0;


			  }

		    }
#endif

		    //CAN_MessageProcess(RdNum);


		    /*
		    for( i = 0; UARTBUFF[i] !='\0'; i++){
			  UARTSend( &UARTBUFF[i], 1 ); //(uint8_t *)
		    }*/

/*
 * Print out the can frame in Hex:
  sprintf(UARTBUFF, "Obj%u - %x, %x, %x, %x, %x\r\n", RdNum, can_buff[RdNum].id, can_buff[RdNum].data[0], can_buff[RdNum].data[1], can_buff[RdNum].data[2], can_buff[RdNum].data[3]);
  for( i = 0; UARTBUFF[i] !='\0'; i++){
	UARTSend( &UARTBUFF[i], 1 ); //(uint8_t *)
  }
 *
 */

					if(gpioGetValue(2, 9)){ //MagInt is P2.9, test on another pin
						GPIOSetValue(2,7,0); //Yel LED, Low (on)
						GPIOSetValue(2,8,0); //Green LED, Low (on)
					}else{
						GPIOSetValue(2,7,1); //Yel LED, Low (off)
						GPIOSetValue(2,8,1); //Green LED, Low (off)
					}

				} //End of While
			}
#endif

/*	  //Testing UART and I2C
	  		uint8_t uartdata;
	  		uartdata = 7;

	  		 while (1)
	  		  {
	  			 i2cdebug();
	  			 //printf("%u \n", &uartdata);
	  			 //UARTSend(&uartdata, 1);
	  			 printf("Time: %d", timer32_0_counter);
	  		  }
*/

	  clkold=clknew; //clkold is clknew from the last iteration
	  clknew=timer32_0_counter; //sets clknew to current
	  dT=clknew-clkold; // Calculate dT for last iteration

	#if 0 //Clock testing, blinks the Yel LED at 500Hz
	  while(1){
		  if((timer32_0_counter%2)== 1){
				GPIOSetValue(2,7,0); //Yel LED, Low (on)
		  }else{
				GPIOSetValue(2,7,1); //Yel LED, Low (off)
		  }
	  }
	#endif

#if 0
	  float Yorientation=0;
	  while(1)
	  {
		if(gpioGetValue(0, 6)==1){

			clkold=clknew; //clkold is clknew from the last iteration
			clknew=timer32_0_counter; //sets clknew to current
			dT=clknew-clkold; // Calculate dT for last iteration

			readgyro(0, &gyrox, &gyroy, &gyroz, &gyrot);
			gyro2omega(gyrox, gyroy, gyroz, &gyroXOmega, &gyroYOmega, &gyroZOmega);
			Yorientation = ((gyroYOmega-4.727) * (float) dT )/1000 + Yorientation;

			sprintf(UARTBUFF, "Y: %f, dY: %f, dt: %d \r\n", Yorientation, gyroYOmega, dT);
			for( i = 0; UARTBUFF[i] !='\0'; i++){
				UARTSend( &UARTBUFF[i], 1 ); //(uint8_t *)
			}
		}else{

		}


	  }

#endif


	  //TODO: Read Gyro
	  readgyro(0, &gyrox, &gyroy, &gyroz, &gyrot);
	  gyro2omega(gyrox, gyroy, gyroz, &gyroXOmega, &gyroYOmega, &gyroZOmega);

	  //TODO: Accelerometer:
	 readacc(0, &accx, &accy, &accz, &acct);
	 acc2deg(accx, accy, accz, &AccXDeg, &AccYDeg);

	 readmag(0, &magx, &magy, &magz);
	 mag2bearing(magx, magy, magz, &MagBearing, &MagTheta);

	  GPIOSetValue(0,7,1);

#if ENABLE_KALMAN
	  kalman_output = kalman_update(gyroYOmega, AccXDeg, dT);
#else
	  kalman_output=0;
#endif

#if ENABLE_COMPLIMENTARY
	  CompXState=CompFilter(CompXState, gyroYOmega, AccXDeg, dT, 0);
	  CompYState=CompFilter(CompYState, gyroXOmega, AccYDeg, dT, 0);
#else
	  CompXState=0;
	  CompYState=0;
#endif

	  GPIOSetValue(0,7,0);

	  //TODO: Demo code for speech (Kalman Print)
//	  printf("Acc: %g\n", accxang*180/Pi+90);
	  //printf("%d gyro=%7.3f deg/sec    accel=%7.3f deg    kalman_output=%5.1f deg\n", i, gyroinp, accinp, kalman_output);
	  //printf("G,%d,A,%f,K,%f:\n", gyroy, accxang*180/Pi+90,kalman_output);

	  //UART Print
	//sprintf(UARTBUFF, "T,%8d,G,%4d,A,%3.4f,K,%3.4f:\r\n",clknew, gyroy, accxang*180/Pi+90,kalman_output);
	  //Print Gyro output:
/*	  sprintf(UARTBUFF, "Gyro: X, %d, Y, %d, Z, %d, T, %d\r\n", gyrox, gyroy, gyroz, gyrot);
	for( i = 0; UARTBUFF[i] !='\0'; i++){
		UARTSend( &UARTBUFF[i], 1 ); //(uint8_t *)
	}*/

	  //CAN Sending

#if ENABLE_CAN //CAN Sending

	  //addrgen(uint16_t Ext, uint16_t Pri, uint16_t MsgType, uint16_t NodAddr, uint16_t NodTyp)
	  //Extended, Priority, Type, Address, Channel


	  CanTimeStamp = timer32_0_counter;
	  if (timer32_0_counter>NextDataSendTime){
	  
	  CanPitch = CompXState*1000;
	  CanRoll = CompYState*1000;
	  int32_t CanAcc = ((int32_t) Acceleration) * 1000;
	  
/*	  can_buff[0].id      = addrgen(1, 7, 0, 61, 0);	// standard frame 
	  can_buff[0].dlc     = 0x08;		// Length = 8 
	  can_buff[0].data[0] = ((CanPitch >> 16) & 0x0000FFFF);//0000
	  can_buff[0].data[1] = (CanPitch & 0x0000FFFF);//03E1
	  can_buff[0].data[2] = ((CanTimeStamp >> 16) & 0x0000FFFF);//0006
	  can_buff[0].data[3] = (CanTimeStamp & 0x0000FFFF);//1A82
	  CAN_Send( 0, FALSE, (uint32_t *)&can_buff[0] );
*/
    SCANDAL_Send(7, 0, 61, 0, CanPitch);
    SCANDAL_Send(7, 0, 61, 1, CanRoll);
    SCANDAL_Send(7, 0, 61, 2, CanAcc);

    //SCANDAL_Send(7, 0, 61, 0, CanAcc);
    
    

//	  can_buff[1].id      = addrgen(1, 7, 0, 61, 1);
//	  can_buff[1].dlc     = 0x08;
//	  can_buff[1].data[0] = ((CanRoll >> 16) & 0x0000FFFF);//0000
//	  can_buff[1].data[1] = (CanRoll & 0x0000FFFF);//03E1
//	  can_buff[1].data[2] = ((CanTimeStamp >> 16) & 0x0000FFFF);//0006
//	  can_buff[1].data[3] = (CanTimeStamp & 0x0000FFFF);//1A82
//	  CAN_Send( 1, FALSE, (uint32_t *)&can_buff[1] );


	  //CAN_Send( 0, FALSE, (uint32_t *)&can_buff[0] );
	  NextDataSendTime=timer32_0_counter+100;
	  }
	  
	  
	  
	  // Send a heartbeat
	  if (timer32_0_counter>NextHBSendTime){
		  CanTimeStamp = timer32_0_counter;
	  	  can_buff[0].id      = addrgen(1, 7, 2, 61, 21);	/* standard frame */
	  	  can_buff[0].dlc     = 0x08;		/* Length = 8 */
	  	  can_buff[0].data[0] = 0x0000;//0000
	  	  can_buff[0].data[1] = 0x0A00;//03E1
		  can_buff[0].data[2] = ((CanTimeStamp >> 16) & 0x0000FFFF);//0006
		  can_buff[0].data[3] = (CanTimeStamp & 0x0000FFFF);//1A82

	  	  CAN_Send( 0, FALSE, (uint32_t *)&can_buff[0] );
	  	  NextHBSendTime=timer32_0_counter+1000;
	  	  }
#endif

	#if 0 //Tilt data out
	  sprintf(UARTBUFF, "Xa, %02.5f, Xc: %02.5f, Ya, %02.5f, Yc: %02.5f\r\n", AccXDeg,CompXState,AccYDeg, CompYState);
	  for( i = 0; UARTBUFF[i] !='\0'; i++){
		  UARTSend( &UARTBUFF[i], 1 ); //(uint8_t *)
	  }
	#endif

	#if 0
	  //Print Accelerometer output:
	  sprintf(UARTBUFF, "Acc: X, %d, Y, %d, Z, %d, T, %d\r\n", accx, accy, accz, acct);
	  for( i = 0; UARTBUFF[i] !='\0'; i++){
		UARTSend( &UARTBUFF[i], 1 ); //(uint8_t *)
	  }
	#endif

	#if 0 //Print Magnetometer data
	  sprintf(UARTBUFF, "Mag: Bear, %g,X, %d, Y, %d, Z %d, %02.5f, %02.5f\r\n", MagBearing, magx, magy, magz, CompXState, CompYState);
	  //sprintf(UARTBUFF, "%u \r\n",gpioGetValue(2, 10));
	  for( i = 0; UARTBUFF[i] !='\0'; i++){
		UARTSend( &UARTBUFF[i], 1 ); //(uint8_t *)
	  }
	#endif


	#if 0
	  sprintf(UARTBUFF, "Angles: X, %02.5f, Xk, %02.5f, Xc: %02.5f, Y, %02.5f, dt, %f (ms)\r\n", AccXDeg,kalman_output,CompXState, AccYDeg, dT);
	  for( i = 0; UARTBUFF[i] !='\0'; i++){
		UARTSend( &UARTBUFF[i], 1 ); //(uint8_t *)
	  }
	#endif


	#if 1 //Filter Comparison
	  sprintf(UARTBUFF, "Filt Out: X, %02.5f, Xk, %02.5f, Xc: %02.5f\r\n", AccXDeg,kalman_output,CompXState);
	  for( i = 0; UARTBUFF[i] !='\0'; i++){
		  UARTSend( &UARTBUFF[i], 1 ); //(uint8_t *)
	  }
	#endif

#if 0 //Calibration Print, Gyro - X, Y, Z, Temp, Time
  sprintf(UARTBUFF, "CalPr_Gyr:,%d,%d,%d,%d,%d\r\n", gyrox, gyroy, gyroz, gyrot, timer32_0_counter);
  for( i = 0; UARTBUFF[i] !='\0'; i++){
	  UARTSend( &UARTBUFF[i], 1 ); //(uint8_t *)
  }
	delay(100); //Delay so the data logger can keep up
#endif

	//End UART Print


	#if 0 //LED Indicate Sensors are returning non zero values INDICATE_SENSORS_WORKING

	if ( gyrox==0 ) { //Yellow = Gyroscope
		GPIOSetValue(2,7,1); //Yel LED, Low (off)
	}
	else {
		GPIOSetValue(2,7,0); //Yel LED, Low (on)
	}

	if ( accx==0 ) { //Green = Accelerometer
		GPIOSetValue(2,8,1); //Grn LED, Low (off)
	}
	else {
		GPIOSetValue(2,8,0); //Grn LED, Low (on)
	}

	#endif //End LED Indicate Sensor Data


	 // printf("%u\n", LPC_GPIO2->DATA);
	  //GetGPIOBit(2, 4);
	  // Blink LED at 10Hz
/*	  if((timer32_0_counter%100)>50){
		  GPIOSetValue(0,7,1);
	  }else{
		  GPIOSetValue(0,7,0);
	  }



  // Polling Pin 2.4 test
	  if( (LPC_GPIO2 ->FIOPIN0) & 0x10){
	  GPIOSetValue(0,7,1);
  }else{
	  GPIOSetValue(0,7,0);
	  }
*/




	//printf("Acc T %d, X %d, Y %d, Z %d\n", acct, accx, accy, accz); // Prints the values into the console
	//trig = (accx);

	//printf("%g, %g\n", pow(accz, 2), trig);
	//printf("%g -debug: az - %d, sq - %g, sqrt - %d, div - %d\n", asin(accz/(sqrt(accz^2+accx^2))), accz, trig, sqrt((accz)^2+(accx)^2), accz/(sqrt(accz^2+accx^2)));






	//for ( i = 0; i < 0x100000; i++ ); //delay
	cyc++; // fix for overflows!
	//CycleCount++;
	//CycleCount=CycleCount%0xFFFFFFFF; //for the sake of overflow prevention
	//CycleMod1=CycleCount%1; // Zeros every 10 cycles
  } // end infinite loop

}


// END OF MAIN, Functions below! _________________________________________


/******************************************************************************
**                            End Of File
******************************************************************************/
