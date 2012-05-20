#! /usr/bin/python
import serial
import io
import time
import subprocess
from subprocess import Popen, PIPE, STDOUT
#print("hello")
DEV='/dev/ttyUSB0'
SPD=115200
ser = serial.Serial(DEV, SPD, timeout=1, dsrdtr=True)

ser.flush()
ser.flushInput()

# Bring the node offline
ser.setRTS(True)
ser.setDTR(True)
time.sleep(0.1)

# Bring it back online
ser.setRTS(False)
time.sleep(0.1)
ser.setDTR(False)
#ser.setRTS(False)
time.sleep(0.1)

#Default start state
input= 'se'
waitCycles = 1;
while (1):
  try:

    if waitCycles == 0:
      print 'Which mode would you like to enter?\n\
  se = View serial stream\n\
  rs = Reset the microcontroller\n\
  cf = Compile and Flash the code to the microcontroller\n\
  ex = Exit script\n\
Simply use ctrl+c to leave a mode or the program'

    if waitCycles<1:
      input = raw_input(">> ")
    waitCycles -= 1

    
    if input == 'ex':
      ser.flushInput()
      ser.close()
      exit()
		
    if input == 'se':
      ser.flushInput()
      while(1):
	try:
	  print ser.readline().rstrip('\n')
	except KeyboardInterrupt:
	  waitCycles=0;
	  print ''
	  break

    if input == 'rs':
      print 'Taking the node offline..'
      ser.setRTS(True)
      ser.setDTR(True)
      time.sleep(0.5)

      print 'Bring it back online..'
      ser.setRTS(False)
      time.sleep(0.5)
      ser.setDTR(False)
      #ser.setRTS(False)
      time.sleep(0.5)
      waitCycles=0;
      print ''
      
    if input == 'cf':
      #print subprocess.check_output(["make", "clean"])
      print subprocess.check_output(["make", 'clean'])
      print subprocess.check_output(["make"])
      ser.close()
      #print subprocess.check_output(["./home/charithjperera/code/tilt/utilities/lpc21isp"])
      print subprocess.check_output(["make", 'flash'])
      ser = serial.Serial(DEV, SPD, timeout=1, dsrdtr=True)
      #print subprocess.Popen(["make"], bufsize=1)
      waitCycles=1;
      input = 'rs'
      
      #print 'BLAAAARGHHH\n'
      #print subprocess.Popen(["../utilities/lpc21isp", 'bin', 'build/main.bin', '-control', DEV, SPD, '12000'], bufsize=1)
      
    #input = 'end'    
  
  except KeyboardInterrupt:
    print ' exiting'
    ser.flushInput()
    ser.close()
    exit()
    break
