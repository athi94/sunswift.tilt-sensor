#! /usr/bin/python
import serial
import io
import time
#print("hello")
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1, dsrdtr=True)
ser.flush()

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

while (1):
  print ser.readline().rstrip('\n')
