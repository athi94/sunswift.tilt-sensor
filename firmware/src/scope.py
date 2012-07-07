import serial
from pylab import *
import re
from re import search,match
import sys
from time import sleep
import pdb


scopeLen = 101
ydata = []
xdata = []
front = 0
running = True

ion()
fig = figure()
p, = plot(xdata,ydata)
draw()
xlim(0,scopeLen-1)
ylim(-pi,pi)

#r = re.compile(r'([^,]+),([^,]+),([^,]+)')
r = re.compile(r'([\d.-]+),([\d.-]+),([\d.-]+)')

s = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
while True:
    l = s.readline()
    print l
    #m = search(r,l)
    #pdb.set_trace()
    #if m:
        #if front<scopeLen:
            #ydata.append(float(m.group(1)))
            #xdata.append(front)
            #print('"'+l+ '"' + ' ->%f %f' %(xdata[front],ydata[front]))
            #front+=1

        #else:
            ##go back to left hand side, zero everything
            #xdata = [float(m.group(1))]
            #ydata = [0]
            #front = 1
        
        #p.set_data(xdata,ydata)
        #draw()

s.close()
