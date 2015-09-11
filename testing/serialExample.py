#!/usr/bin/env python2
import serial

s = serial.Serial('/dev/robot')
speed = 0
#s.open()
def send(
s.write( chr(0)  + chr(0) + chr(speed) )
s.write( chr(1)  + chr(1) + chr(speed) )
s.close()
