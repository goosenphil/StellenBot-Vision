#!/usr/bin/env python2
# Try to make baud rate higher
import serial
import time

left = 1
right = 1

def send(motor, speed):
    try:
        s.open()
    except:
        pass

    if speed > 0:
        s.write( chr(motor)  + chr(0) + chr(speed) )
    else:
        s.write( chr(motor)  + chr(1) + chr(speed) )
    s.close()

s = serial.Serial('/dev/ttyACM3', 115200)

speed = 100
s.write( chr(1)  + chr(1) + chr(speed) + chr(0)  + chr(1) + chr(speed) )

# for x in range(50):
#     s.write( chr(1) )
#     s.write( chr(1) )
#     s.write( chr(100) )
#     time.sleep(0.2)
#     s.write( chr(1) )
#     s.write( chr(1) )
#     s.write( chr(0) )
#     time.sleep(0.2)

# for x in range(10):
#     send(1,100)
#     send(0,100)
#     time.sleep(0.2)
#     send(1,0)
#     send(0,0)
#     time.sleep(0.2)


#speed = 70
# s.write( chr(1)  + chr(1) + chr(speed) + chr(0)  + chr(1) + chr(speed) )
#s.write( chr(101) + chr(2)  + chr(1) + chr(1))
# s.write(chr(0))
s.close()
# send(1, 0)
# send(0, 0)
