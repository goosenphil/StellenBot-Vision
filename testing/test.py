

import serial

​

s = serial.Serial('/dev/ttyACM3')

speed = 100

#s.open() #setting serial object automatically opens the port

s.write( chr(0)  + chr(0) + chr(speed) )
s.close()


