#!/usr/bin/env python2

# This program is for the robot to rush directly to the blocks, using the largest one found as reference
# After the block is obtained, it should rotate until it sees the starting marker and return the block
import cv2
import numpy as np
import time

import serial
s = serial.Serial('/dev/ttyACM3', 115200)

# try:
#     # import sys
#     # sys.path.insert(0, '../interface')
#     import RobotModel
#     import RobotSerial
#     print "[+] Imported robot interface!"
# except ImportError:
#     print "[-]Cannot import robot interface"

cap = cv2.VideoCapture(1)

def nothing(x):
    pass

tick = 0.0
def checkMs(diff):
    newTime = time.time() * 1000
    global tick

    if (newTime-tick > diff):
        tick = newTime
        return True

    return False

# Causes the robot to perform actions
# lastAction = (0,0,0)
# rm = RobotModel.RobotModel()
# ss = RobotSerial.SerialSession(rm, '/dev/ttyACM3', 115200)
# ToDo: Make speed proportionate to angle
# def doRobot(tolerance, angle, speed):
#     global lastAction
#     global rm
#     global ss
#     if True:
#         lastAction = (tolerance, angle, speed)
#         speed = -1*(speed + abs(angle))
#         if (-tolerance < angle < tolerance): # Attempt to move forward when on line
#             rm.speedLeft = speed
#             rm.speedRight = speed
#             print "______________"
#         if ( angle < -tolerance): # Attempt to move right towards line
#             print "<<<<<<<<<<<<<<"
#             rm.speedLeft = speed+10
#             rm.speedRight = speed
#         if( angle > tolerance): # Attempt to move left towards line
#             print ">>>>>>>>>>>>>>"
#             # rm.speedLeft = speed
#             rm.speedLeft = speed
#             rm.speedRight = speed+10
#
#     ss.updateRobotState(rm)

def resetSliders(): #Sets sliders to their default positions
    cv2.setTrackbarPos('Bl', 'mask', 0)
    cv2.setTrackbarPos('Gl', 'mask', 0)
    cv2.setTrackbarPos('Rl', 'mask', 0)
    cv2.setTrackbarPos('Bu', 'mask', 255)
    cv2.setTrackbarPos('Gu', 'mask', 255)
    cv2.setTrackbarPos('Ru', 'mask', 255)
    cv2.setTrackbarPos('Se', 'filter', 1)
    cv2.setTrackbarPos('Ie', 'filter', 1)
    cv2.setTrackbarPos('Sd', 'filter', 1)
    cv2.setTrackbarPos('Id', 'filter', 1)

def drawFeatures(cx, cy, sides):
    cv2.circle(f2,(cx,cy), 5, (255,0,0), -1)
    cv2.putText(f2,(str((cx,cy,conA))),(cx,cy), font, 1,(255,100,50),2,cv2.LINE_AA) # Puts co-ordinates of object
    cv2.putText(f2,(str((midw-cx,height-cy,conA,sides))),(cx,cy+50), font, 1,(155,200,100),2,cv2.LINE_AA) #Distance from bottom centre, contour area and sides
    cv2.line(f2, (cx,cy),(midw,height), (200,150,0), 5)

# cap.set(640, 480)
cv2.namedWindow('mask', cv2.WINDOW_NORMAL)
cv2.namedWindow('filter', cv2.WINDOW_NORMAL)
cv2.namedWindow('contours', cv2.WINDOW_NORMAL)
cv2.resizeWindow('mask', 800, 600);
cv2.resizeWindow('contours', 800, 600);

#Lower BGR sliders
cv2.createTrackbar('Bl','mask',76,255,nothing)
cv2.createTrackbar('Gl','mask',102,255,nothing)
cv2.createTrackbar('Rl','mask',90,255,nothing)

#Upper BGR sliders
cv2.createTrackbar('Bu','mask',95,255,nothing)
cv2.createTrackbar('Gu','mask',255,255,nothing)
cv2.createTrackbar('Ru','mask',255,255,nothing)

cv2.createTrackbar('Se','filter',1,255,nothing)
cv2.createTrackbar('Ie','filter',1,255,nothing)

cv2.createTrackbar('Sd','filter',1,255,nothing)
cv2.createTrackbar('Id','filter',1,255,nothing)

# resetSliders()

# Reads a single frame in order to determine the height and width of the image
_, test = cap.read()
height = test.shape[0]
width = test.shape[1]
midw = width/2
midh = height/2
print midw,midh

di = 1
ie = 1
font = cv2.FONT_HERSHEY_SIMPLEX
robotEnabled = False

while(1):
    _, frame = cap.read()

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    bl = cv2.getTrackbarPos('Bl','mask')
    gl = cv2.getTrackbarPos('Gl','mask')
    rl = cv2.getTrackbarPos('Rl','mask')
    bu = cv2.getTrackbarPos('Bu','mask')
    gu = cv2.getTrackbarPos('Gu','mask')
    ru = cv2.getTrackbarPos('Ru','mask')

    # define range of BGR color in HSV
    lower_BGR = np.array([bl,gl,rl])
    upper_BGR = np.array([bu,gu,ru])
    # Threshold the HSV image to get only BGR colors
    mask = cv2.inRange(hsv, lower_BGR, upper_BGR)

    #ToDo, add morpological operations, find contours and moments
    se = cv2.getTrackbarPos('Se','filter')
    ie = cv2.getTrackbarPos('Ie','filter')

    sd = cv2.getTrackbarPos('Sd','filter')
    di = cv2.getTrackbarPos('Id','filter')

    # print(se)

    ke = np.ones((se,se),np.uint8)
    kd = np.ones((di,di),np.uint8)

    # it = 2

    filter = np.copy(mask)

    filter = cv2.erode(filter, ke, iterations = ie)
    filter = cv2.dilate(filter, kd, iterations = di)

    cv2.imshow('filter', filter)

    # Bitwise-AND mask and original image
    # res = cv2.bitwise_and(frame,frame, mask= filter)
    # cv2.imshow('result',res)
    # cv2.imshow('hsv', hsv)
    # cv2.imshow('frame',frame)

    cv2.imshow('mask',mask)

    f2 = np.copy(frame)

    #Find contours and draw them
    _, contours, hierarchy = cv2.findContours(filter, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    contours = sorted(contours, key = cv2.contourArea, reverse = True)[:5] # Reduces contours to only top 5 based on area.
    blockVisible = True


    for c in contours:
        mo = cv2.moments(c)
        conA = cv2.contourArea(c)

    	if conA > 200:
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.02 * peri, True)

            if 4 <= len(approx) < 10: # Look for object within specific range of sides
                blockVisible = False

                try:
                    cx = int(mo['m10']/mo['m00'])
                    cy = int(mo['m01']/mo['m00'])
                    if checkMs(10) and cy > midh-midh:
                        if robotEnabled:
                            drawFeatures(cx, cy, len(approx))
                            cv2.drawContours(f2, [approx], -1, (0, 200, 50), 3)
                            angle = np.degrees(np.arctan(float(cx-midw)/float(cy)))
                            cv2.putText(f2,str(angle),(100,100), font, 1,(160,100,50),2,cv2.LINE_AA)
                            tol = 3
                            speed = (50+ abs(int(angle)))
                            speed = int(speed)
                            if speed > 90:
                                speed = 70
                            if speed < 70:
                                speed = 70
                            print "[",str(speed),"]"

                            if (tol > abs(angle) ): # Attempt to move forward when on line
                                cv2.putText(f2,"|"+str(speed)+"|",(100,150), font, 1,(255,255,100),2,cv2.LINE_AA)
                                s.write( chr(101) + chr(0)  + chr(1) + chr(80) )
                                s.write( chr(101) + chr(1)  + chr(1) + chr(80) )
                                print "______________"
                            elif ( angle < -tol): # Attempt to move right towards line
                                print "<<<<<<<<<<<<<<"
                                cv2.putText(f2,"<"+str(speed),(100,150), font, 1,(255,255,100),2,cv2.LINE_AA)
                                s.write( chr(101) + chr(1)  + chr(1) + chr(speed) )
                                s.write( chr(101) + chr(0)  + chr(1) + chr(speed-10) )
                            elif( angle > tol): # Attempt to move left towards line
                                # rm.speedLeft = speed
                                cv2.putText(f2,(str(speed)+">"),(100,150), font, 1,(255,255,100),2,cv2.LINE_AA)
                                print ">>>>>>>>>>>>>>"
                                s.write( chr(101) + chr(0)  + chr(1) + chr(speed) )
                                s.write( chr(101) + chr(1)  + chr(1) + chr(speed-10) )


                            # ss.updateRobotState(rm)


                except:
                    pass


            else:
                s.write( chr(101) + chr(1)  + chr(1) + chr(0) )
                s.write( chr(101) + chr(0)  + chr(1) + chr(0) )

    if blockVisible is True:
        s.write( chr(101) + chr(1)  + chr(1) + chr(0) )
        s.write( chr(101) + chr(0)  + chr(1) + chr(0) )


    cv2.imshow('contours', f2)

    k = cv2.waitKey(10) & 0xFF
    if k == 27: #Checks if escape is pressed
        break
    if k == 'r': # Check for spacebar (To reset sliders)
        resetSliders()
    if k == 32: # Press space to toggle robot
        if robotEnabled == False:
            robotEnabled = True
        else:
            s.write( chr(101) + chr(1)  + chr(1) + chr(0) )
            s.write( chr(101) + chr(0)  + chr(1) + chr(0) )
            robotEnabled = False

cv2.destroyAllWindows()
s.close()
