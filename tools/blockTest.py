#!/usr/bin/env python2

# This program is for the robot to rush directly to the blocks, using the largest one found as reference
# After the block is obtained, it should rotate until it sees the starting marker and return the block
import cv2
import numpy as np
import time

import serial
# s = serial.Serial('/dev/pts/21', 115200)
s = serial.Serial('/dev/ttyACM0', 115200)

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

def resetSliders(): #Sets sliders to their default positions
    cv2.setTrackbarPos('Bl', 'mask', 0)
    cv2.setTrackbarPos('Gl', 'mask', 0)
    cv2.setTrackbarPos('Rl', 'mask', 0)
    cv2.setTrackbarPos('Bu', 'mask', 255)
    cv2.setTrackbarPos('Gu', 'mask', 255)
    cv2.setTrackbarPos('Ru', 'mask', 255)

def drawFeatures(cx, cy, sides):
    cv2.circle(drawFrame,(cx,cy), 5, (255,0,0), -1)
    cv2.putText(drawFrame,(str((cx,cy,conA))),(cx,cy), font, 1,(255,100,50),2,cv2.LINE_AA) # Puts co-ordinates of object
    cv2.putText(drawFrame,(str((midw-cx,height-cy,conA,sides))),(cx,cy+50), font, 1,(155,200,100),2,cv2.LINE_AA) #Distance from bottom centre, contour area and sides
    cv2.line(drawFrame, (cx,cy),(midw,height), (200,150,0), 5)

def doRobot(angle):
    tol = 3
    speed = (60+ abs(int(angle)))
    speed = int(speed)
    if speed > 90:
        speed = 75
    if speed < 70:
        speed = 75
    print "[",str(speed),"]"
    if (tol > abs(angle) ): # Attempt to move forward when on line
        cv2.putText(drawFrame,"|"+str(speed)+"|",(100,150), font, 1,(255,255,100),2,cv2.LINE_AA)
        send(0, 1, 80)
        send(1, 1, 80)
        print "______________"
    elif ( angle < -tol): # Attempt to move right towards line
        print "<<<<<<<<<<<<<<"
        cv2.putText(drawFrame,"<"+str(speed),(100,150), font, 1,(255,255,100),2,cv2.LINE_AA)
        send(1,1,speed)
        send(1,1,speed-10)
    elif( angle > tol): # Attempt to move left towards line
        # rm.speedLeft = speed
        cv2.putText(drawFrame,(str(speed)+">"),(100,150), font, 1,(255,255,100),2,cv2.LINE_AA)
        print ">>>>>>>>>>>>>>"
        send(0,1,speed)
        send(1,1,speed-10)

def stopRobot():
    send(1,1,0)
    send(0,1,0)

def centroid(contour):
    mo = cv2.moments(c)
    cx = int(mo['m10']/mo['m00'])
    cy = int(mo['m01']/mo['m00'])
    return (cx, cy)

def send(byte1, byte2, byte3):
    s.write( chr(101) + chr(byte1)  + chr(byte2) + chr(byte3) )

def doStuff():
    # print "mooooooooooooooooooooooooooooo"
    for c in contours:
        conA = cv2.contourArea(c)
        # print "2222222222222222222222222"

    	if conA > 200:
            # print "33333333333333333ooo"
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.02 * peri, True)

            if 4 <= len(approx) < 10: # Look for object within specific range of sides
                blockVisible = True
                print "44444444444444444"

                try:
                    centre = centroid(c)
                    print "5234234234324324"
                    cx = centre[0]
                    cy = centre[1]
                    print "7777777777777777777777777"
                    if checkMs(10):
                        print "555555555555555555"
                        if robotEnabled:
                            print "66666666666666666666666"
                            drawFeatures(cx, cy, len(approx))
                            cv2.drawContours(drawFrame, [approx], -1, (0, 200, 50), 3)
                            angle = np.degrees(np.arctan(float(cx-midw)/float(cy)))
                            doRobot(angle)


                except:
                    pass


            else:
                stopRobot()

cv2.namedWindow('mask', cv2.WINDOW_NORMAL)
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

# resetSliders()

# Reads a single frame in order to determine the height and width of the image
_, test = cap.read()
height = test.shape[0]
width = test.shape[1]
midw = width/2
midh = height/2
print midw,midh

# cx = 0
# cy = 0

font = cv2.FONT_HERSHEY_SIMPLEX
robotEnabled = False

#Todo: States, reading reed switch. Return to homing beacon

while(1):
    _, frame = cap.read()
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # print str(s.inWaiting())+"***"

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

    cv2.imshow('mask',mask)
    drawFrame = np.copy(frame)

    #Find contours and draw them
    _, contours, hierarchy = cv2.findContours(np.copy(mask), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key = cv2.contourArea, reverse = True)[:1] # Reduces contours to only top 5 based on area.
    blockVisible = False

    doStuff()

    # if blockVisible is False and (cx, cy) != (0, 0) and robotEnabled:
    #     stopRobot()
    #     # print int(height*0.9), int(width*0.2), int(width*0.8)
    #     # print cy, cx
    #     # print "mooo"
    #     if cy > int(height*0.95):
    #         # checkMs(10) # Resets the counter of the timer
    #         if cx < int(width*0.2):
    #             send(1, 1, 65)
    #             send(0, 1, 75)
    #         elif cx > int(width*0.8):
    #             send(1, 1, 75)
    #             send(0, 1, 65)
    #             # pass
    #         else:
    #             send(1, 1, 75)
    #             send(0, 1, 75)
                # pass
            # guessBlock = True
            # while(guessBlock):
            #     cv2.putText(drawFrame,"WAITING...",(200,200), font, 1,(255,100,255),2,cv2.LINE_AA)
            #     # cv2.putText(image,"Hello World!!!", (x,y), cv2.FONT_HERSHEY_SIMPLEX, 2, 255)
            #     print "WAITING..."
            #     if checkMs(1000):
            #         guessBlock = False
            #         send(0, 1, 0)
            #         send(1, 1, 0)
            #         cx = 0
            #         cy = 0

    # elif blockVisible is False and robotEnabled:
    #     cv2.putText(drawFrame,"ROTATING",(200,200), font, 1,(255,100,255),2,cv2.LINE_AA)
        # stopRobot()
        # send(1, 0, 90)
        send(1, 0, 65)
        send(1, 1, 65)

    # elif blockVisible is False:
        # stopRobot()




    cv2.imshow('contours', drawFrame)

    k = cv2.waitKey(10) & 0xFF
    if k == 27: #Checks if escape is pressed
        break
    if k == 'r': # Check for spacebar (To reset sliders)
        resetSliders()
    if k == 32: # Press space to toggle robot
        if robotEnabled == False:
            robotEnabled = True
        else:
            stopRobot()
            robotEnabled = False

cv2.destroyAllWindows()
s.close()
