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

# lastSend =
def send(byte1, byte2, byte3):
    s.write( chr(101) + chr(byte1)  + chr(byte2) + chr(byte3) )
    # print "SERIAL DISABLED!", str(byte1), str(byte2), str(byte3)
    pass

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

stopTick = 0.0
def checkMsStop(diff):
    newTime = time.time() * 1000
    global stopTick

    if (newTime-stopTick > diff):
        stopTick = newTime
        return True

    return False

def drawFeatures(cx, cy, sides, conA):
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
    # print "[",str(speed),"]"
    if (tol > abs(angle) ): # Attempt to move forward when on line
        cv2.putText(drawFrame,"|"+str(speed)+"|",(100,150), font, 1,(255,255,100),2,cv2.LINE_AA)
        send(0, 1, 80)
        send(1, 1, 80)
    elif ( angle < -tol): # Attempt to move right towards line
        cv2.putText(drawFrame,"<"+str(speed),(100,150), font, 1,(255,255,100),2,cv2.LINE_AA)
        send(1,1,speed)
        send(1,1,speed-10)
    elif( angle > tol): # Attempt to move left towards line
        cv2.putText(drawFrame,(str(speed)+">"),(100,150), font, 1,(255,255,100),2,cv2.LINE_AA)
        send(0,1,speed)
        send(1,1,speed-10)

def stopRobot():
    if checkMsStop(100):
        send(1,1,0)
        send(0,1,0)

def rotateRobot():
    send(1, 0, 75)
    send(1, 1, 70)

def centroid(contour):
    mo = cv2.moments(contour)
    try:
        cx = int(mo['m10']/mo['m00'])
        cy = int(mo['m01']/mo['m00'])
        return (cx, cy)
    except:
        return 0, 0


def seekBlocks():
    rotateRobot()

def trackBlock(mask, drawFrame):
    _, contours, hierarchy = cv2.findContours(np.copy(mask), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    global blockVisible
    blockVisible = False
    contours = sorted(contours, key = cv2.contourArea, reverse = True)[:1] # Reduces contours to only top 5 based on area.
    cx, cy, conA = 0, 0, 0

    for c in contours:
        conA = cv2.contourArea(c)

    	if conA > 200:
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.02 * peri, True)

            if 4 <= len(approx) < 10: # Look for object within specific range of sides
                blockVisible = True
                centre = centroid(c)
                cx = centre[0]
                cy = centre[1]
                drawFeatures(cx, cy, len(approx), conA)
                cv2.drawContours(drawFrame, [approx], -1, (0, 200, 50), 3)

    return cx, cy, conA

cv2.namedWindow('mask', cv2.WINDOW_NORMAL)
cv2.namedWindow('contours', cv2.WINDOW_NORMAL)
cv2.resizeWindow('mask', 800, 600);
cv2.resizeWindow('contours', 800, 600);

#Lower BGR sliders
cv2.createTrackbar('Bl','mask',50,255,nothing)
cv2.createTrackbar('Gl','mask',102,255,nothing)
cv2.createTrackbar('Rl','mask',90,255,nothing)

#Upper BGR sliders
cv2.createTrackbar('Bu','mask',95,255,nothing)
cv2.createTrackbar('Gu','mask',255,255,nothing)
cv2.createTrackbar('Ru','mask',255,255,nothing)

# Reads a single frame in order to determine the height and width of the image
_, test = cap.read()
height = test.shape[0]
width = test.shape[1]
midw = width/2
midh = height/2
font = cv2.FONT_HERSHEY_SIMPLEX

robotEnabled = False
blockVisible = False

while(1):
    _, frame = cap.read()
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    drawFrame = np.copy(frame)

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
    # print "NON"+str(cv2.countNonZero(mask))

    #
    angle = None
    if cv2.countNonZero(mask) > 200: # A fast way of checking for the block before performing further operations on it
        cx, cy, conA = trackBlock(mask, drawFrame)
        if cx != 0 and cx != 0 and conA != 0:
            angle = np.degrees(np.arctan(float(cx-midw)/float(cy)))

    cv2.imshow('mask',mask)

    if blockVisible and angle != None:
        blockVisible = False
        print "block is visible!"
        if robotEnabled:
            if checkMs(10):
                doRobot(angle)
        # seekBlocks()
    elif not robotEnabled:
        stopRobot()
        print "No block found."
    else:
        # rotateRobot();
        print "Block lost!"
        stopRobot()
        pass


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
