#!/usr/bin/env python2

# This script can make robot follow a black line.
# Press spacebar to toggle robot ON/OFF (OFF at start)

import cv2
import numpy as np
import serial
import time
import sys

sys.path.insert(0, '../interface')
import RobotModel
import RobotSerial
print "Imported serial!"

# An empty function to allow sliders
def nothing(x):
    pass

# Calculates the centorid of a shape given.
def centroid(moments):
    try:
        return int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])
    except:
        return 0,0

# Crops a rectangle out of the input frame and draws where it was cut from on top of drawFrame
def crop(inFrame, x1, y1, x2, y2):
    cv2.rectangle(drawFrame,(x1,y1),(x2,y2), ( x1-y2, y1-y2, x2-y2 )  ,5) #The part that says ( x1-y2, y1-y2, x2-y2 ) is to create constant (across frames, no flickering) but differing colours for each rectangle based on positions
    return inFrame[y1:y2, x1:x2]

# Checks if a certain amount of milliseconds has passed since it's last call
# Use this function to create non-blocking wait states in the while loop
# Thus you can have delayed actions within the main loop without dropping frames
tick = 0.0
def checkMs(diff):
    newTime = time.time() * 1000
    global tick

    if (newTime-tick > diff):
        tick = newTime
        return True

    return False

# Causes the robot to perform actions
lastAction = (0,0,0)
rm = RobotModel.RobotModel()
ss = RobotSerial.SerialSession(rm, '/dev/ttyACM3', 115200)
# ToDo: Make speed proportionate to angle
def doRobot(tolerance, angle, speed):
    global lastAction
    global rm
    global ss
    if lastAction != (tolerance, angle, speed):
        lastAction = (tolerance, angle, speed)
        if (-tolerance < angle < tolerance): # Attempt to move forward when on line
            rm.speedLeft = speed
            rm.speedRight = speed
            print "______________"
        if ( angle < -tolerance): # Attempt to move right towards line
            print "<<<<<<<<<<<<<<"
            rm.speedLeft = speed-10
            rm.speedRight = speed
        if( angle > tolerance): # Attempt to move left towards line
            print ">>>>>>>>>>>>>>"
            # rm.speedLeft = speed
            rm.speedLeft = speed
            rm.speedRight = speed-10

        ss.updateRobotState(rm)

# Stops the robot from moving
def stopRobot():
    rm.SpeedLeft = 0
    rm.SpeedRight = 0
    # pass


cap = cv2.VideoCapture(1)
cv2.namedWindow('mask')
cv2.namedWindow('draw')
cv2.createTrackbar('thresh','mask',42,255,nothing)
cv2.createTrackbar('cutHeight','draw',40,255,nothing)
cv2.createTrackbar('offset','draw',400,500,nothing)

_, test = cap.read()
test = cv2.transpose(test)
height = test.shape[0]
width = test.shape[1]
midw = width/2
midh = height/2

cutHeight = 0
cutWidth = 300
offset = 0
wait = 10
oldLine = [(0,0), (0,0)]
robotEnabled = False

tolerance = 30
speed = 70

# The main function
while(1):

    _, frame = cap.read()
    frame = cv2.transpose(frame)
    drawFrame = np.copy(frame)
    # cv2.imshow('crop', crop(frame, midw-cutWidth,0, midw+cutWidth,height) )

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    threshVal = cv2.getTrackbarPos('thresh','mask')
    cutHeight = cv2.getTrackbarPos('cutHeight', 'draw')
    offset = cv2.getTrackbarPos('offset', 'draw')
    ret,mask = cv2.threshold(gray, threshVal, 255, cv2.THRESH_BINARY_INV)
    cv2.imshow('mask',mask)

    mask2 = np.copy(mask)

    # morpological operations, uncomment below to apply filtering
    # ke = np.ones((10,10),np.uint8)
    # kd = np.ones((3,3),np.uint8)
    # filter = np.copy(mask)
    # filter = cv2.erode(filter, ke, iterations = 5)
    # filter = cv2.dilate(filter, kd, iterations = 2)
    # cv2.imshow('filter', filter)
    # mask = filter

    cuts = crop(mask, midw-cutWidth,offset, midw+cutWidth,offset+cutHeight), crop(mask, midw-cutWidth,height-cutHeight-offset, midw+cutWidth,height-offset)

    corners = [(midw-cutWidth,offset), (midw-cutWidth,height-cutHeight-offset)] # The top left corners of the rectangles

    #ToDo, try to remove 'jumping', point cannot traverse amount of units per amount of frames
    prev = (0,0)
    line = []
    for a,corn in zip(cuts,corners):
        _, contours, hierarchy = cv2.findContours(np.copy(a), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for b in contours:
            if cv2.contourArea(b) > 10:
                cord = centroid(cv2.moments(contours[0]))
                if cord != (0,0):
                    pos = (cord[0]+corn[0],cord[1]+corn[1])
                    cv2.circle(drawFrame,(pos), 5, (0,0,255), 5)
                    line.append(pos)

                if cord != prev:
                    prev = cord

    if len(line) == 2:
        cv2.line(drawFrame, line[0],line[1], (255,0,0), 5)

    if line != oldLine:
        oldLine = line
        try:
            if len(line) == 2: # If two points are found representing line
                angle = np.degrees(np.arctan(-float(line[0][0]-line[1][0])/float(line[0][1]-line[1][1])))
                # print "angle: ", str(angle), "line", str(line)

                if robotEnabled:
                    if checkMs(100):
                        print angle
                        doRobot(tolerance, angle, speed)

            # else:
                # stopRobot()
        except:
            pass

    # cv2.imshow('crop', crop(frame, 100,200,300,400)) # Demonstrates the cropping function visually
    # cv2.imshow('Source image, make sure I am unmodified!', frame) # Make sure this frame is unmodified at the end, to avoid conflicts in calculations
    cv2.imshow('draw',drawFrame)


    # if checkMs(10): # Demonstrates non-blocking wait
        # print tick/1000

    k = cv2.waitKey(wait) & 0xFF
    if k == 27: #Checks if escape is pressed
        break
    if k == 32: # Press space to toggle robot
        s.flush()
        if robotEnabled == False:
            robotEnabled = True
        else:
            robotEnabled = False
            stopRobot()

cv2.destroyAllWindows()
