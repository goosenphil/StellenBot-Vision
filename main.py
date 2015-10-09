#!/usr/bin/env python2

# This program is for the robot to rush directly to the blocks, using the largest one found as reference
# After the block is obtained, it should rotate until it sees the starting marker and return the block
import cv2
import numpy as np
import time

import serial

s = serial.Serial('/dev/ttyACM3', 115200)
cap = cv2.VideoCapture(1)

try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    print "[+] Imported GPIO interface!"
except ImportError:
    print "[-]Cannot import GPIO interface"


def checkSwitch():
    if GPIO.input(18) is False:
        return True
    else:
        return False

# lastSend =
def send(byte1, byte2, byte3):
    s.write( chr(101) + chr(byte1)  + chr(byte2) + chr(byte3) )

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
    newTimeS = time.time() * 1000
    global stopTick

    if (newTimeS-stopTick > diff):
        stopTick = newTimeS
        return True

    return False

rotTick = 0.0
def checkMsRotate(diff):
    newTimeR = time.time() * 1000
    global rotTick

    if (newTimeR-rotTick  > diff):
        rotTick  = newTimeR
        return True

    return False

# ToDo: make the robot stop less often.
def doRobot(angle):
    tol = 3
    speed = (50+ abs(int(angle)))
    speed = int(speed)
    if speed > 90:
        speed = 75
    if speed < 70:
        speed = 75
    # print "[",str(speed),"]"
    if (tol > abs(angle) ): # Attempt to move forward when on line
        send(0, 1, 80)
        send(1, 1, 80)
    elif ( angle < -tol): # Attempt to move right towards line
        send(1,1,speed)
        send(1,1,speed-10)
    elif( angle > tol): # Attempt to move left towards line
        send(0,1,speed)
        send(1,1,speed-10)

def stopRobot():
    s.write( chr(101) + chr(1)  + chr(1) + chr(0) )
    s.write( chr(101) + chr(0)  + chr(1) + chr(0) )

# Keep on rotating (anti clockswise) if bock does not change in area or co-ordinates enough
toggleState = False
def rotateRobot():
    rotSpeed = 79
    send(1, 0, rotSpeed)
    send(1, 1, rotSpeed-10)
    if checkMsRotate(300):
        stopRobot()

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

def trackBlock(mask):
    _, contours, hierarchy = cv2.findContours(np.copy(mask), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    global blockVisible
    blockVisible = False
    contours = sorted(contours, key = cv2.contourArea, reverse = True)[:1] # Reduces contours to only top 5 based on area.
    cx, cy, conA = 0, 0, 0

    for c in contours:
        conA = cv2.contourArea(c)

    	if conA > 250:
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.02 * peri, True)

            if 4 <= len(approx) < 10: # Look for object within specific range of sides
                blockVisible = True
                centre = centroid(c)
                cx = centre[0]
                cy = centre[1]

    return cx, cy, conA

# Reads a single frame in order to determine the height and width of the image
_, test = cap.read()
height = test.shape[0]
width = test.shape[1]
midw = width/2
midh = height/2
font = cv2.FONT_HERSHEY_SIMPLEX

robotEnabled = True
blockVisible = False

lower_BGR = np.array([37, 99, 30])
upper_BGR = np.array([100,255,255])

while(1):
    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Threshold the HSV image to get only BGR colors
    checkSwitch()
    if gotBlock: # Set to beacon if block is found. We can also have a state reset colour?
        lower_BGR = np.array([9,150,150]) #Current beacon: Yellow screwdriver box
        upper_BGR = np.array([160,255,255])

    mask = cv2.inRange(hsv, lower_BGR, upper_BGR)
    angle = None
    if cv2.countNonZero(mask) > 200: # A fast way of checking for the block before performing further operations on it
        cx, cy, conA = trackBlock(mask)
        if cx != 0 and cx != 0 and conA != 0:
            angle = np.degrees(np.arctan(float(cx-midw)/float(cy)))


    if blockVisible and angle != None:
        blockVisible = False #Possibly use a counter instead to prevent single frame block loss
        print "block is visible!"
        if checkMs(10):
            doRobot(angle)
        # seekBlocks()
    else:
        rotateRobot();
        print "Block lost! Now rotating robot"

s.close()

# What needs to be done:
# testing the gpio interface, if reed is enabled long enough, return to base. Show it a reset colour card? Or if base is large enought, stop. Possibly place magnet into base station.
# improving the block tracking part, determine why it sometimes stands still. Make it into a more elegant solution that isn't as reliant on luck.
# Handling offscreen blocks, by guessing.
# Example of offscreen classification: (By using percentages of screen space.)
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
