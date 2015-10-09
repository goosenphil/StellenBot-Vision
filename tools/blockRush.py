#!/usr/bin/env python2

# This program is for the robot to rush directly to the blocks, using the largest one found as reference
# After the block is obtained, it should rotate until it sees the starting marker and return the block
import cv2
import numpy as np
import time
import threading
import serial

try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    print "[+] Imported GPIO interface!"
except ImportError:
    print "[-]Cannot import GPIO interface"

# s = serial.Serial('/dev/pts/21', 115200)
s = serial.Serial('/dev/ttyACM3', 115200)

cap = cv2.VideoCapture(1)


gotBlock = False
def checkSwitch():
    return gotBlock
    # return  !GPIO.input(18)

# lastSend =
def send(byte1, byte2, byte3):
    s.write( chr(101) + chr(byte1)  + chr(byte2) + chr(byte3) )
    # print "SERIAL DISABLED!", str(byte1), str(byte2), str(byte3)
    pass

def nothing(x):
    pass

def drawFeatures(cx, cy, sides, conA):
    cv2.circle(drawFrame,(cx,cy), 5, (255,0,0), -1)
    cv2.putText(drawFrame,(str((cx,cy,conA))),(cx,cy), font, 1,(255,100,50),2,cv2.LINE_AA) # Puts co-ordinates of object
    cv2.putText(drawFrame,(str((midw-cx,height-cy,conA,sides))),(cx,cy+50), font, 1,(155,200,100),2,cv2.LINE_AA) #Distance from bottom centre, contour area and sides
    cv2.line(drawFrame, (cx,cy),(midw,height), (200,150,0), 5)

# ToDo: make the robot stop less often.
def doRobot(angle):
    tol = 3
    speed = (60+abs(int(angle)))
    if 70 > speed > 100:
        speed = 75
    # print "[",str(speed),"]"
    if ( 7.5 * tol > abs(angle)): # Attempt to move forward when on line
        cv2.putText(drawFrame,"|"+str(speed)+"|",(100,150), font, 1,(255,255,100),2,cv2.LINE_AA)
        send(0, 1, speed+int(angle))
        send(1, 1, speed-int(angle))
    elif ( angle < -tol/2): # Attempt to move right towards line
        cv2.putText(drawFrame,"<"+str(speed),(100,150), font, 1,(255,255,100),2,cv2.LINE_AA)
        send(1,1,speed)
        send(1,1,speed-10)
    elif( angle > tol/2): # Attempt to move left towards line
        cv2.putText(drawFrame,(str(speed)+">"),(100,150), font, 1,(255,255,100),2,cv2.LINE_AA)
        send(0,1,speed)
        send(1,1,speed-10)
    else:
        send(0, 1, 80)
        send(1, 1, 80)

"""
Do robot alternative
    offset is the corresponding methode to get the variables
    
def doRobot():
    offset, offset_gradient, visible = getOffset()
    print offset, offset_gradient, visible
    error = offset/abs(offset_gradient)
    gain = error/70
    tollerance_offset = 30

    threshold_speed = lambda x: x if 60<x<100 else 80
    speed = 80 #base speed
    speed_left =  threshold_speed(speed - gain)
    speed_right = threshold_speed(speed + gain)

    right_bais = 0
    left_bais = 0

    if(visible): #turns
        if (tollerance_offset < abs(offset) ): # Attempt to move forward when on line
            s.write( chr(101) + chr(0)  + chr(1) + chr(int(speed_right)))
            s.write( chr(101) + chr(1)  + chr(1) + chr(int(speed_left)))
        else:
            s.write( chr(101) + chr(0)  + chr(1) + chr(speed))
            s.write( chr(101) + chr(1)  + chr(1) + chr(speed))
    else:
"""

def stopRobot():
    # if checkMsStop(200): # Possibly disable me
    send(1,1,0)
    send(0,1,0)

# Keep on rotating (anti clockswise) if bock does not change in area or co-ordinates enough
toggleState = False
def rotateRobot():
    rotSpeed = 90
    send(1, 0, rotSpeed)
    send(1, 1, rotSpeed-10)
    if checkMsRotate(400):
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

def trackBlock():
    _, contours, hierarchy = cv2.findContours(np.copy(mask), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    global blockVisible
    global mask
    global drawFrame
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
angleViable = False
"""
def getOffset():
    cx, cy, conA = trackBlock()
    if cv2.countNonZero(mask) > 1000:
        if cx != 0 and cx != 0 and conA != 0:
             offset_x = average(int(cx-midw))
             offset_y = int(cy-midh)
             print "robot ready"
             offset_gradient = offset_x/offset_y
             print "no you do robot"
        return offset_gradient, offset, blockVisible
    print("failed")
"""

def getAngle():
    angle = None
    angleViable = False
    if cv2.countNonZero(mask) > 1000: # A fast way of checking for the block before performing further operations on it
        cx, cy, conA = trackBlock()
        if cx != 0 and cx != 0 and conA != 0:
            angle = np.degrees(np.arctan(float(cx-midw)/float(cy)))
            angleViable = True
    return angle

"""
please not we might have to make another global mask for seeking home
"""
mask = None
drawFrame = None

def timer(name, func, delay, cond):
    print "Timer: " + name + " Started"
    Lock.acquire()
    try:
        print name + " Has Acquired the lock"
        while cond(): # you can modify the sentinal [key press]
            time.sleep(delay)
            func()
    print name + " is releasing the lock"
    finally:
        Lock.release()
    print "Timer: " + name + " Completed"

t1 = threading.Thread(target=timer, args=("Timer1", doRobot(angle()), 0.02, (blockVisible and angleViable and robotEnabled)))
t2 = threading.Thread(target=timer, args=("Timer1", robotStop(), 0.02, (not robotEnabled)))
t3 = threading.Thread(target=timer, args=("Timer1", doRobot(angle()), 0.02, (not blockVisible and robotEnabled)))


while(1):
    _, frame = cap.read()
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    global drawFrame = np.copy(frame)

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
    if gotBlock: # Set to beacon if block is found. We can also have a state reset colour?
        lower_BGR = np.array([0,93,0]) #Current beacon: Yellow screwdriver box
        upper_BGR = np.array([33,255,255])

    mask = cv2.inRange(hsv, lower_BGR, upper_BGR)
    # print "NON"+str(cv2.countNonZero(mask))
    getAngle() #updates the states

    cv2.imshow('mask',mask)
    try:
        if(robotEnabled):
            if(blockVisible()):
                if(not t1.isAlive() and angleViable):
                    t1.start()
            else:
                if(not t3.isAlive()):
                    t3.start()
        else:
            if(not t2.isAlive()):
            t2.start()
    finally:
        stopRobot()

    if(threading.activeCount()<2): #because main is thread
        robot()

    cv2.imshow('contours', drawFrame)

    k = cv2.waitKey(10) & 0xFF
    if k == 27: #Checks if escape is pressed
        break
    if k == 32: # Press space to toggle robot
        if robotEnabled == False:
            robotEnabled = True
        else:
            stopRobot()
            robotEnabled = False
    if k == 97:
        print "BLOCK FOUND ACTIVATED. Remember to enable as GPIO"
        gotBlock = True

cv2.destroyAllWindows()
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
