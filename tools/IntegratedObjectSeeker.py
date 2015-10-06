#!/usr/bin/env python2

# This program is for the robot to rush directly to the blocks, using the largest one found as reference
# After the block is obtained, it should rotate until it sees the starting marker and return the block
import cv2
import numpy as np
import time

import serial

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

_, test = cap.read()
# test = cv2.transpose(test)
height = test.shape[0]
width = test.shape[1]
midw = width/2
midh = height/2
print midw,midh

di = 1
ie = 1
font = cv2.FONT_HERSHEY_SIMPLEX
robotEnabled = True
# cv2.resizeWindow('mask', 320, 320);

while(1):

    # Take each frame
    _, frame = cap.read()

    #Try denoising algorithm (ToDo)

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

    filter = np.copy(mask)

    filter = cv2.erode(filter, ke, iterations = ie)
    filter = cv2.dilate(filter, kd, iterations = di)

    # if (ie < 1 and id < 1): #Bug, cannot disable morpological operations
        # filter = np.copy(mask)
    cv2.imshow('filter', filter)
    cv2.imshow('mask',mask)

    f2 = np.copy(frame)

    #Find contours and draw them
    _, contours, hierarchy = cv2.findContours(filter, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    contours = sorted(contours, key = cv2.contourArea, reverse = True)[:5] # Reduces contours to only top 5 based on area.
    # cv2.drawContours(f2, contours, -1, (0,0,255), 3)
    objectVisible = False
    try:
    #Find centroids of contour moments (Refer to http://docs.opencv.org/master/dd/d49/tutorial_py_contour_features.html#gsc.tab=0)
        for c in contours: #Try dealing with similiar area
               	moments = cv2.moments(c)
                contourArea = cv2.contourArea(c)

                if contourArea > 200:
                    contourPerimeter = cv2.arcLength(c, True)
                    approx = cv2.approxPolyDP(c, 0.02 * contourPerimeter, True)

                if 4 <= len(approx) < 10: # Look for object within specific range of sides
                    objectVisible = True
                    if(mo['m00']):
                	       cx = int(mo['m10']/mo['m00'])

            	if checkMs(10) and cy > 0:
                    if robotEnabled:
                        drawFeatures(cx, cy, len(approx))
                        cv2.drawContours(f2, [approx], -1, (0, 200, 50), 3)
                        cv2.putText(f2,str(angle),(100,100), font, 1,(160,100,50),2,cv2.LINE_AA)
                        oftset = int(cx-midw)
                        angle = int(np.degrees(np.arctan(offset/int(cy)))) + 45 #angle offset 45
                        cv2.putText(f2,str(angle),(100,100), font, 1,(160,100,50),2,cv2.LINE_AA)
                        tollerance_angle = 1 #angle tollerance
                        tollerance_offset = 100 #absolute error tollerance

                        gain_angle = 1 * angle * 0.5
                        gain_offset = 0.0 * offset * 0.5

                        def threshold_speed(speed):
                            if(55<speed<90):
                                return 70
                            else:
                                return speed

                        speed = 50 #base speed
                        speed_left =  threshold_speed(speed - gain_angle - gain_error)
                        speed_right = threshold_speed(speed + gain_angle + gain_error)

                        if(objectVisible): #turns
                                if (tollorance_angle < abs(angle) or tollerance_error < abs(offset) ): # Attempt to move forward when on line
                                    s.write( chr(101) + chr(0)  + chr(1) + chr(speed_left))
                                    s.write( chr(101) + chr(1)  + chr(1) + chr(right_right))
                                else:
                                    s.write( chr(101) + chr(0)  + chr(1) + chr(75))
                                    s.write( chr(101) + chr(1)  + chr(1) + chr(75))
                        else:
                            s.write( chr(101) + chr(0) + chr(1) + chr(1) + chr(0))
                            s.write( chr(101) + chr(0) + chr(0) + chr(1) + chr(0))
    except:
        s.write( chr(101) + chr(0) + chr(1) + chr(1) + chr(0))
        s.write( chr(101) + chr(0) + chr(0) + chr(1) + chr(0))

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
            # stopRobot()
    # cv2.resizeWindow('mask', 800, 600);

cv2.destroyAllWindows()
s.close()
