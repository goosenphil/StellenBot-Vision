import cv2
import numpy as np
import time

import serial
# s = serial.Serial('/dev/pts/21', 115200)
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

def stopRobot():
    s.write( chr(101) + chr(1)  + chr(1) + chr(0) )
    s.write( chr(101) + chr(0)  + chr(1) + chr(0) )
    
def doRobot(angle, offset, visible):
    gain_angle = 1 * angle * 0.5
    gain_offset = 0 * offset * 0.5

    tollorance_angle = 3
    tollorance_offset = 0
    
    threshold_speed = lambda x: x if 55<x<90 else 70
    
    speed = 50 #base speed
    speed_left =  threshold_speed(speed - gain_angle - gain_error)
    speed_right = threshold_speed(speed + gain_angle + gain_error)

    right_bais = 0
    left_bais = 0
    
    if(visible): #turns
        if (tollorance_angle < abs(angle) or tollerance_offset < abs(offset) ): # Attempt to move forward when on line
            s.write( chr(101) + chr(0)  + chr(1) + chr(speed_left))
            s.write( chr(101) + chr(1)  + chr(1) + chr(right_right))
        else:
            s.write( chr(101) + chr(0)  + chr(1) + chr(80))
            s.write( chr(101) + chr(1)  + chr(1) + chr(80))
    else:
        stopRobot()



def centroid(contour):
    mo = cv2.moments(c)
    cx = int(mo['m10']/mo['m00'])
    cy = int(mo['m01']/mo['m00'])
    return (cx, cy)

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

    cv2.imshow('mask',mask)

    drawFrame = np.copy(frame)

    #Find contours and draw them
    _, contours, hierarchy = cv2.findContours(np.copy(mask), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    contours = sorted(contours, key = cv2.contourArea, reverse = True)[:5] # Reduces contours to only top 5 based on area.
    blockVisible = False


    for c in contours:
        conA = cv2.contourArea(c)

    	if conA > 200:
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.02 * peri, True)
            if 4 <= len(approx) < 10: # Look for object within specific range of sides
                blockVisible = True

                try:
                    centre = centroid(c)
                    cx = centre[0]
                    cy = centre[1]
                    if checkMs(10) and cy > midh-midh:
                        if robotEnabled:
                            drawFeatures(cx, cy, len(approx))
                            cv2.drawContours(drawFrame, [approx], -1, (0, 200, 50), 3)
                            oftset = int(cx-midw)
                            angle = int(np.degrees(np.arctan(offset/int(cy)))) + 45 #angle offset 45
                            angle = np.degrees(np.arctan(float(cx-midw)/float(cy)))
                            doRobot(angle, offset, blockVisible)
                except:
                    stopRobot()

            else:
                stopRobot()

    if blockVisible is False:
        stopRobot()


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
