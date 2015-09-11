#!/usr/bin/env python2

# A tool for selecting a specific colour for inRange thresholding in the HSV colour space
# now also with contour and moment centroid indication

#ToDo: replace wrong naming of colour space sliders (they actually represent HSV)

import cv2
import numpy as np

def nothing(x):
    pass

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

cap = cv2.VideoCapture(0)
cv2.namedWindow('mask')
cv2.namedWindow('filter')

#Lower BGR sliders
cv2.createTrackbar('Bl','mask',0,255,nothing)
cv2.createTrackbar('Gl','mask',0,255,nothing)
cv2.createTrackbar('Rl','mask',0,255,nothing)

#Upper BGR sliders
cv2.createTrackbar('Bu','mask',0,255,nothing)
cv2.createTrackbar('Gu','mask',0,255,nothing)
cv2.createTrackbar('Ru','mask',0,255,nothing)

cv2.createTrackbar('Se','filter',1,255,nothing)
cv2.createTrackbar('Ie','filter',1,255,nothing)

cv2.createTrackbar('Sd','filter',1,255,nothing)
cv2.createTrackbar('Id','filter',1,255,nothing)

resetSliders()

id = 1
ie = 1

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

    # it = 2

    filter = np.copy(mask)

    filter = cv2.erode(filter, ke, iterations = ie)
    filter = cv2.dilate(filter, kd, iterations = id)

    # if (ie < 1 and id < 1): #Bug, cannot disable morpological operations
        # filter = np.copy(mask)
    cv2.imshow('filter', filter)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= filter)
    cv2.imshow('result',res)

    # cv2.imshow('hsv', hsv)
    # cv2.imshow('frame',frame)
    cv2.imshow('mask',mask)

    f2 = np.copy(frame)

    #Find contours and draw them
    _, contours, hierarchy = cv2.findContours(filter, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(f2, contours, -1, (0,0,255), 3)

    #Find centroids of contour moments (Refer to http://docs.opencv.org/master/dd/d49/tutorial_py_contour_features.html#gsc.tab=0)
    maxArea = 0

    #Finds centre of largest contour area, using image moment centroid and draws it as a circle
    for c in contours: #Try dealing with similiar areas, using a a +- percentage change

        area = cv2.contourArea(c)

        if (area > maxArea):
            maxArea = area
            mo = cv2.moments(c)

            cx = int(mo['m10']/mo['m00'])
            cy = int(mo['m01']/mo['m00'])


    try:
        cv2.circle(f2,(cx,cy), 5, (255,0,0), -1)

    except:
        pass

    cv2.imshow('contours', f2)

    k = cv2.waitKey(5) & 0xFF
    if k == 27: #Checks if escape is pressed
        break
    if k == 32: # Check for spacebar (To reset sliders)
        resetSliders()

cv2.destroyAllWindows()
