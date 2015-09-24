#!/usr/bin/env python2

import cv2
import numpy as np

def nothing(x):
    pass

def centroid(moments):
    return (int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00']))

def crop(x1, y1, x2, y2):
    tempFrame = np.copy(frame)
    cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,255),5)
    return tempFrame[y1:y2, x1:x2]

cap = cv2.VideoCapture(1)
cv2.namedWindow('mask')
cv2.createTrackbar('thresh','mask',0,255,nothing)

_, test = cap.read()
print test.shape

height = test.shape[0]
width = test.shape[1]
midw = width/2
midh = height/2
print midw, midh

while(1):

    # Take each frame
    _, frame = cap.read()
    cv2.imshow('crop', crop(midw-100,0, midw+100,height) )
    cv2.circle(frame,(midw,midh), 5, (0,255,0), 5)
    cv2.imshow('frame',frame)
    # cv2.imshow('crop', crop(100,200,300,400, frame))

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    threshVal = cv2.getTrackbarPos('thresh','mask')
    ret,mask = cv2.threshold(gray, threshVal, 255, cv2.THRESH_BINARY_INV)
    cv2.imshow('mask',mask)

    mask2 = np.copy(mask)
    _, contours, hierarchy = cv2.findContours(mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frame, contours, -1, (255,0,0), 3)

    # for a in contours:
        # if 1000 > cv2.contourArea(a) > 5:
            # print cv2.contourArea(a)
    # cv2.imshow('contours', mask2);

    # ke = np.ones((se,se),np.uint8)
    # kd = np.ones((di,di),np.uint8)
    # filter = np.copy(mask)
    #
    # filter = cv2.erode(filter, ke, iterations = ie)
    # filter = cv2.dilate(filter, kd, iterations = id)
    # cv2.imshow('filter', filter)

    cv2.imshow('contours',frame)

    # cv2.imshow('crop', crop(100,200,300,400, frame))

    k = cv2.waitKey(5) & 0xFF
    if k == 27: #Checks if escape is pressed
        break
    if k == 32: # Check for spacebar (To reset sliders)
        # resetSliders()
        print frame.shape[0], frame.shape[1]

cv2.destroyAllWindows()
