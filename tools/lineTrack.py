#!/usr/bin/env python2

import cv2
import numpy as np
from random import randint

def nothing(x):
    pass

def centroid(moments):
    try:
        return (int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00']))
    except:
        return 0,0
        # pass

def crop(inframe, x1, y1, x2, y2):
    tempFrame = np.copy(inframe) # Prevents drawing onto frame if cropping from same frame
    cv2.rectangle(frame,(x1,y1),(x2,y2), ( x1-y2, y1-y2, x2-y2 )  ,5) #The part that says ( x1-y2, y1-y2, x2-y2 ) is to create constant but differing colours for each rectangle based on positions
    return tempFrame[y1:y2, x1:x2]

cap = cv2.VideoCapture(1)
cv2.namedWindow('mask')
cv2.createTrackbar('thresh','mask',0,255,nothing)
cv2.createTrackbar('cutSize','frame',0,255,nothing)
cv2.createTrackbar('offset','frame',0,255,nothing)

_, test = cap.read()
# print test.shape

height = test.shape[0]
width = test.shape[1]
midw = width/2
midh = height/2
# print midw, midh

cutSize = 50
offset = 400

while(1):

    _, frame = cap.read()
    cv2.imshow('crop', crop(frame, midw-100,0, midw+100,height) )
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

    cuts = crop(mask, midw-100,offset, midw+100,offset+cutSize), crop(mask, midw-100,height-cutSize-offset, midw+100,height-offset)

    # corn1 = midw-100,offset
    # corn2 = midw-100,height-cutSize-offset
    # corner = np.array([[midw-100,offset], [midw-100,height-cutSize-offset]])
    corners = [(midw-100,offset), (midw-100,height-cutSize-offset)] # The top left corners of the rectangles
    # print corner

    cv2.circle(frame,corners[0], 5, (0,255,0), 5)
    cv2.circle(frame,corners[1], 5, (0,255,0), 5)
    # cv2.imshow('top', cuts[0])
    # cv2.imshow('bot', cuts[1])

    for a,corn in zip(cuts,corners):
        _, contours, hierarchy = cv2.findContours(np.copy(a), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for b in contours:
            if cv2.contourArea(b) > 0:
                cord = centroid(cv2.moments(contours[0]))
                # if cord != (0,0)
                # cv2.circle(frame,(cord[0],cord[1]), 5, (255,0,255), 5)
                cv2.circle(frame,(cord[0]+corn[0],cord[1]+corn[1]), 5, (255,0,255), 5)
                # cv2.circle(frame,(cord[0]+corn1[1],cord[1]+corn1[1]), 5, (255,0,255), 5)
                print "[",cord[0], cord[1], "]"
                cv2.drawContours(a, contours, -1, (255,0,0), 3)

    # cv2.imshow('top', cuts[0])
    # cv2.imshow('bot', cuts[1])
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

    cv2.imshow('Final frame',frame)

    # cv2.imshow('crop', crop(100,200,300,400, frame))

    k = cv2.waitKey(5) & 0xFF
    if k == 27: #Checks if escape is pressed
        break
    if k == 32: # Check for spacebar (To reset sliders)
        # resetSliders()
        print srandint(0,255)

cv2.destroyAllWindows()
