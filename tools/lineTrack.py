#!/usr/bin/env python2

import cv2
import numpy as np

def nothing(x):
    pass

def centroid(moments):
    try:
        return int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])
    except:
        return 0,0

def crop(inFrame, x1, y1, x2, y2):
    cv2.rectangle(drawFrame,(x1,y1),(x2,y2), ( x1-y2, y1-y2, x2-y2 )  ,5) #The part that says ( x1-y2, y1-y2, x2-y2 ) is to create constant but differing colours for each rectangle based on positions
    return inFrame[y1:y2, x1:x2]

cap = cv2.VideoCapture(1)
cv2.namedWindow('mask')
cv2.namedWindow('draw')
cv2.createTrackbar('thresh','mask',55,255,nothing)
cv2.createTrackbar('cutSize','draw',40,255,nothing)
cv2.createTrackbar('offset','draw',400,500,nothing)

_, test = cap.read()

height = test.shape[0]
width = test.shape[1]
midw = width/2
midh = height/2

cutSize = 0
offset = 0
wait = 5
oldLine = [(0,0), (0,0)]

while(1):

    _, frame = cap.read()
    drawFrame = np.copy(frame)
    cv2.imshow('crop', crop(frame, midw-100,0, midw+100,height) )

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    threshVal = cv2.getTrackbarPos('thresh','mask')
    cutSize = cv2.getTrackbarPos('cutSize', 'draw')
    offset = cv2.getTrackbarPos('offset', 'draw')
    ret,mask = cv2.threshold(gray, threshVal, 255, cv2.THRESH_BINARY_INV)
    cv2.imshow('mask',mask)

    mask2 = np.copy(mask)

    cuts = crop(mask, midw-100,offset, midw+100,offset+cutSize), crop(mask, midw-100,height-cutSize-offset, midw+100,height-offset)

    corners = [(midw-100,offset), (midw-100,height-cutSize-offset)] # The top left corners of the rectangles

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

                if cord is not prev:
                    # print cord
                    prev = cord

    if len(line) == 2:
        cv2.line(drawFrame, line[0],line[1], (255,0,0), 5)

    if line != oldLine:
        oldLine = line
        try:
            if len(line) == 2:
                angle = np.degrees(np.arctan(-float(line[0][0]-line[1][0])/float(line[0][1]-line[1][1])))
                print "angle: ", str(angle), "line", str(line)
        except:
            pass

    cv2.imshow('top', cuts[0])
    cv2.imshow('bot', cuts[1])

    #ToDo: Possibly use morpological operations
    # ke = np.ones((se,se),np.uint8)
    # kd = np.ones((di,di),np.uint8)
    # filter = np.copy(mask)
    #
    # filter = cv2.erode(filter, ke, iterations = ie)
    # filter = cv2.dilate(filter, kd, iterations = id)
    # cv2.imshow('filter', filter)

    cv2.imshow('Source image, make sure I am unmodified!', frame) # Make sure this frame is unmodified at the end, to avoid conflicts in calculations
    cv2.imshow('draw',drawFrame)

    # cv2.imshow('crop', crop(100,200,300,400, frame))

    k = cv2.waitKey(wait) & 0xFF
    if k == 27: #Checks if escape is pressed
        break

cv2.destroyAllWindows()
