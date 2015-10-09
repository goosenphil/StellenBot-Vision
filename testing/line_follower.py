#!/usr/bin/env python2

# A tool for selecting a specific colour for inRange thresholding in the HSV colour space
# now also with contour and moment centroid indication

# This still needs a lot of work.
# Here are some possible approaches to get the black line:
# 1. Canny edge detection -> Hough transform
# 2. Greyscale -> Threshold

import cv2
import numpy as np

def auto_canny(image, sigma=0.33):
	# compute the median of the single channel pixel intensities
	v = np.median(image)

	# apply automatic Canny edge detection using the computed median
	lower = int(max(0, (1.0 - sigma) * v))
	upper = int(min(255, (1.0 + sigma) * v))
	edged = cv2.Canny(image, lower, upper)

	# return the edged image
	return edged

def nothing(x):
    pass

def resetSliders(): #Sets sliders to their default positions
    cv2.setTrackbarPos('minCan', 'canny', 0)
    cv2.setTrackbarPos('maxCan', 'canny', 255)
    cv2.setTrackbarPos('appSize', 'canny', 10)

cap = cv2.VideoCapture(0)
cv2.namedWindow('canny')

#Lower BGR sliders
cv2.createTrackbar('minCan','canny',0,255,nothing)
cv2.createTrackbar('maxCan','canny',0,255,nothing)
cv2.createTrackbar('appSize','canny',3,255,nothing)


resetSliders()

appSize = 3

while(1):

    # Take each frame
    _, frame = cap.read()
    cv2.imshow('frame', frame)
    gr = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)



    blur = cv2.GaussianBlur(gr, (3,3), 0)
    autocan = auto_canny(blur)
    cv2.imshow('Autocan', autocan)


    minCan = cv2.getTrackbarPos('minCan','canny')
    maxCan = cv2.getTrackbarPos('maxCan','canny')

    appSize = cv2.getTrackbarPos('appSize','canny') #App size needs to be odd

    # if (appSize%2 == 0):
        # appSize += 1
        # cv2.setTrackbarPos('appSize', 'canny', appSize) # Does not fix it

    #ToDo: Add canny edge and hough line
    # http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_imgproc/py_canny/py_canny.html
    # http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_imgproc/py_houghlines/py_houghlines.html

    # edge = cv2.Canny(gr, minCan, maxCan, apertureSize = 3)
    edge = autocan

    lines = cv2.HoughLines(edge, 1, np.pi/180, 200)

    hueNorm = np.copy(frame)
    try:

        for rho, the, in lines[0]: #Polar conversion
            a = np.cos(the)
            b = np.sin(the)

            x0 = a*rho
            y0 = b*rho

            x1 = int(x0 + 1000*(-b))
            x1 = int(y0 + 1000*(a))
            x1 = int(x0 + 1000*(-b))
            x1 = int(y0 - 1000*(a))

            cv2.line(hueNorm, (x1,y1), (x2,y2), (0,0,255), 5)

        # cv2.imshow('lines', frame)
    except:
        pass

    #Using probalistic HoughLines
    try:
        hueProb = np.copy(frame)
        minLineLength = 100
        maxLineGap = 10
        lines = cv2.HoughLinesP(edge,1,np.pi/180,100,minLineLength,maxLineGap)
        for x1,y1,x2,y2 in lines[0]:
            cv2.line(hueProb,(x1,y1),(x2,y2),(0,255,0),2)

    except:
        pass

    cv2.imshow('hueNorm', hueNorm)
    cv2.imshow('probHue', hueProb)
    cv2.imshow('canny', edge)

    k = cv2.waitKey(appSize) & 0xFF
    if k == 27: #Checks if escape is pressed
        break
    if k == 32: # Check for spacebar (To reset sliders)
        resetSliders()

cv2.destroyAllWindows()
