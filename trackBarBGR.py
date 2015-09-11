#!/bin/env python2
# A program to select a colour for openCV's BGR format
# Is also used to display numerical values of keys pressed
import cv2
import numpy as np

def nothing(x):
    pass

# Create a black image, a window
img = np.zeros((300,512,3), np.uint8) #resolution (300x512) and 3 colour dimensions
cv2.namedWindow('image')

# create trackbars for color change
cv2.createTrackbar('B','image',0,255,nothing)
cv2.createTrackbar('G','image',0,255,nothing)
cv2.createTrackbar('R','image',0,255,nothing)

# create switch for ON/OFF functionality
switch = '0 : OFF \n1 : ON'

cv2.createTrackbar(switch, 'image',0,1,nothing)
cv2.setTrackbarPos(switch, 'image', 1)

# pk = 255 # When no key is pressed

while(1):
    cv2.imshow('image',img)
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break
    if ( k is not 255):
        print chr(k), '-->', str(k)  #Show character and integer of keypress

    # get current positions of four trackbars
    b = cv2.getTrackbarPos('B','image')
    g = cv2.getTrackbarPos('G','image')
    r = cv2.getTrackbarPos('R','image')
    s = cv2.getTrackbarPos(switch,'image')

    if s == 0:
        img[:] = 0
    else:
        img[:] = [b,g,r]

cv2.destroyAllWindows()
