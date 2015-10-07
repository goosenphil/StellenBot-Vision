import numpy as np
import cv2
#creates haar for image data
block = cv2.CascadeClassifier('block_data.xml')
#captures an image from the camer
img = cv2.imread('sachin.jpg')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#detects image for haar
blocks = block.detectMultiScale(gray, 1.3, 5)

#itterates through the co-ordinates in block objects
for (x,y,w,h) in blocks:
    #draws rectangles arround the region of interest
    img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
