import numpy as np
import cv2

cap = cv2.VideoCapture(0)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    # rows, cols, ch = frame.shape
    # M = cv2.getRotationMatrix2D((cols/2,rows/2),90,1)
    # frame = cv2.warpAffine(frame,M,(cols,rows))

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = np.transpose(gray)
    frame = cv2.transpose(frame)

    # Display the resulting frame
    cv2.imshow('frame',frame)
    cv2.imshow('gray', gray)
    # print "x"
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
