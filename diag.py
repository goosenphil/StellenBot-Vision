#!/usr/bin/env python2

# This is a diagnostic script to make sure that OpenCV is working on linux

try:
    import cv2
    print "[+] OpenCV installed! Version: " + str(cv2.__version__)
except ImportError:
    print "[-]You must have OpenCV installed"

try:
    import numpy as np
    print "[+] Numpy installed!"
except ImportError:
    print "[-]You must have numpy installed"

def check_install():    
    print "Checking if latest build of OpenCV is installed..."
    if (cv2.__version__[:1] == "3"):
        print "[+]Good, you're running the latest source build"
    else:
        print "[-]NOPE! You're running release: " + cv2.__version__

def test_image():
    print "Now displaying, press any key to exit"
    image = cv2.imread('flag.png')
    cv2.imshow("Image", image)
    print "Press any key to exit..."
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    print "Debug mode on, look in the comments on how to fix the problems."
    check_install()
    test_image()

# Notes:
# To fix the error: (python:20917): Gtk-WARNING **: Unable to locate theme engine in module_path: "pixmap", 
# install the gtk2-engines-pixbuf package
# installing numpy: sudo apt-get install python-pip python-dev && sudo pip install numpy
# For installing openCV, use script the script from the repo
