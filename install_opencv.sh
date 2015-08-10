#!/bin/bash

# This install script is meant for linux

echo "================================================================================================================="
echo "This script will install the latest OpenCV from source with Python and C++ bindings."
echo "This script needs QT installed (0r remove -D WITH_QT=ON from build options)"
echo "Please note that OpenCV is a very large download, github access is free without inetkey so please close it."
echo "You may also require to download a few packages, please set your repository to ftp.sun.ac.za to download them for free"
echo "This can easily be selected from the ubuntu software centre"
read -n1 -p "This will clone and build the latest OpenCV in the current directory, press any key to continue... (Press Ctrl+C to exit)"

if [ -x "$(command -v apt-get)" ]; then
    echo "Looks like you are running a debian based distro like Ubuntu, now installing packages"
    sudo apt-get install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev gtk2-engines-pixbuf
else
    echo "Not running a debian based distro, please install the packages"
fi

echo "Now downloading OpenCV from github..."
git clone https://github.com/Itseez/opencv.git
git clone https://github.com/Itseez/opencv_contrib.git
cd opencv
mkdir build
cd build

echo "Done, now compiling. This might take very long..."
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON -D OPENCV_EXTRA_MODULES_PATH=~/projects/OpenCV3/opencv_contrib/modules -D PYTHON_INCLUDE_DIR=/usr/include/python2.7 -D PYTHON_INCLUDE_DIR2=/usr/include/x86_64-linux-gnu/python2.7 -D PYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython2.7.so ..
make -j2 #You might want to tweak this variable to a lower/higher amount depending on your CPU.
sudo make install

sudo echo '/usr/local/lib' > /etc/ld.so.conf.d/opencv.conf

sudo ldconfig

sudo echo 'PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig' >> '/etc/bash.bashrc'
sudo echo 'export PKG_CONFIG_PATH' >> '/etc/bash.bashrc'
