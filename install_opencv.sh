#!/bin/bash

echo " This script needs QT installed (0r remove -D WITH_QT=ON from build options)"
read -n1 -p "This will clone and build the latest OpenCV in the current directory, press any key to continue..."


git clone https://github.com/Itseez/opencv.git
git clone https://github.com/Itseez/opencv_contrib.git
cd opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON -D OPENCV_EXTRA_MODULES_PATH=~/projects/OpenCV3/opencv_contrib/modules -D PYTHON_INCLUDE_DIR=/usr/include/python2.7 -D PYTHON_INCLUDE_DIR2=/usr/include/x86_64-linux-gnu/python2.7 -D PYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython2.7.so ..
make -j4 #You might want to tweak this variable to a lower amount depending on your CPU
sudo make install

sudo echo '/usr/local/lib' > /etc/ld.so.conf.d/opencv.conf

sudo ldconfig

sudo echo 'PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig' >> '/etc/bash.bashrc'
sudo echo 'export PKG_CONFIG_PATH' >> '/etc/bash.bashrc'
