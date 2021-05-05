#!/bin/bash

set -e  # exit on error

# Install dependencies
sudo apt-get install libglew-dev
sudo apt-get install libpython2.7-dev
git submodule init && git submodule update
sudo python -mpip install numpy pyopengl Pillow pybind11


TMP_PATH=/tmp
PANGOLIN_PATH=${TMP_PATH}/Pangolin
PANGOLIN_BUILD_PATH=${PANGOLIN_PATH}/build

if [ ! -d ${PANGOLIN_PATH} ] ; then # Pangolin has not been downloaded does not exist
	echo "Cloning Pangolin"
	cd ${TMP_PATH}
	git clone https://github.com/stevenlovegrove/Pangolin.git
	cd ${PANGOLIN_PATH}
fi

if [ ! -d ${PANGOLIN_BUILD_PATH} ] ; then # build does not exist
	echo "Creating build directory"
	mkdir -p ${PANGOLIN_BUILD_PATH}
fi

# Create makefiles and build
echo "Building Pangolin library"
cd ${PANGOLIN_BUILD_PATH}
cmake -DBUILD_TESTS=OFF \
      -DBUILD_EXAMPLES=OFF \
      -DBUILD_PANGOLIN_LIBREALSENSE=OFF \
      -DBUILD_PANGOLIN_LIBREALSENSE2=OFF \
      -DBUILD_PANGOLIN_OPENNI=OFF \
      -DBUILD_PANGOLIN_OPENNI2=OFF \
      -DBUILD_PANGOLIN_TELICAM=OFF \
      ..
cmake --build .

BUILD_PATH=${PANGOLIN_BUILD_PATH}/src/libpangolin.so

if [ ! -f ${BUILD_PATH} ] ; then # build does not exist
	echo "Unsucessfull Pangolin build"
	exit -1
fi

# Install library and headers
echo "Installing Pangolin library"
sudo make install
sudo ldconfig

# Remove temp files
echo "Cleaning up"
rm -rf ${PANGOLIN_PATH}
