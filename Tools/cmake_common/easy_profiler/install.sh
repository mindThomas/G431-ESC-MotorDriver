#!/bin/bash

set -e  # exit on error

TMP_PATH=/tmp
EASY_PROFILER_PATH=${TMP_PATH}/easy_profiler
EASY_PROFILER_BUILD_PATH=${EASY_PROFILER_PATH}/build

if [ ! -d ${EASY_PROFILER_PATH} ] ; then # easy_profiler has not been downloaded does not exist
	echo "Cloning easy_profiler"
	cd ${TMP_PATH}
	git clone https://github.com/yse/easy_profiler
	cd ${EASY_PROFILER_PATH}
	git checkout tags/v1.3.0
fi

if [ ! -d ${EASY_PROFILER_BUILD_PATH} ] ; then # build does not exist
	echo "Creating build directory"
	mkdir -p ${EASY_PROFILER_BUILD_PATH}
fi

# Create makefiles and build
echo "Building easy_profiler library"
cd ${EASY_PROFILER_BUILD_PATH}
cmake -DCMAKE_BUILD_TYPE="Release" ..
make

BUILD_PATH=${EASY_PROFILER_BUILD_PATH}/bin/libeasy_profiler.so

if [ ! -f ${BUILD_PATH} ] ; then # build does not exist
	echo "Unsucessfull easy_profiler build"
	exit -1
fi

# Install library and headers
echo "Installing easy_profiler library"
sudo make install
sudo ldconfig

# Remove temp files
echo "Cleaning up"
rm -rf ${EASY_PROFILER_PATH}
