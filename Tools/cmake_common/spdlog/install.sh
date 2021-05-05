#!/bin/bash

set -e  # exit on error

# Install dependencies

TMP_PATH=/tmp
SPDLOG_PATH=${TMP_PATH}/spdlog
SPDLOG_BUILD_PATH=${SPDLOG_PATH}/build

if [ ! -d ${SPDLOG_PATH} ] ; then # Pangolin has not been downloaded does not exist
	echo "Cloning SPDLOG"
	cd ${TMP_PATH}	
    git clone https://github.com/gabime/spdlog.git
	cd ${SPDLOG_PATH}
fi

if [ ! -d ${SPDLOG_BUILD_PATH} ] ; then # build does not exist
	echo "Creating build directory"
	mkdir -p ${SPDLOG_BUILD_PATH}
fi

# Create makefiles and build
echo "Building SPDLOG library"
cd ${SPDLOG_BUILD_PATH}
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8

BUILD_PATH=${SPDLOG_BUILD_PATH}/libspdlog.a

if [ ! -f ${BUILD_PATH} ] ; then # build does not exist
	echo "Unsucessfull SPDLOG build"
	exit -1
fi

# Install library and headers
echo "Installing SPDLOG library"
sudo make install
sudo ldconfig

# Remove temp files
echo "Cleaning up"
rm -rf ${SPDLOG_PATH}
