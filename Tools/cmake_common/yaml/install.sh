#!/bin/bash

set -e  # exit on error

TMP_PATH=/tmp
YAML_PATH=${TMP_PATH}/yaml-cpp
YAML_BUILD_PATH=${YAML_PATH}/build

if [ ! -d ${YAML_PATH} ] ; then # Pangolin has not been downloaded does not exist
	echo "Cloning yaml-cpp"
	cd ${TMP_PATH}
	git clone https://github.com/jbeder/yaml-cpp
	cd ${YAML_PATH}
fi

if [ ! -d ${YAML_BUILD_PATH} ] ; then # build does not exist
	echo "Creating build directory"
	mkdir -p ${YAML_BUILD_PATH}
fi

# Create makefiles and build
echo "Building YAML library"
cd ${YAML_BUILD_PATH}
cmake ..
make

BUILD_PATH=${YAML_BUILD_PATH}/libyaml.so

if [ ! -f ${BUILD_PATH} ] ; then # build does not exist
	echo "Unsucessfull YAML build"
	exit -1
fi

# Install library and headers
echo "Installing YAML library"
sudo make install
sudo ldconfig

# Remove temp files
echo "Cleaning up"
rm -rf ${YAML_PATH}
