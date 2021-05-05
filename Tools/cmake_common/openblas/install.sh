#!/bin/bash

set -e  # exit on error

TMP_PATH=/tmp
OPENBLAS_PATH=${TMP_PATH}/OpenBLAS

if [ ! -d ${OPENBLAS_PATH} ] ; then # OpenBLAS has not been downloaded does not exist
	echo "Cloning OpenBLAS"
	cd ${TMP_PATH}
	git clone https://github.com/xianyi/OpenBLAS
	cd ${OPENBLAS_PATH}
	git checkout tags/v0.2.20
fi

# Create makefiles and build
echo "Building OpenBLAS library"
cd ${OPENBLAS_PATH}
make -j8 -o3

BUILD_PATH=${OPENBLAS_PATH}/libopenblas_*.so

if [ ! -f ${BUILD_PATH} ] ; then # build does not exist
	echo "Unsucessfull OpenBLAS build"
	exit -1
fi

# Install library and headers
echo "Installing OpenBLAS library"
sudo make install
sudo ldconfig

# Remove temp files
echo "Cleaning up"
rm -rf ${OPENBLAS_PATH}
