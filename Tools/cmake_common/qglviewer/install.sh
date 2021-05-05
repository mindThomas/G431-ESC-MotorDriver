#!/bin/bash

set -e  # exit on error

# Install dependencies


TMP_PATH=/tmp
QGLVIEWER_PATH=${TMP_PATH}/libQGLViewer-2.7.2

if [ ! -d ${QGLVIEWER_PATH} ] ; then # QGLViewer has not been downloaded does not exist
	echo "Downloading QGLViewer library source code"
	cd ${TMP_PATH}
	wget http://www.libqglviewer.com/src/libQGLViewer-2.7.2.tar.gz
        tar -xzf libQGLViewer-2.7.2.tar.gz
fi

# Create makefiles and build
echo "Building QGLViewer library"
cd ${QGLVIEWER_PATH}/QGLViewer
qmake
make -j


BUILD_PATH=${QGLVIEWER_PATH}/QGLViewer/libQGLViewer-qt5.so

if [ ! -f ${BUILD_PATH} ] ; then # build does not exist
	echo "Unsucessfull QGLViewer build"
	exit -1
fi

# Install library and headers
echo "Installing QGLViewer library"
sudo make install
sudo ldconfig

# Remove temp files
echo "Cleaning up"
rm -rf ${QGLVIEWER_PATH}
