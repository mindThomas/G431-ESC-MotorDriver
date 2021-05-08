# - Try to find QGLViewer
# Once done this will define
#
#  QGLVIEWER_FOUND - system has QGLViewer
#  QGLVIEWER_INCLUDE_DIR - the QGLViewer include directory
#  QGLVIEWER_LIBRARIES - Link these to use QGLViewer
#  QGLVIEWER_DEFINITIONS - Compiler switches required for using QGLViewer
#

find_path(QGLVIEWER_INCLUDE_DIR
          NAMES QGLViewer/qglviewer.h
          PATHS /Library/Frameworks
                /usr/include
                /usr/local/include
                ENV
                QGLVIEWERROOT
          PATH_SUFFIXES Headers)

find_library(QGLVIEWER_LIBRARY_RELEASE
             NAMES QGLViewer QGLViewer2
             PATHS /usr/lib
                   /usr/local/lib
                   /Library/Frameworks
                   ENV
                   QGLVIEWERROOT
                   ENV
                   LD_LIBRARY_PATH
                   ENV
                   LIBRARY_PATH
             PATH_SUFFIXES QGLViewer QGLViewer/release)

find_library(QGLVIEWER_LIBRARY_DEBUG
             NAMES dqglviewer
                   dQGLViewer
                   dQGLViewer2
                   QGLViewerd2
             PATHS /usr/lib
                   /usr/local/lib
                   /Library/Frameworks
                   ENV
                   QGLVIEWERROOT
                   ENV
                   LD_LIBRARY_PATH
                   ENV
                   LIBRARY_PATH
             PATH_SUFFIXES QGLViewer QGLViewer/debug)

if(QGLVIEWER_LIBRARY_RELEASE)
    if(QGLVIEWER_LIBRARY_DEBUG)
        set(QGLVIEWER_LIBRARIES_ optimized ${QGLVIEWER_LIBRARY_RELEASE} debug ${QGLVIEWER_LIBRARY_DEBUG})
    else()
        set(QGLVIEWER_LIBRARIES_ ${QGLVIEWER_LIBRARY_RELEASE})
    endif()

    set(QGLVIEWER_LIBRARIES ${QGLVIEWER_LIBRARIES_} CACHE FILEPATH "The QGLViewer library")

endif()

if(QGLVIEWER_INCLUDE_DIR AND QGLVIEWER_LIBRARIES)
    set(QGLVIEWER_FOUND TRUE)
endif()

if(QGLVIEWER_FOUND)
    if(NOT CMAKE_FIND_QUIETLY)
        message(STATUS "Found QGLViewer: ${QGLVIEWER_LIBRARIES}")
    endif()
else()
    if(QGLViewer_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find QGLViewer")
    endif()
endif()

#--- Remove QGLViewer warnings (@todo can we do this inline?)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-gnu-anonymous-struct -Wno-nested-anon-types")
