# Add custom OpenCV build paths
#set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH}  /path/to/opencv-2.4.13.6/install/share/OpenCV)

# check the existance of OpenCV, if not/outdated, install/reinstall
find_package(OpenCV 2.4.13 REQUIRED core imgproc highgui contrib)

if (${OpenCV_FOUND})
	message(STATUS "OpenCV found")
	message(STATUS "OpenCV include directory:  ${OpenCV_INCLUDE_DIRS}")
else()
	find_package(OpenCV 2.4.3 QUIET)
	if(NOT OpenCV_FOUND)
		message(FATAL_ERROR "OpenCV >= 2.4.3 not found.")
	endif()
endif()

# Configure not to use CUDA
set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCUDA_USE_STATIC_CUDA_RUNTIME=OFF")

mark_as_advanced(OpenCV_DIR)
