# Add custom OpenCV build paths
#set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH}  /path/to/opencv-3.0.0/install/share/OpenCV)
set(OPENCV_SEARCH_PATH ~/repos/opencv-3.4.3/install/share/OpenCV)
list(APPEND CMAKE_MODULE_PATH "${OPENCV_SEARCH_PATH}")
list(APPEND CMAKE_PREFIX_PATH "${OPENCV_SEARCH_PATH}")
message(STATUS "OpenCV search path: ${OPENCV_SEARCH_PATH}")

# check the existance of OpenCV, if not/outdated, install/reinstall
find_package(OpenCV 3.0 REQUIRED)

if (${OpenCV_FOUND})
	message(STATUS "OpenCV found")
	message(STATUS "OpenCV include directory:  ${OpenCV_INCLUDE_DIRS}")
else()
	find_package(OpenCV 3.0 QUIET)
	if(NOT OpenCV_FOUND)
		message(FATAL_ERROR "OpenCV >= 3.0 not found.")
	endif()
endif()

# Configure not to use CUDA
#set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCUDA_USE_STATIC_CUDA_RUNTIME=OFF")

mark_as_advanced(OpenCV_DIR)
