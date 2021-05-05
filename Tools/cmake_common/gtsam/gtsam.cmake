#LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})

# check the existance of GTSAM, if not/outdated, install/reinstall
find_package(GTSAM) # REQUIRED
if (${GTSAM_FOUND})
	message(STATUS "GTSAM found")
else()
	message(STATUS "GTSAM not found! Executing install script.")
	execute_process(COMMAND ${CMAKE_CURRENT_LIST_DIR}/install.sh
	    		RESULT_VARIABLE retcode)
	if(NOT ${retcode} EQUAL 0)
	    message(FATAL_ERROR "gtsam.cmake: Error when excuting ${CMAKE_CURRENT_LIST_DIR}/install.sh")
	endif()
	find_package(GTSAM REQUIRED) # try and find again
endif()
include_directories(${GTSAM_INCLUDE_DIR})

#set(EIGEN3_INCLUDE_DIR "")
#find_package(Eigen3 REQUIRED)
#set(EIGEN_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR}) 


# Force GTSAM-bundled version of Eigen to be used
#mark_as_advanced(
#    EIGEN_INCLUDE_DIRS
#    EIGEN3_INCLUDE_DIRS
#)
#set(EIGEN3_FOUND TRUE)
#set(EIGEN_INCLUDE_DIRS ${GTSAM_INCLUDE_DIR}/gtsam/3rdparty/Eigen)

#set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{NUCORE}/build/gtsam/lib/cmake/GTSAM)
#find_package(GTSAM REQUIRED)
#include_directories(SYSTEM "$ENV{NUCORE}/build/gtsam/include")
#link_directories($ENV{NUCORE}/build/gtsam/lib)

#mark_as_advanced(
#    EIGEN_INCLUDE_DIRS
#    GTSAM_DIR
#    GTSAM_INCLUDE_DIR
#)
