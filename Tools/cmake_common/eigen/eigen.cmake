# check the existance of OpenCV, if not/outdated, install/reinstall
find_package(Eigen3) # REQUIRED
if (${EIGEN3_FOUND})
	message(STATUS "Eigen3 found")
else()
	message(STATUS "Eigen3 not found! Executing install script.")
	execute_process(COMMAND ${CMAKE_CURRENT_LIST_DIR}/install.sh
	    		RESULT_VARIABLE retcode)
	if(NOT ${retcode} EQUAL 0)
	    message(FATAL_ERROR "eigen.cmake: Error when excuting ${CMAKE_CURRENT_LIST_DIR}/install.sh")
	endif()
	find_package(Eigen3 REQUIRED) # try and find again
endif()
include_directories(${EIGEN3_INCLUDE_DIR})
