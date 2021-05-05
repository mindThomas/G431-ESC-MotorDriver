# check the existance of apriltags, if not/outdated, install/reinstall
find_package(Brisk)
if (${BRISK_FOUND})
	message(STATUS "BRISK found")
else()
	message(STATUS "BRISK not found! Executing install script.")
	execute_process(COMMAND ${CMAKE_CURRENT_LIST_DIR}/install.sh
	    		RESULT_VARIABLE retcode)
	if(NOT ${retcode} EQUAL 0)
	    message(FATAL_ERROR "brisk.cmake: Error when excuting ${CMAKE_CURRENT_LIST_DIR}/install.sh")
	endif()
	find_package(Brisk REQUIRED) # try and find again
endif()
include_directories(${BRISK_INCLUDE_DIR})
