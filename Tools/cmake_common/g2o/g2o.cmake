# check the existance of G2O, if not/outdated, install/reinstall
#find_package(G2O) # REQUIRED
include(${CMAKE_CURRENT_LIST_DIR}/FindG2O.cmake)
if (${G2O_FOUND})
	message(STATUS "G2O found")
else()
	message(STATUS "G2O not found! Executing install script.")
	execute_process(COMMAND ${CMAKE_CURRENT_LIST_DIR}/install.sh
	    		RESULT_VARIABLE retcode)
	if(NOT ${retcode} EQUAL 0)
	    message(FATAL_ERROR "g2o.cmake: Error when excuting ${CMAKE_CURRENT_LIST_DIR}/install.sh")
	endif()
	include(${CMAKE_CURRENT_LIST_DIR}/FindG2O.cmake) # try and find again
	if (NOT ${G2O_FOUND})
		message(FATAL_ERROR "G2O not found!")
	endif()
endif()
include_directories(${G2O_INCLUDE_DIR})
