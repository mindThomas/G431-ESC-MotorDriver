# check the existance of Pangolin, if not/outdated, install/reinstall
find_package(Pangolin 0.4)
if (${Pangolin_FOUND})
	message(STATUS "Pangolin found")
else()
	message(STATUS "Pangolin not found! Executing install script.")
	execute_process(COMMAND ${CMAKE_CURRENT_LIST_DIR}/install.sh
	    		RESULT_VARIABLE retcode)
	if(NOT ${retcode} EQUAL 0)
	    message(FATAL_ERROR "pangolin.cmake: Error when excuting ${CMAKE_CURRENT_LIST_DIR}/install.sh")
	endif()
	find_package(Pangolin 0.4 REQUIRED) # try and find again
endif()
include_directories(${Pangolin_INCLUDE_DIRS})
