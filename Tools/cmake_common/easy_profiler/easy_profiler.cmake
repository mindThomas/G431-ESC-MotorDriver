# check the existance of easy_profiler, if not/outdated, install/reinstall
find_package(easy_profiler)
if (${easy_profiler_FOUND})
	message(STATUS "easy_profiler found")
else()
	message(STATUS "easy_profiler not found! Executing install script.")
	execute_process(COMMAND ${CMAKE_CURRENT_LIST_DIR}/install.sh
	    		RESULT_VARIABLE retcode)
	if(NOT ${retcode} EQUAL 0)
	    message(FATAL_ERROR "easy_profiler.cmake: Error when excuting ${CMAKE_CURRENT_LIST_DIR}/install.sh")
	endif()
	find_package(easy_profiler REQUIRED) # try and find again
endif()
include_directories(${easy_profiler_INCLUDE_DIRS})
