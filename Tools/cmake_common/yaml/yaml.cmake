# check the existance of yaml-cpp, if not/outdated, install/reinstall
find_package(yaml-cpp 0.5)
if (IS_DIRECTORY ${YAML_CPP_INCLUDE_DIR})
	message(STATUS "YAML found")
else()
	message(STATUS "YAML not found! Executing install script.")
	execute_process(COMMAND ${CMAKE_CURRENT_LIST_DIR}/install.sh
	    		RESULT_VARIABLE retcode)
	if(NOT ${retcode} EQUAL 0)
	    message(FATAL_ERROR "yaml.cmake: Error when excuting ${CMAKE_CURRENT_LIST_DIR}/install.sh")
	endif()
	find_package(yaml-cpp 0.5 REQUIRED) # try and find again
endif()
include_directories(${YAML_CPP_INCLUDE_DIR})
