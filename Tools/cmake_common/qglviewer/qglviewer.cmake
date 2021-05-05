# check the existance of QGLViewer, if not/outdated, install/reinstall
find_package(QGLViewer)
if (${QGLVIEWER_FOUND})
	message(STATUS "QGLViewer found")
    message("QGLVIEWER_INCLUDE_DIRS: ${QGLVIEWER_INCLUDE_DIR}")
else()
	message(STATUS "QGLViewer not found! Executing install script.")
	execute_process(COMMAND ${CMAKE_CURRENT_LIST_DIR}/install.sh
	    		RESULT_VARIABLE retcode)
	if(NOT ${retcode} EQUAL 0)
	    message(FATAL_ERROR "QGLViewer.cmake: Error when excuting ${CMAKE_CURRENT_LIST_DIR}/install.sh")
	endif()
	find_package(QGLViewer REQUIRED) # try and find again
endif()
include_directories(${QGLVIEWER_INCLUDE_DIR})
