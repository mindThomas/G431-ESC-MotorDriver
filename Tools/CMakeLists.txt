# Copyright (C) 2021- Thomas Jespersen, TKJ Electronics. All rights reserved.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the MIT License
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the MIT License for further details.
#
# Contact information
# ------------------------------------------
# Thomas Jespersen, TKJ Electronics
# Web      :  http://www.tkjelectronics.dk
# e-mail   :  thomasj@tkjelectronics.dk
# ------------------------------------------

project(G431-ESC-MotorDriver)

cmake_minimum_required(VERSION 3.10)

if(POLICY CMP0020)
    cmake_policy(SET CMP0020 NEW)
endif()

message("##################################\n" "# Thomas' Automatic CMake loader #\n"
        "##################################")

# Add support for C++11
#add_definitions(-std=c++11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_CXX_EXTENSIONS NO) # use -std=c++14 instead of -std=gnu++14

# Add automatic Qt UI file conversion
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# Compile for native architecture to optimize performance
#include(CheckCXXCompilerFlag)
#CHECK_CXX_COMPILER_FLAG("-march=native" COMPILER_SUPPORTS_MARCH_NATIVE)
#if(COMPILER_SUPPORTS_MARCH_NATIVE)
#message(STATUS "Enabling native architecture")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
#endif()
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=x86-64 -mavx")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Winit-self -D_FORTIFY_SOURCE=2 -fstack-protector-strong")
# "-Werror=format-security -Werror=implicit-function-declaration"
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -DNDEBUG") # Disable assert() in release builds
set(COMMON_LINK_FLAGS "") # "-Wl,--copy-dt-needed-entries -Wl,-E -Wl,-z,relro,-z,now -Wl,--as-needed -Wl,-z,noexecstack"

# Use -pthread or the compiler's equivalent
find_package(Threads REQUIRED)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

# Make sure -fPIC is enabled
set(CMAKE_POSITION_INDEPENDENT_CODE True)
# -pie is needed here because of https://gitlab.kitware.com/cmake/cmake/issues/14983
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${COMMON_LINK_FLAGS} -pie")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${COMMON_LINK_FLAGS}")
set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} ${COMMON_LINK_FLAGS}")

# For using pkg-config to find library flags
find_package(PkgConfig)

# build type
if(CMAKE_BUILD_TYPE STREQUAL "")
    unset(CMAKE_BUILD_TYPE CACHE) # no ambiguity
endif()
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "RelWithDebInfo, Release or Debug")

message(STATUS "Default Compile flags [current build: ${CMAKE_BUILD_TYPE}]")
message(STATUS "  CMAKE_C_FLAGS_DEBUG: ${CMAKE_C_FLAGS_DEBUG}")
message(STATUS "  CMAKE_C_FLAGS_RELEASE: ${CMAKE_C_FLAGS_RELEASE}")
message(STATUS "  CMAKE_C_FLAGS_RELWITHDEBINFO: ${CMAKE_C_FLAGS_RELWITHDEBINFO}")
message(STATUS "  CMAKE_C_FLAGS_MINSIZEREL: ${CMAKE_C_FLAGS_MINSIZEREL}")

message(STATUS "  CMAKE_CXX_FLAGS_DEBUG: ${CMAKE_CXX_FLAGS_DEBUG}")
message(STATUS "  CMAKE_CXX_FLAGS_RELEASE: ${CMAKE_CXX_FLAGS_RELEASE}")
message(STATUS "  CMAKE_CXX_FLAGS_RELWITHDEBINFO: ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
message(STATUS "  CMAKE_CXX_FLAGS_MINSIZEREL: ${CMAKE_CXX_FLAGS_MINSIZEREL}")
message(STATUS "\n--")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_common)
include(cmake_common/gtsam/gtsam.cmake) # also includes Eigen
include(cmake_common/cuda.cmake)
include(cmake_common/qt5.cmake) # MUST COME BEFORE pcl.cmake
include(cmake_common/eigen/eigen.cmake)
include(cmake_common/opencv/opencv4.cmake)
include(cmake_common/pangolin/pangolin.cmake)
include(cmake_common/qglviewer/qglviewer.cmake)
include(cmake_common/openmp.cmake) # for code optimization
include(cmake_common/yaml/yaml.cmake)
include(cmake_common/easy_profiler/easy_profiler.cmake) # profiling tool

option(SPDLOG_BUILD_SHARED "" OFF)
add_subdirectory(libs/spdlog)

message(STATUS "   Eigen include directory: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "   OpenCV path: ${OpenCV_INCLUDE_DIRS}")

find_package(Boost REQUIRED
             COMPONENTS system
                        signals
                        python
                        thread
                        program_options
                        regex
                        timer
                        filesystem
             QUIET)
mark_as_advanced(BOOST_THREAD_LIBRARY)

find_package(CURL REQUIRED)
find_package(gflags REQUIRED) # sudo apt-get install libgflags-dev
find_package(SuiteParse REQUIRED)
find_package(Eigen3 3.3.0 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(Pangolin 0.2 REQUIRED)

include_directories(... ${OpenCV_INCLUDE_DIRS})
include_directories(... ${Boost_INCLUDE_DIRS})

link_directories(... ${OpenCV_LIBRARY_DIRS})
link_directories(... ${Boost_LIBRARY_DIRS})

# Print all include directories
#get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
#foreach(dir ${dirs})
#message(STATUS "dir='${dir}'")
#endforeach()

# Make a compiler flag for the current source folder path
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__SOURCE_FOLDER__='\"$(dir $(abspath $<))\"'")

# Place the executable in the root of build folder
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/libs") # shared libs
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/libs") # static libs

# Define Global Headers placed in include folder
set(GLOBAL_HEADERS "")
include_directories(include)
add_subdirectory(include)

include_directories(../Common)
add_subdirectory(../Common "Common")

macro(update_folder_name)
    get_filename_component(folder_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
    string(REPLACE " "
                   "_"
                   folder_name
                   ${folder_name})
endmacro()

# Include libraries
macro(CMAKE_LIBS_DIRECTORIES return_list)
    set(EXCLUDE_DIRS
        libs/spdlog
        ${PROJECT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src)
    file(GLOB_RECURSE new_list CMakeLists.txt)
    set(dir_list "")
    foreach(file_path ${new_list})
        get_filename_component(dir_path ${file_path} PATH)
        foreach(EXCLUDE_DIR ${EXCLUDE_DIRS})
            string(FIND ${dir_path} ${EXCLUDE_DIR} EXCLUDE_DIR_FOUND)
            if(NOT ${EXCLUDE_DIR_FOUND} EQUAL -1)
                break()
            endif()
        endforeach()
        if(${EXCLUDE_DIR_FOUND} EQUAL -1)
            set(dir_list ${dir_list} ${dir_path})
        endif()
    endforeach()
    list(REMOVE_DUPLICATES dir_list)
    list(REMOVE_ITEM dir_list ${CMAKE_CURRENT_SOURCE_DIR})
    set(${return_list} ${dir_list})
endmacro()

message("#################### FINISHED FINDING/LOADING PACKAGES ####################\n")
message(STATUS "Project directory: ${PROJECT_SOURCE_DIR}")
message(STATUS "Root directory: " ${CMAKE_CURRENT_SOURCE_DIR})
message(STATUS "")

include_directories(libs)
cmake_libs_directories(SUBDIRS)
message(STATUS "Adding libraries:")
foreach(subdir ${SUBDIRS})
    #message(STATUS "   " ${subdir})
    add_subdirectory(${subdir})
endforeach()
message(STATUS "")

# Include the binary projects
macro(HEADER_DIRECTORIES return_list)
    file(GLOB_RECURSE new_list *.h)
    set(dir_list "")
    foreach(file_path ${new_list})
        get_filename_component(dir_path ${file_path} PATH)
        set(dir_list ${dir_list} ${dir_path})
    endforeach()
    list(REMOVE_DUPLICATES dir_list)
    set(${return_list} ${dir_list})
endmacro()

macro(CMAKE_SRC_DIRECTORIES return_list)
    set(EXCLUDE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
    set(EXCLUDE_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs)
    file(GLOB_RECURSE new_list CMakeLists.txt)
    set(dir_list "")
    foreach(file_path ${new_list})
        get_filename_component(dir_path ${file_path} PATH)
        string(FIND ${dir_path} ${EXCLUDE_INCLUDE_DIR} EXCLUDE_INCLUDE_DIR_FOUND)
        string(FIND ${dir_path} ${EXCLUDE_LIBS_DIR} EXCLUDE_LIBS_DIR_FOUND)
        if(${EXCLUDE_INCLUDE_DIR_FOUND} EQUAL -1 AND ${EXCLUDE_LIBS_DIR_FOUND} EQUAL -1)
            set(dir_list ${dir_list} ${dir_path})
        endif()
    endforeach()
    list(REMOVE_DUPLICATES dir_list)
    list(REMOVE_ITEM dir_list ${CMAKE_CURRENT_SOURCE_DIR})
    set(${return_list} ${dir_list})
endmacro()

macro(SUBDIRLIST result curdir)
    file(GLOB children RELATIVE ${curdir} ${curdir}/*)
    set(dirlist "")
    foreach(child ${children})
        if(IS_DIRECTORY ${curdir}/${child})
            list(APPEND dirlist ${child})
        endif()
    endforeach()
    set(${result} ${dirlist})
endmacro()

macro(prepare_binary)
    #message(STATUS "   ${CMAKE_CURRENT_SOURCE_DIR}\t\t[${CMAKE_BUILD_TYPE}]")
    update_folder_name() # Name the binary in this folder as the folder name
    set(binary_name "${folder_name}")
    message(STATUS "   ${binary_name}\t\t[${CMAKE_BUILD_TYPE}]")
    file(GLOB_RECURSE binary_sources_c *.c) # find all cpp files
    file(GLOB_RECURSE binary_sources_cpp *.cpp) # find all cpp files
    file(GLOB_RECURSE binary_headers *.hpp) # find all hpp files
                                            #message(STATUS "        Binary sources: " ${binary_sources})
                                            #message(STATUS "        Binary headers: " ${binary_headers})
                                            #message(STATUS "        Global headers: " ${GLOBAL_HEADERS})
    set_property(SOURCE ${binary_sources_c} PROPERTY SKIP_AUTOGEN ON)
    set_property(SOURCE ${binary_sources_cpp} PROPERTY SKIP_AUTOGEN ON)
    set_property(SOURCE ${binary_headers} PROPERTY SKIP_AUTOGEN ON)
    set_property(SOURCE ${GLOBAL_HEADERS} PROPERTY SKIP_AUTOGEN ON)

    add_executable(${binary_name}
                   ${binary_sources_c}
                   ${binary_sources_cpp}
                   ${binary_headers}
                   ${GLOBAL_HEADERS})
    target_link_libraries(${binary_name} ${binary_dependencies})

    set_target_properties(${binary_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
    set_target_properties(${binary_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
    set_target_properties(${binary_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

    set_target_properties(${binary_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
    set_target_properties(${binary_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
    set_target_properties(${binary_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
endmacro()

macro(prepare_qt_binary)
    #message(STATUS "   ${CMAKE_CURRENT_SOURCE_DIR}\t\t[${CMAKE_BUILD_TYPE}]")
    update_folder_name() # Name the binary in this folder as the folder name
    set(binary_name "${folder_name}")
    message(STATUS "   ${binary_name}\t\t[${CMAKE_BUILD_TYPE}]")
    file(GLOB_RECURSE binary_sources_c *.c) # find all cpp files
    file(GLOB_RECURSE binary_sources_cpp *.cpp) # find all cpp files
    file(GLOB_RECURSE binary_sources_ui *.ui) # find all ui files
    file(GLOB_RECURSE binary_sources_qrc *.qrc) # find all Qt Ressource files
    file(GLOB_RECURSE binary_headers *.hpp) # find all hpp files

    #file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.ui)
    #file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.qrc)
    #QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
    #QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
    #add_executable(qt_rosbag ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})

    #message(STATUS "        Binary sources: " ${binary_sources})
    #message(STATUS "        Binary headers: " ${binary_headers})
    #message(STATUS "        Global headers: " ${GLOBAL_HEADERS})
    add_executable(${binary_name}
                   ${binary_sources_c}
                   ${binary_sources_cpp}
                   ${binary_sources_ui}
                   ${binary_sources_qrc}
                   ${GLOBAL_HEADERS})
    target_link_libraries(${binary_name} ${binary_dependencies})

    set_target_properties(${binary_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
    set_target_properties(${binary_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
    set_target_properties(${binary_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

    set_target_properties(${binary_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
    set_target_properties(${binary_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
    set_target_properties(${binary_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
endmacro()

cmake_src_directories(SUBDIRS)

message(STATUS "Adding binaries:")
foreach(subdir ${SUBDIRS})
    #message(STATUS "   " ${subdir})
    add_subdirectory(${subdir})
    #include_directories(${subdir})
endforeach()
message(STATUS "")
