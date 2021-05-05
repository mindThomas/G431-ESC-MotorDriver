# CMakeLists.txt -- Build system for the pybind11 examples
#
# Copyright (c) 2015 Wenzel Jakob <wenzel@inf.ethz.ch>
#
# All rights reserved. Use of this source code is governed by a
# BSD-style license that can be found in the LICENSE file.

include(CheckCXXCompilerFlag)

check_cxx_compiler_flag("-std=c++14" HAS_CPP14_FLAG)
check_cxx_compiler_flag("-std=c++11" HAS_CPP11_FLAG)

if (HAS_CPP14_FLAG)
  set(PYBIND11_CPP_STANDARD -std=c++14)
elseif (HAS_CPP11_FLAG)
  set(PYBIND11_CPP_STANDARD -std=c++11)
else()
  message(FATAL_ERROR "Unsupported compiler -- pybind11 requires C++11 support!")
endif()

# Determine pybind11 header location (installed via pip)
set (python_cmd "python3")
set (python_arg "-c" "from\ pybind11\ import\ get_include\;\ print\(get_include\(\)\)")
execute_process(COMMAND ${python_cmd} ${python_arg} OUTPUT_VARIABLE PYBIND11_INCLUDE_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
message(STATUS "Pybind11 include dir: ${PYBIND11_INCLUDE_DIR}")
message(STATUS "Python include dir: ${PYTHON_INCLUDE_DIRS}")

# Build a Python extension module:
# pybind11_add_module(<name> source1 [source2 ...])
#
function(pybind11_add_module target_name)
  add_library(${target_name} MODULE ${ARGN})
  target_include_directories(${target_name} PUBLIC ${PYBIND11_INCLUDE_DIR} ${PYTHON_INCLUDE_DIRS})

  # Disable `lib' prefix
  set_target_properties(${target_name} PROPERTIES PREFIX "")

  # Make sure C++11/14 are enabled
  target_compile_options(${target_name} PUBLIC ${PYBIND11_CPP_STANDARD})

  # Enable link time optimization and set the default symbol
  # visibility to hidden (very important to obtain small binaries)
  string(TOUPPER "${CMAKE_BUILD_TYPE}" U_CMAKE_BUILD_TYPE)
  if (NOT ${U_CMAKE_BUILD_TYPE} MATCHES DEBUG)
    # Check for Link Time Optimization support (GCC/Clang)
    check_cxx_compiler_flag("-flto" HAS_LTO_FLAG)
    if(HAS_LTO_FLAG AND NOT CYGWIN)
      target_compile_options(${target_name} PRIVATE -flto)
    endif()

    # Intel equivalent to LTO is called IPO
    if(CMAKE_CXX_COMPILER_ID MATCHES "Intel")
      check_cxx_compiler_flag("-ipo" HAS_IPO_FLAG)
      if(HAS_IPO_FLAG)
        target_compile_options(${target_name} PRIVATE -ipo)
      endif()
    endif()

    # Default symbol visibility
    target_compile_options(${target_name} PRIVATE "-fvisibility=hidden")

    # Strip unnecessary sections of the binary on Linux/Mac OS
    if(CMAKE_STRIP)
      add_custom_command(TARGET ${target_name} POST_BUILD
        COMMAND ${CMAKE_STRIP} $<TARGET_FILE:${target_name}>)
    endif()
  endif()
endfunction()
