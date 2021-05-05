
# This is FindBRISK.cmake
# CMake module to locate the BRISK package
#
# The following cache variables may be set before calling this script:
#
# BRISK_DIR (or BRISK_ROOT): (Optional) The install prefix OR source tree of brisk (e.g. /usr/local or src/brisk)
# BRISK_BUILD_NAME:          (Optional) If compiling against a source tree, the name of the build directory
#                            within it (e.g build-debug).  Without this defined, this script tries to
#                            intelligently find the build directory based on the project's build directory name
#                            or based on the build type (Debug/Release/etc).
#
# The following variables will be defined:
#
# BRISK_FOUND          : TRUE if the package has been successfully found
# BRISK_INCLUDE_DIR    : paths to BRISK's INCLUDE directories
# BRISK_LIBS           : paths to BRISK's libraries
#
# NOTES on compiling against an uninstalled BRISK build tree:
# - A BRISK source tree will be automatically searched for in the directory
#   'brisk' next to your project directory, after searching
#   CMAKE_INSTALL_PREFIX and $HOME, but before searching /usr/local and /usr.
# - The build directory will be searched first with the same name as your
#   project's build directory, e.g. if you build from 'MyProject/build-optimized',
#   'brisk/build-optimized' will be searched first.  Next, a build directory for
#   your project's build type, e.g. if CMAKE_BUILD_TYPE in your project is
#   'Release', then 'brisk/build-release' will be searched next.  Finally, plain
#   'brisk/build' will be searched.
# - You can control the brisk build directory name directly by defining the CMake
#   cache variable 'BRISK_BUILD_NAME', then only 'brisk/${BRISK_BUILD_NAME} will
#   be searched.
# - Use the standard CMAKE_PREFIX_PATH, or BRISK_DIR, to find a specific brisk
#   directory.

# Get path suffixes to help look for brisk
if(BRISK_BUILD_NAME)
  set(brisk_build_names "${BRISK_BUILD_NAME}/brisk")
else()
  # lowercase build type
  string(TOLOWER "${CMAKE_BUILD_TYPE}" build_type_suffix)
  # build suffix of this project
  get_filename_component(my_build_name "${CMAKE_BINARY_DIR}" NAME)
  
  set(brisk_build_names "${my_build_name}/brisk" "build-${build_type_suffix}/brisk" "build/brisk")
endif()

# Use BRISK_ROOT or BRISK_DIR equivalently
if(BRISK_ROOT AND NOT BRISK_DIR)
  set(BRISK_DIR "${BRISK_ROOT}")
endif()

if(BRISK_DIR)
  # Find include dirs
  find_path(BRISK_INCLUDE_DIR brisk/brisk.h
    PATHS "${BRISK_DIR}/include" "${BRISK_DIR}" NO_DEFAULT_PATH
    DOC "BRISK include directories")

  # Find libraries
  find_library(BRISK_LIBS NAMES brisk
    HINTS "${BRISK_DIR}/lib" "${BRISK_DIR}" NO_DEFAULT_PATH
    PATH_SUFFIXES ${brisk_build_names}
    DOC "BRISK libraries")
else()
  # Find include dirs
  set(extra_include_paths ${CMAKE_INSTALL_PREFIX}/include "$ENV{HOME}/include" "${PROJECT_SOURCE_DIR}/../brisk" /usr/local/include /usr/include)
  find_path(BRISK_INCLUDE_DIR brisk/brisk.h
    PATHS ${extra_include_paths}
    DOC "BRISK include directories")
  if(NOT BRISK_INCLUDE_DIR)
    message(STATUS "Searched for brisk headers in default paths plus ${extra_include_paths}")
  endif()

  # Find libraries
  find_library(BRISK_LIBS NAMES brisk
    HINTS ${CMAKE_INSTALL_PREFIX}/lib "$ENV{HOME}/lib" "${PROJECT_SOURCE_DIR}/../brisk" /usr/local/lib /usr/lib
    PATH_SUFFIXES ${brisk_build_names}
    DOC "BRISK libraries")
endif()

# handle the QUIETLY and REQUIRED arguments and set BRISK_FOUND to TRUE
# if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(BRISK DEFAULT_MSG
                                  BRISK_LIBS BRISK_INCLUDE_DIR)
 
