if(NOT FreeRTOS_FIND_COMPONENTS)
    set(FreeRTOS_FIND_COMPONENTS ARM_CM0 ARM_CM3 ARM_CM4F ARM_CM7)
endif()
list(REMOVE_DUPLICATES FreeRTOS_FIND_COMPONENTS)

set(FreeRTOS_HEAPS 1 2 3 4 5)

load_from_environment(FREERTOS_PATH)

if(NOT FREERTOS_PATH OR NOT EXISTS "${FREERTOS_PATH}")
    message(STATUS "No FREERTOS_PATH specified. Looking after Git Submodule stm32_mw_freertos otherwise cloning.")

    load_from_environment(FREERTOS_GIT_URL)
    load_from_environment(FREERTOS_GIT_TAG)
    if(NOT FREERTOS_GIT_URL AND NOT FREERTOS_GIT_TAG)
        load_git_submodule(other stm32_mw_freertos Source FREERTOS_PATH)
    endif()

    if(NOT FREERTOS_PATH)
        if(FREERTOS_GIT_URL)
            git_clone(${FREERTOS_GIT_URL} other/stm32_mw_freertos "${FREERTOS_GIT_TAG}" include FREERTOS_PATH)
        else()
            git_clone_st(other stm32_mw_freertos "${FREERTOS_GIT_TAG}" Source FREERTOS_PATH)
        endif()
    endif()

    if(FREERTOS_PATH AND ${VERBOSE})
        message("Submodule path: ${FREERTOS_PATH}")
    endif()
endif()

find_path(FreeRTOS_COMMON_INCLUDE
          NAMES FreeRTOS.h
          PATHS "${FREERTOS_PATH}" "${FREERTOS_PATH}/FreeRTOS"
          PATH_SUFFIXES "Source/include" "include"
          NO_DEFAULT_PATH)
list(APPEND FreeRTOS_INCLUDE_DIRS "${FreeRTOS_COMMON_INCLUDE}")

find_path(FreeRTOS_SOURCE_DIR
          NAMES tasks.c
          PATHS "${FREERTOS_PATH}" "${FREERTOS_PATH}/FreeRTOS"
          PATH_SUFFIXES "Source" "."
          NO_DEFAULT_PATH)
if(NOT (TARGET FreeRTOS))
    add_library(FreeRTOS INTERFACE IMPORTED)
    target_sources(FreeRTOS
                   INTERFACE
                   "${FreeRTOS_SOURCE_DIR}/tasks.c"
                   "${FreeRTOS_SOURCE_DIR}/list.c"
                   "${FreeRTOS_SOURCE_DIR}/queue.c")
    target_include_directories(FreeRTOS INTERFACE "${FreeRTOS_COMMON_INCLUDE}")
    target_compile_definitions(FreeRTOS INTERFACE USE_FREERTOS)

    if (${STRIP_UNUSED_CODE})
        # Make sure that uxTopUsedPriority remains in the output code to be used by the debugger
        target_link_options(FreeRTOS INTERFACE "-Wl,--undefined=uxTopUsedPriority")
    endif()
endif()

if(NOT (TARGET FreeRTOS::Coroutine))
    add_library(FreeRTOS::Coroutine INTERFACE IMPORTED)
    target_sources(FreeRTOS::Coroutine INTERFACE "${FreeRTOS_SOURCE_DIR}/croutine.c")
    target_link_libraries(FreeRTOS::Coroutine
                          INTERFACE FreeRTOS)
endif()

if(NOT (TARGET FreeRTOS::EventGroups))
    add_library(FreeRTOS::EventGroups INTERFACE IMPORTED)
    target_sources(FreeRTOS::EventGroups INTERFACE "${FreeRTOS_SOURCE_DIR}/event_groups.c")
    target_link_libraries(FreeRTOS::EventGroups
                          INTERFACE FreeRTOS)
endif()

if(NOT (TARGET FreeRTOS::StreamBuffer))
    add_library(FreeRTOS::StreamBuffer INTERFACE IMPORTED)
    target_sources(FreeRTOS::StreamBuffer INTERFACE "${FreeRTOS_SOURCE_DIR}/stream_buffer.c")
    target_link_libraries(FreeRTOS::StreamBuffer
                          INTERFACE FreeRTOS)
endif()

if(NOT (TARGET FreeRTOS::Timers))
    add_library(FreeRTOS::Timers INTERFACE IMPORTED)
    target_sources(FreeRTOS::Timers INTERFACE "${FreeRTOS_SOURCE_DIR}/timers.c")
    target_link_libraries(FreeRTOS::Timers
                          INTERFACE FreeRTOS)
endif()

if(NOT (TARGET FreeRTOS::CMSIS))
    if(EXISTS "${FreeRTOS_SOURCE_DIR}/CMSIS_RTOS")
        add_library(FreeRTOS::CMSIS INTERFACE IMPORTED)
        target_sources(FreeRTOS::CMSIS INTERFACE "${FreeRTOS_SOURCE_DIR}/CMSIS_RTOS/cmsis_os.c")
        target_include_directories(FreeRTOS::CMSIS INTERFACE "${FreeRTOS_SOURCE_DIR}/CMSIS_RTOS")
        target_link_libraries(FreeRTOS::CMSIS
                              INTERFACE FreeRTOS)
        target_compile_definitions(FreeRTOS::CMSIS INTERFACE USE_FREERTOS_CMSIS)
    endif()
endif()

foreach(HEAP ${FreeRTOS_HEAPS})
    if(NOT (TARGET FreeRTOS::Heap::${HEAP}))
        add_library(FreeRTOS::Heap::${HEAP} INTERFACE IMPORTED)
        target_sources(FreeRTOS::Heap::${HEAP} INTERFACE "${FreeRTOS_SOURCE_DIR}/portable/MemMang/heap_${HEAP}.c")
        target_link_libraries(FreeRTOS::Heap::${HEAP}
                              INTERFACE FreeRTOS)
    endif()
endforeach()

foreach(PORT ${FreeRTOS_FIND_COMPONENTS})
    find_path(FreeRTOS_${PORT}_PATH
              NAMES portmacro.h
              PATHS "${FREERTOS_PATH}" "${FREERTOS_PATH}/FreeRTOS"
              PATH_SUFFIXES "Source/portable/GCC/${PORT}"
                            "Source/portable/GCC/${PORT}/r0p1"
                            "portable/GCC/${PORT}"
                            "portable/GCC/${PORT}/r0p1"
              NO_DEFAULT_PATH)
    list(APPEND FreeRTOS_INCLUDE_DIRS "${FreeRTOS_${PORT}_PATH}")

    find_file(FreeRTOS_${PORT}_SOURCE NAMES port.c PATHS "${FreeRTOS_${PORT}_PATH}" NO_DEFAULT_PATH)
    if(NOT (TARGET FreeRTOS::${PORT}))
        add_library(FreeRTOS::${PORT} INTERFACE IMPORTED)
        target_link_libraries(FreeRTOS::${PORT}
                              INTERFACE FreeRTOS)
        target_sources(FreeRTOS::${PORT} INTERFACE "${FreeRTOS_${PORT}_SOURCE}")
        target_include_directories(FreeRTOS::${PORT} INTERFACE "${FreeRTOS_${PORT}_PATH}")
    endif()

    if(FreeRTOS_${PORT}_PATH AND FreeRTOS_${PORT}_SOURCE AND FreeRTOS_COMMON_INCLUDE AND FreeRTOS_SOURCE_DIR)
        set(FreeRTOS_${PORT}_FOUND TRUE)
    else()
        set(FreeRTOS_${PORT}_FOUND FALSE)
    endif()
endforeach()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(FreeRTOS
                                  REQUIRED_VARS
                                  FreeRTOS_INCLUDE_DIRS
                                  FOUND_VAR
                                  FreeRTOS_FOUND
                                  HANDLE_COMPONENTS)
