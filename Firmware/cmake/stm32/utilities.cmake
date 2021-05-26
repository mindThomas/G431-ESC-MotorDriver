function(stm32_util_create_family_targets FAMILY)
    set(CORES ${ARGN})
    list(LENGTH CORES NUM_CORES)
    if(${NUM_CORES} EQUAL 0)
        set(CORE "")
        set(CORE_C "")
    elseif(${NUM_CORES} EQUAL 1)
        set(CORE "_${CORES}")
        set(CORE_C "::${CORES}")
    else()
        message(FATAL_ERROR "Expected at most one core for family ${FAMILY}: ${CORES}")
    endif()

    if(NOT (TARGET STM32::${FAMILY}${CORE_C}))
        add_library(STM32::${FAMILY}${CORE_C} INTERFACE IMPORTED)
        target_compile_options(
            STM32::${FAMILY}${CORE_C}
            INTERFACE --sysroot="${TOOLCHAIN_SYSROOT}"
                      -mthumb
                      -mabi=aapcs
                      -fno-strict-aliasing
                      -fno-builtin
                      -ffast-math
                      -ffreestanding
                      -fno-exceptions #-nostartfiles -fno-rtti
                      -fno-common
                      -fmessage-length=0
                      -fstack-usage
                      #-fdump-ipa-cgraph # dump call graph
                      -Wall
                      #-Wall -Wextra -Wpedantic # Be extra verbose (throw more warning) when compiling
                      #-Werror # report all warnings as errors
                      #$<$<CONFIG:Debug>:-Og>
                      #$<$<CONFIG:Release>:-Os>
            )
        target_link_options(STM32::${FAMILY}${CORE_C}
                            INTERFACE
                            --sysroot="${TOOLCHAIN_SYSROOT}"
                            -mthumb #-mthumb-interwork
                            -mabi=aapcs
                            -Wl,--print-memory-usage
                            -Wl,--start-group
                            -lc
                            -lm
                            #-lnosys
                            #-lstdc++
                            #-lsupc++
                            -Wl,--end-group
                            -Wl,--cref
                            #$<$<CONFIG:Debug>:-Og>
                            #$<$<CONFIG:Release>:-Os -s>
                            )
        target_compile_definitions(STM32::${FAMILY}${CORE_C}
                                   INTERFACE
                                   STM32${FAMILY}
                                   "__weak=__attribute__((weak))"
                                   "__packed=__attribute__((__packed__))"
                                   ARM_MATH_MATRIX_CHECK
                                   ARM_MATH_ROUNDING)

        if(${STRIP_UNUSED_CODE})
            target_compile_options(STM32::${FAMILY}${CORE_C} INTERFACE -ffunction-sections -fdata-sections)
            target_link_options(STM32::${FAMILY}${CORE_C} INTERFACE -Wl,--gc-sections)
        endif()
    endif()
    foreach(TYPE ${STM32_${FAMILY}_TYPES})
        if(NOT (TARGET STM32::${TYPE}${CORE_C}))
            add_library(STM32::${TYPE}${CORE_C} INTERFACE IMPORTED)
            target_link_libraries(STM32::${TYPE}${CORE_C}
                                  INTERFACE STM32::${FAMILY}${CORE_C})
            target_compile_definitions(STM32::${TYPE}${CORE_C} INTERFACE STM32${TYPE})
        endif()
    endforeach()
endfunction()

function(load_from_environment VARIABLE)
    if(NOT ${VARIABLE} AND DEFINED ENV{${VARIABLE}})
        set(${VARIABLE} $ENV{${VARIABLE}} PARENT_SCOPE)
    endif()
endfunction()

include(FetchContent)

set(STM32_FETCH_FAMILIES F0 F1 F2 F3 F4 F7 G0 G4 H7 L0 L1 L4 L5)
set(STM32_FETCH_CUBE_VERSIONS
    v1.11.1
    v1.8.1
    v1.9.0
    v1.11.1
    v1.25.1
    v1.16.0
    v1.4.0
    v1.3.0
    v1.8.0
    v1.11.3
    v1.10.0
    v1.16.0
    v1.3.1)
set(STM32_FETCH_CMSIS_VERSIONS
    v2.3.4
    v4.3.1
    v2.2.4
    v2.3.4
    v2.6.4
    v1.2.5
    v1.4.0
    v1.2.0
    v1.9.0
    v1.9.0
    v2.3.1
    v1.7.0
    v1.0.3)
set(STM32_FETCH_HAL_VERSIONS
    v1.7.4
    v1.1.6
    v1.2.5
    v1.5.4
    v1.7.9
    v1.2.8
    v1.4.0
    v1.2.0
    v1.9.0
    v1.10.3
    v1.4.2
    v1.12.0
    v1.0.3)

# STM32-CMSIS
load_from_environment(STM32_CMSIS_GIT_URL)
if(STM32_CMSIS_GIT_URL)
    set(CMSIS_GIT_URL ${STM32_CMSIS_GIT_URL})
else()
    set(CMSIS_GIT_URL "https://github.com/STMicroelectronics/cmsis_core/")
endif()

load_from_environment(STM32_CMSIS_GIT_TAG)
if(STM32_CMSIS_GIT_TAG)
    set(CMSIS_GIT_TAG ${STM32_CMSIS_GIT_TAG})
else()
    set(CMSIS_GIT_TAG "v5.6.0")
endif()

fetchcontent_declare(STM32-CMSIS GIT_REPOSITORY ${CMSIS_GIT_URL} GIT_TAG ${CMSIS_GIT_TAG} GIT_PROGRESS TRUE)

set(IDX 0)
foreach(FAMILY ${STM32_FETCH_FAMILIES})
    string(TOLOWER ${FAMILY} FAMILY_L)

    list(GET STM32_FETCH_CUBE_VERSIONS ${IDX} CUBE_VERSION)

    load_from_environment(STM32_CMSIS_${FAMILY}_GIT_TAG)
    if(STM32_CMSIS_${FAMILY}_GIT_TAG)
        set(CMSIS_VERSION ${STM32_CMSIS_${FAMILY}_GIT_TAG})
    else()
        list(GET STM32_FETCH_CMSIS_VERSIONS ${IDX} CMSIS_VERSION)
    endif()

    load_from_environment(STM32_HAL_${FAMILY}_GIT_TAG)
    if(STM32_HAL_${FAMILY}_GIT_TAG)
        set(HAL_VERSION ${STM32_HAL_${FAMILY}_GIT_TAG})
    else()
        list(GET STM32_FETCH_HAL_VERSIONS ${IDX} HAL_VERSION)
    endif()

    fetchcontent_declare(STM32Cube${FAMILY}
                         GIT_REPOSITORY
                         https://github.com/STMicroelectronics/STM32Cube${FAMILY}/
                         GIT_TAG
                         ${CUBE_VERSION}
                         GIT_PROGRESS
                         TRUE)

    load_from_environment(STM32_CMSIS_${FAMILY}_GIT_URL)
    if(STM32_CMSIS_${FAMILY}_GIT_URL)
        set(CMSIS_${FAMILY}_GIT_URL ${STM32_CMSIS_${FAMILY}_GIT_URL})
    else()
        set(CMSIS_${FAMILY}_GIT_URL "https://github.com/STMicroelectronics/cmsis_device_${FAMILY_L}/")
    endif()
    fetchcontent_declare(STM32-CMSIS-${FAMILY}
                         GIT_REPOSITORY
                         ${CMSIS_${FAMILY}_GIT_URL}
                         GIT_TAG
                         ${CMSIS_VERSION}
                         GIT_PROGRESS
                         TRUE)

    load_from_environment(STM32_HAL_${FAMILY}_GIT_URL)
    if(STM32_HAL_${FAMILY}_GIT_URL)
        set(HAL_${FAMILY}_GIT_URL ${STM32_HAL_${FAMILY}_GIT_URL})
    else()
        set(HAL_${FAMILY}_GIT_URL "https://github.com/STMicroelectronics/stm32${FAMILY_L}xx_hal_driver/")
    endif()
    fetchcontent_declare(STM32-HAL-${FAMILY}
                         GIT_REPOSITORY
                         ${HAL_${FAMILY}_GIT_URL}
                         GIT_TAG
                         ${HAL_VERSION}
                         GIT_PROGRESS
                         TRUE)
    math(EXPR IDX "${IDX} + 1")
endforeach()

function(stm32_fetch_cube)
    foreach(FAMILY ${ARGV})
        set(CUBE_NAME STM32Cube${FAMILY})
        string(TOLOWER ${CUBE_NAME} CUBE_NAME_L)

        if(STM32_CUBE_${FAMILY}_PATH)
            message(INFO "STM32_CUBE_${FAMILY}_PATH specified, skipping fetch for ${CUBE_NAME}")
            continue()
        endif()

        fetchcontent_getproperties(${CUBE_NAME} POPULATED CUBE_POPULATED)
        if(NOT CUBE_POPULATED)
            set(FETCHCONTENT_QUIET FALSE) # To see progress
            fetchcontent_populate(${CUBE_NAME})
        endif()

        set(STM32_CUBE_${FAMILY}_PATH ${${CUBE_NAME_L}_SOURCE_DIR} PARENT_SCOPE)
    endforeach()
endfunction()

function(stm32_fetch_cmsis)
    if(NOT STM32_CMSIS_PATH)
        if(NOT STM32-CMSIS_POPULATED)
            set(FETCHCONTENT_QUIET FALSE) # To see progress
            fetchcontent_populate(STM32-CMSIS)
        endif()

        set(STM32_CMSIS_PATH ${stm32-cmsis_SOURCE_DIR} PARENT_SCOPE)
    else()
        message(INFO "STM32_CMSIS_PATH specified, skipping fetch for STM32-CMSIS")
    endif()
    foreach(FAMILY ${ARGV})
        set(CMSIS_NAME STM32-CMSIS-${FAMILY})
        string(TOLOWER ${CMSIS_NAME} CMSIS_NAME_L)

        if(STM32_CMSIS_${FAMILY}_PATH)
            message(INFO "STM32_CMSIS_${FAMILY}_PATH specified, skipping fetch for ${CMSIS_NAME}")
            continue()
        endif()

        fetchcontent_getproperties(${CMSIS_NAME_L} POPULATED CMSIS_POPULATED)
        if(NOT CMSIS_POPULATED)
            set(FETCHCONTENT_QUIET FALSE) # To see progress
            fetchcontent_populate(${CMSIS_NAME})
        endif()

        set(STM32_CMSIS_${FAMILY}_PATH ${${CMSIS_NAME_L}_SOURCE_DIR} PARENT_SCOPE)
    endforeach()
endfunction()

function(stm32_fetch_hal)
    foreach(FAMILY ${ARGV})
        set(HAL_NAME STM32-HAL-${FAMILY})
        string(TOLOWER ${HAL_NAME} HAL_NAME_L)

        if(STM32_HAL_${FAMILY}_PATH)
            message(INFO "STM32_HAL_${FAMILY}_PATH specified, skipping fetch for ${HAL_NAME}")
            continue()
        endif()

        fetchcontent_getproperties(${HAL_NAME} POPULATED HAL_POPULATED)
        if(NOT HAL_POPULATED)
            set(FETCHCONTENT_QUIET FALSE) # To see progress
            fetchcontent_populate(${HAL_NAME})
        endif()

        set(STM32_HAL_${FAMILY}_PATH ${${HAL_NAME_L}_SOURCE_DIR} PARENT_SCOPE)
    endforeach()
endfunction()
