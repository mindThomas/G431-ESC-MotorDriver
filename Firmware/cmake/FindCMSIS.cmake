if(NOT CMSIS_FIND_COMPONENTS)
    set(CMSIS_FIND_COMPONENTS ${STM32_SUPPORTED_FAMILIES_LONG_NAME})
endif()
if(STM32H7 IN_LIST CMSIS_FIND_COMPONENTS)
    list(REMOVE_ITEM CMSIS_FIND_COMPONENTS STM32H7)
    list(APPEND CMSIS_FIND_COMPONENTS STM32H7_M7 STM32H7_M4)
endif()
list(REMOVE_DUPLICATES CMSIS_FIND_COMPONENTS)

if(${VERBOSE})
    message("CMSIS Components to find: ${CMSIS_FIND_COMPONENTS}")
endif()

include(stm32/devices)
include(git_submodule)

define_property(TARGET
                PROPERTY STARTUP_FILE
                BRIEF_DOCS "Startup file"
                FULL_DOCS "Specify Non-default Startup file (override)")

function(stm32_add_startup_script TARGET SCRIPT)
    get_filename_component(SCRIPT "${SCRIPT}" ABSOLUTE)
    set_target_properties(${TARGET} PROPERTIES STARTUP_FILE "${SCRIPT}")
endfunction()

function(cmsis_generate_default_linker_script FAMILY DEVICE CORE)
    if(CORE)
        set(CORE_C "::${CORE}")
        set(CORE_U "_${CORE}")
    endif()

    set(OUTPUT_LD_FILE "${CMAKE_CURRENT_BINARY_DIR}/${DEVICE}${CORE_U}.ld")

    stm32_get_memory_info(FAMILY ${FAMILY} DEVICE ${DEVICE} CORE ${CORE} FLASH SIZE FLASH_SIZE ORIGIN FLASH_ORIGIN)
    stm32_get_memory_info(FAMILY ${FAMILY} DEVICE ${DEVICE} CORE ${CORE} RAM SIZE RAM_SIZE ORIGIN RAM_ORIGIN)
    stm32_get_memory_info(FAMILY ${FAMILY} DEVICE ${DEVICE} CORE ${CORE} CCRAM SIZE CCRAM_SIZE ORIGIN CCRAM_ORIGIN)
    stm32_get_memory_info(FAMILY ${FAMILY} DEVICE ${DEVICE} CORE ${CORE} HEAP SIZE HEAP_SIZE)
    stm32_get_memory_info(FAMILY ${FAMILY} DEVICE ${DEVICE} CORE ${CORE} STACK SIZE STACK_SIZE)

    #    if(CORE)
    #        message("FAMILY=${FAMILY}, DEVICE=${DEVICE}, CORE=${CORE}, FLASH=${FLASH_SIZE}")
    #    else()
    #        message("FAMILY=${FAMILY}, DEVICE=${DEVICE}, FLASH=${FLASH_SIZE}")
    #    endif()

    add_custom_command(OUTPUT "${OUTPUT_LD_FILE}"
                       COMMAND ${CMAKE_COMMAND} -DFLASH_ORIGIN="${FLASH_ORIGIN}" -DRAM_ORIGIN="${RAM_ORIGIN}"
                               -DCCRAM_ORIGIN="${CCRAM_ORIGIN}" -DFLASH_SIZE="${FLASH_SIZE}" -DRAM_SIZE="${RAM_SIZE}"
                               -DCCRAM_SIZE="${CCRAM_SIZE}" -DSTACK_SIZE="${STACK_SIZE}" -DHEAP_SIZE="${HEAP_SIZE}"
                               -DLINKER_SCRIPT="${OUTPUT_LD_FILE}" -P "${STM32_CMAKE_DIR}/stm32/linker_ld.cmake")
    add_custom_target(CMSIS_LD_${DEVICE}${CORE_U} DEPENDS "${OUTPUT_LD_FILE}")
    add_dependencies(CMSIS::STM32::${DEVICE}${CORE_C} CMSIS_LD_${DEVICE}${CORE_U})
    stm32_add_linker_script(CMSIS::STM32::${DEVICE}${CORE_C} "${OUTPUT_LD_FILE}")
endfunction()

function(load_from_environment VARIABLE)
    if(NOT ${VARIABLE} AND DEFINED ENV{${VARIABLE}})
        set(${VARIABLE} $ENV{${VARIABLE}} PARENT_SCOPE)
    endif()
endfunction()

foreach(COMP ${CMSIS_FIND_COMPONENTS})
    string(TOLOWER ${COMP} COMP_L)
    string(TOUPPER ${COMP} COMP)

    string(REGEX MATCH
                 "^STM32([A-Z][0-9])([0-9A-Z][0-9])?([A-Z][0-9A-Z])?_?(M[47])?.*$"
                 COMP
                 ${COMP})

    if((NOT CMAKE_MATCH_1) AND (NOT CMAKE_MATCH_2))
        message(FATAL_ERROR "Unknown CMSIS component: ${COMP}")
    endif()

    set(FAMILY ${CMAKE_MATCH_1})
    if(CMAKE_MATCH_2 AND CMAKE_MATCH_3)
        set(DEVICES "${FAMILY}${CMAKE_MATCH_2}${CMAKE_MATCH_3}")
    elseif(CMAKE_MATCH_2)
        stm32_get_devices_by_family(DEVICES FAMILY ${FAMILY})
        message("REGEX=${FAMILY}${CMAKE_MATCH_2}..")
        list(FILTER DEVICES INCLUDE REGEX "${FAMILY}${CMAKE_MATCH_2}..")
        message("DEVICES=${DEVICES}")
    else()
        stm32_get_devices_by_family(DEVICES FAMILY ${FAMILY})
    endif()

    if(CMAKE_MATCH_4)
        set(CORE ${CMAKE_MATCH_4})
        set(CORE_C "::${CORE}")
        set(CORE_D "::${CORE}")
        set(CORE_U "_${CORE}")
        message("Loading FAMILY=${FAMILY}, DEVICES=${DEVICES}, CORE=${CORE}")
    else()
        unset(CORE)
        unset(CORE_C)
        unset(CORE_D)
        unset(CORE_U)
        message("Loading FAMILY=${FAMILY}, DEVICES=${DEVICES}")
    endif()

    # If no core is specified, make sure that the DEVICES do not need the core to be specified
    if(NOT CORE)
        stm32_get_cores(FAMILY_CORES FAMILY ${FAMILY})
        if(FAMILY_CORES)
            # The family includes devices with multiple cores and thus the CORE would have to be specified for those devices
            # If only a single device is specified and it contains just a single core, just that as the default
            list(LENGTH DEVICES NUM_DEVICES)
            if(${NUM_DEVICES} EQUAL 1)
                stm32_get_cores(DEVICE_CORES FAMILY ${FAMILY} DEVICE ${DEVICES})
                list(LENGTH DEVICE_CORES NUM_CORES)
                if(${NUM_CORES} EQUAL 1)
                    set(CORE_D "::${DEVICE_CORES}")
                else()
                    message(FATAL_ERROR "Missing CORE specification for ${COMP}")
                endif()
            elseif(NOT CMAKE_MATCH_3)
                stm32_get_cores(DEVICE_CORES
                                FAMILY
                                ${FAMILY}
                                DEVICE
                                "${FAMILY}${CMAKE_MATCH_2}xx"
                                TYPE
                                "${FAMILY}${CMAKE_MATCH_2}xx")
                list(LENGTH DEVICE_CORES NUM_CORES)
                if(${NUM_CORES} EQUAL 1)
                    set(CORE_D "::${DEVICE_CORES}")
                else()
                    message(FATAL_ERROR "Missing CORE specification for ${COMP}")
                endif()
            endif()
        endif()
    endif()

    string(TOLOWER ${FAMILY} FAMILY_L)

    load_from_environment(STM32_CMSIS_${FAMILY}_PATH)
    load_from_environment(STM32_CUBE_${FAMILY}_PATH)
    load_from_environment(STM32_CMSIS_PATH)
    if((NOT STM32_CMSIS_${FAMILY}_PATH)
       AND (NOT STM32_CUBE_${FAMILY}_PATH)
       OR (NOT EXISTS "${STM32_CMSIS_${FAMILY}_PATH}")
       AND (NOT EXISTS "${STM32_CUBE_${FAMILY}_PATH}"))
        string(TOLOWER CMSIS_DEVICE_${FAMILY} SUBMODULE_FOLDER)
        message(
            STATUS
                "Neither STM32_CUBE_${FAMILY}_PATH nor STM32_CMSIS_${FAMILY}_PATH specified. Looking after Git Submodule '${SUBMODULE_FOLDER}' otherwise cloning."
            )

        load_from_environment(STM32_CMSIS_${FAMILY}_GIT_URL)
        load_from_environment(STM32_CMSIS_${FAMILY}_GIT_TAG)
        if(NOT STM32_CMSIS_${FAMILY}_GIT_URL AND NOT STM32_CMSIS_${FAMILY}_GIT_TAG)
            load_git_submodule(cmsis ${SUBMODULE_FOLDER} Include STM32_CMSIS_${FAMILY}_PATH)
        endif()

        if(NOT STM32_CMSIS_${FAMILY}_PATH)
            if(STM32_CMSIS_${FAMILY}_GIT_URL)
                git_clone(${STM32_CMSIS_${FAMILY}_GIT_URL}
                          cmsis/${SUBMODULE_FOLDER}
                          "${STM32_CMSIS_${FAMILY}_GIT_TAG}"
                          Include
                          STM32_CMSIS_${FAMILY}_PATH)
            else()
                git_clone_st(cmsis
                             ${SUBMODULE_FOLDER} # repository name
                             "${STM32_CMSIS_${FAMILY}_GIT_TAG}"
                             Include
                             STM32_CMSIS_${FAMILY}_PATH)
            endif()
        endif()

        if(STM32_CMSIS_${FAMILY}_PATH AND ${VERBOSE})
            message("Submodule path: ${STM32_CMSIS_${FAMILY}_PATH}")
        endif()
    endif()

    if(NOT STM32_CMSIS_PATH OR NOT EXISTS "${STM32_CMSIS_PATH}")
        load_from_environment(STM32_CMSIS_GIT_URL)
        load_from_environment(STM32_CMSIS_GIT_TAG)
        if(NOT STM32_CMSIS_GIT_URL AND NOT STM32_CMSIS_GIT_TAG)
            load_git_submodule(cmsis cmsis_core Include STM32_CMSIS_PATH)
        endif()

        if(NOT STM32_CMSIS_PATH)
            if(STM32_CMSIS_GIT_URL)
                git_clone(${STM32_CMSIS_GIT_URL} cmsis "${STM32_CMSIS_GIT_TAG}" Include STM32_CMSIS_PATH)
            else()
                git_clone_st(cmsis cmsis_core "${STM32_CMSIS_GIT_TAG}" Include STM32_CMSIS_PATH)
            endif()
        endif()

        if(STM32_CMSIS_PATH AND ${VERBOSE})
            message("Submodule path: ${STM32_CMSIS_PATH}")
        endif()
    endif()

    find_path(CMSIS_${FAMILY}${CORE_U}_CORE_PATH
              NAMES Include/cmsis_gcc.h
              PATHS "${STM32_CMSIS_PATH}" "${STM32_CUBE_${FAMILY}_PATH}/Drivers/CMSIS"
              NO_DEFAULT_PATH)
    if(NOT CMSIS_${FAMILY}${CORE_U}_CORE_PATH)
        message("Did not find CMSIS_${FAMILY}${CORE_U}_CORE_PATH")
        continue()
    endif()

    find_path(CMSIS_${FAMILY}${CORE_U}_PATH
              NAMES Include/stm32${FAMILY_L}xx.h
              PATHS "${STM32_CMSIS_${FAMILY}_PATH}"
                    "${STM32_CUBE_${FAMILY}_PATH}/Drivers/CMSIS/Device/ST/STM32${FAMILY}xx"
              NO_DEFAULT_PATH)
    if(NOT CMSIS_${FAMILY}${CORE_U}_PATH)
        continue()
    endif()
    list(APPEND CMSIS_INCLUDE_DIRS "${CMSIS_${FAMILY}${CORE_U}_CORE_PATH}/Include"
                "${CMSIS_${FAMILY}${CORE_U}_PATH}/Include")

    if(NOT CMSIS_${FAMILY}${CORE_U}_VERSION)
        find_file(CMSIS_${FAMILY}${CORE_U}_PDSC
                  NAMES ARM.CMSIS.pdsc
                  PATHS "${CMSIS_${FAMILY}${CORE_U}_CORE_PATH}"
                  NO_DEFAULT_PATH)
        if(NOT CMSIS_${FAMILY}${CORE_U}_PDSC)
            set(CMSIS_${FAMILY}${CORE_U}_VERSION "0.0.0")
        else()
            file(STRINGS "${CMSIS_${FAMILY}${CORE_U}_PDSC}" VERSION_STRINGS
                 REGEX "<release version=\"([0-9]*\\.[0-9]*\\.[0-9]*)\" date=\"[0-9]+\\-[0-9]+\\-[0-9]+\">")
            list(GET VERSION_STRINGS 0 STR)
            string(REGEX MATCH
                         "<release version=\"([0-9]*)\\.([0-9]*)\\.([0-9]*)\" date=\"[0-9]+\\-[0-9]+\\-[0-9]+\">"
                         MATCHED
                         ${STR})
            set(CMSIS_${FAMILY}${CORE_U}_VERSION "${CMAKE_MATCH_1}.${CMAKE_MATCH_2}.${CMAKE_MATCH_3}"
                CACHE INTERNAL "CMSIS STM32${FAMILY}${CORE_U} version")
        endif()
    endif()

    set(CMSIS_${COMP}_VERSION ${CMSIS_${FAMILY}${CORE_U}_VERSION})
    set(CMSIS_VERSION ${CMSIS_${COMP}_VERSION})

    find_file(CMSIS_${FAMILY}${CORE_U}_SOURCE
              NAMES system_stm32${FAMILY_L}xx.c
              PATHS "${CMSIS_${FAMILY}${CORE_U}_PATH}/Source/Templates"
              NO_DEFAULT_PATH)
    list(APPEND CMSIS_SOURCES "${CMSIS_${FAMILY}${CORE_U}_SOURCE}")

    if(NOT CMSIS_${FAMILY}${CORE_U}_SOURCE)
        continue()
    endif()

    if(NOT (TARGET CMSIS::STM32::${FAMILY}${CORE_C}))
        add_library(CMSIS::STM32::${FAMILY}${CORE_C} INTERFACE IMPORTED)
        list(APPEND CMSIS_LIBRARIES "CMSIS::STM32::${FAMILY}${CORE_C}")
        target_link_libraries(CMSIS::STM32::${FAMILY}${CORE_C}
                              INTERFACE STM32::${FAMILY}${CORE_D})
        target_include_directories(CMSIS::STM32::${FAMILY}${CORE_C}
                                   INTERFACE "${CMSIS_${FAMILY}${CORE_U}_CORE_PATH}/Include")
        target_include_directories(CMSIS::STM32::${FAMILY}${CORE_C}
                                   INTERFACE "${CMSIS_${FAMILY}${CORE_U}_PATH}/Include")
        target_sources(CMSIS::STM32::${FAMILY}${CORE_C} INTERFACE "${CMSIS_${FAMILY}${CORE_U}_SOURCE}")
        target_link_directories(CMSIS::STM32::${FAMILY}${CORE_C} INTERFACE
                                "${CMSIS_${FAMILY}${CORE_U}_CORE_PATH}/DSP/Lib/GCC/")
        target_include_directories(CMSIS::STM32::${FAMILY}${CORE_C}
                INTERFACE "${CMSIS_${FAMILY}${CORE_U}_CORE_PATH}/DSP/Include")
        if(${VERBOSE})
            message("Adding generic library CMSIS::STM32::${FAMILY}${CORE_C}")
        endif()
    endif()

    set(DEVICES_FOUND TRUE)
    foreach(DEVICE ${DEVICES})
        stm32_get_cores(DEV_CORES FAMILY ${FAMILY} DEVICE ${DEVICE})
        if(CORE AND (NOT ${CORE} IN_LIST DEV_CORES))
            continue()
        endif()

        if(DEV_CORES AND (NOT CORE_D))
            message(FATAL_ERROR "Missing CORE specification for ${COMP}")
        endif()

        stm32_get_chip_type(${FAMILY} ${DEVICE} TYPE)
        string(TOLOWER ${DEVICE} DEVICE_L)
        string(TOLOWER ${TYPE} TYPE_L)

        find_file(CMSIS_${FAMILY}${CORE_U}_${TYPE}_STARTUP
                  NAMES startup_stm32${TYPE_L}.s
                  PATHS "${CMSIS_${FAMILY}${CORE_U}_PATH}/Source/Templates/gcc"
                  NO_DEFAULT_PATH)
        list(APPEND CMSIS_SOURCES "${CMSIS_${FAMILY}${CORE_U}_${TYPE}_STARTUP}")
        if(NOT CMSIS_${FAMILY}${CORE_U}_${TYPE}_STARTUP)
            set(DEVICES_FOUND FALSE)
            break()
        endif()

        if(NOT (TARGET CMSIS::STM32::${TYPE}${CORE_C}))
            add_library(CMSIS::STM32::${TYPE}${CORE_C} INTERFACE IMPORTED)
            list(APPEND CMSIS_LIBRARIES "CMSIS::STM32::${TYPE}${CORE_C}")
            target_link_libraries(CMSIS::STM32::${TYPE}${CORE_C}
                                  INTERFACE CMSIS::STM32::${FAMILY}${CORE_C} STM32::${TYPE}${CORE_D})
            target_sources(
                CMSIS::STM32::${TYPE}${CORE_C} INTERFACE
                $<IF:$<BOOL:$<TARGET_PROPERTY:STARTUP_FILE>>,$<TARGET_PROPERTY:STARTUP_FILE>,${CMSIS_${FAMILY}${CORE_U}_${TYPE}_STARTUP}>
                )

            if(${VERBOSE})
                message("Adding library CMSIS::STM32::${TYPE}${CORE_C} with startup file")
            endif()
        endif()

        add_library(CMSIS::STM32::${DEVICE}${CORE_C} INTERFACE IMPORTED)
        list(APPEND CMSIS_LIBRARIES "CMSIS::STM32::${DEVICE}${CORE_C}")
        target_link_libraries(CMSIS::STM32::${DEVICE}${CORE_C}
                              INTERFACE CMSIS::STM32::${TYPE}${CORE_C})
        cmsis_generate_default_linker_script(${FAMILY} ${DEVICE} "${CORE}")
        if(${VERBOSE})
            message("Adding library CMSIS::STM32::${DEVICE}${CORE_C} with startup and linker file")
        endif()
    endforeach()

    if(DEVICES_FOUND)
        set(CMSIS_${COMP}_FOUND TRUE)
    else()
        set(CMSIS_${COMP}_FOUND FALSE)
    endif()
    list(REMOVE_DUPLICATES CMSIS_INCLUDE_DIRS)
    list(REMOVE_DUPLICATES CMSIS_SOURCES)
    list(REMOVE_DUPLICATES CMSIS_LIBRARIES)
endforeach()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CMSIS
                                  REQUIRED_VARS
                                  CMSIS_INCLUDE_DIRS
                                  CMSIS_SOURCES
                                  CMSIS_LIBRARIES
                                  FOUND_VAR
                                  CMSIS_FOUND
                                  VERSION_VAR
                                  CMSIS_VERSION
                                  HANDLE_COMPONENTS)
