option(VERBOSE "Verbose CMake" OFF)
option(STRIP_UNUSED_CODE "Strip Unused code (reduces binary size)" ON)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/stm32_gcc.cmake)

define_property(TARGET
        PROPERTY DEVICE_FAMILY
        BRIEF_DOCS "Target Device family"
        FULL_DOCS "Target Device family, e.g. STM32F4")

define_property(TARGET
        PROPERTY DEVICE_FAMILY_SHORT
        BRIEF_DOCS "Target Device family (short version)"
        FULL_DOCS "Target Device family (short version), e.g. F4")

define_property(TARGET PROPERTY DEVICE_NAME BRIEF_DOCS "Target Device name" FULL_DOCS "Target Device name, e.g. F743")

define_property(TARGET PROPERTY DEVICE_CORE BRIEF_DOCS "Target Device core" FULL_DOCS "Target Device core, e.g. M7")

define_property(TARGET
        PROPERTY LIBRARY_OPTIONS
        BRIEF_DOCS "List of options for a particular library"
        FULL_DOCS "List of options for a particular library")

define_property(TARGET
        PROPERTY CONFIGURABLE_LIBRARY_OPTIONS
        BRIEF_DOCS "List of configurable options for a particular library"
        FULL_DOCS "List of configurable options for a particular library")

function(get_openocd_config_name_for_target TARGET CFG)
    get_target_property(LIBRARIES ${TARGET} LINK_LIBRARIES)
    list(FILTER LIBRARIES INCLUDE REGEX "CMSIS::STM32")
    #foreach(LIBRARY ${LIBRARIES})
    #    message(STATUS ${LIBRARY})
    #endforeach()
    list(LENGTH LIBRARIES NUM_LIBRARIES)
    if (NUM_LIBRARIES EQUAL 1)
        get_target_property(DEPENDENCIES ${LIBRARIES} MANUALLY_ADDED_DEPENDENCIES)
        list(FILTER DEPENDENCIES INCLUDE REGEX "^OPENOCD_[A-Z0-9][A-Z0-9]_[A-Z0-9][A-Z0-9]$")
        list(LENGTH DEPENDENCIES NUM_DEPENDENCIES)
        if (NUM_DEPENDENCIES EQUAL 1)
            message("[${TARGET}] OpenOCD config file: ${DEPENDENCIES}[_FREERTOS?].cfg")
            set(${CFG} ${DEPENDENCIES} PARENT_SCOPE)
        else ()
            message(FATAL_ERROR "More than 1 OpenOCD config defined")
        endif ()
    endif ()
endfunction()

function(stm32_add_flash_target TARGET)
    if (TARGET flash_${TARGET})
        return()
    endif ()

    get_property(TMP TARGET ${TARGET} PROPERTY TYPE)
    if (NOT TMP STREQUAL "EXECUTABLE")
        return()
    endif ()

    # Find OpenOCD path
    find_program(OPENOCD_BIN "openocd")
    if (OPENOCD_BIN)
        get_filename_component(OPENOCD_PATH ${OPENOCD_BIN} DIRECTORY)
        get_filename_component(OPENOCD_PATH ${OPENOCD_PATH} DIRECTORY)
        #message("OpenOCD path: ${OPENOCD_PATH}")

        get_property(DEVICE_FAMILY TARGET ${TARGET} PROPERTY DEVICE_FAMILY)
        if (NOT DEVICE_FAMILY)
            message(FATAL_ERROR "Could not get device family to add OpenOCD config")
        endif ()

        string(TOLOWER ${DEVICE_FAMILY} FAMILY_L)

        if (EXISTS "${OPENOCD_PATH}/scripts/target/${FAMILY_L}x.cfg")
            set(OPENOCD_TARGET_CFG ${FAMILY_L}x.cfg)
        elseif (EXISTS "${OPENOCD_PATH}/scripts/target/${FAMILY_L}.cfg")
            set(OPENOCD_TARGET_CFG ${FAMILY_L}.cfg)
        else ()
            message(FATAL_ERROR "Could not find OpenOCD config: ${FAMILY_L}.cfg or ${FAMILY_L}x.cfg")
        endif ()

        set(OPENOCD_OUTPUT_CFG "${CMAKE_CURRENT_BINARY_DIR}/${TARGET}_openocd.cfg")
        add_custom_command(OUTPUT "${OPENOCD_OUTPUT_CFG}"
                COMMAND ${CMAKE_COMMAND}
                -DOPENOCD_CFG="${OPENOCD_OUTPUT_CFG}"
                -DOPENOCD_FREERTOS=$<IF:$<BOOL:$<FILTER:$<TARGET_PROPERTY:${TARGET},LINK_LIBRARIES>,INCLUDE,FreeRTOS>>,1,0>
                -DOPENOCD_TARGET_CFG="${OPENOCD_TARGET_CFG}" -P
                "${STM32_CMAKE_DIR}/stm32/openocd_cfg.cmake")


        set(OPENOCD_CFG "${CMAKE_CURRENT_BINARY_DIR}/${TARGET}_openocd.cfg")
        #message("OpenOCD config file: ${OPENOCD_CFG}")
        #        add_custom_command(OUTPUT "${OPENOCD_CFG}"
        #                COMMAND ${CMAKE_COMMAND}
        #                -DOPENOCD_CFG="${OPENOCD_CFG}"
        #                -P "${STM32_CMAKE_DIR}/stm32/openocd_cfg.cmake"
        #                )

        # cmake-format: off
        add_custom_target(flash_${TARGET}
                COMMAND ${OPENOCD_BIN} -f ${OPENOCD_CFG} -c "gdb_memory_map disable" -c "init" -c "targets" -c
                "reset halt"
                #-c "load_image $<TARGET_FILE:${PROJECT_NAME}.elf>"
                -c "flash write_image erase $<TARGET_FILE:${TARGET}>"
                #-c "flash write_image erase ${BIN_FILE} 0x08000000"
                -c
                "reset halt"
                -c
                "verify_image $<TARGET_FILE:${TARGET}>"
                -c
                "reset run"
                -c
                "exit" # exit, resume or shutdown
                WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                DEPENDS ${OPENOCD_CFG} ${TARGET} $<TARGET_FILE:${TARGET}>)

        add_custom_target(openocd_${TARGET}
                COMMAND ${OPENOCD_BIN}
                -f ${OPENOCD_OUTPUT_CFG}
                -c "gdb_memory_map disable"
                WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                DEPENDS ${OPENOCD_CFG} ${TARGET} $<TARGET_FILE:${TARGET}>)
        # cmake-format: on
    endif ()

    # Find dfu-util (http://dfu-util.sourceforge.net/dfuse.html) for flashing DFU-enabled devices over USB
    find_program(DFU_UTIL_BIN "dfu-util")
    if (DFU_UTIL_BIN)
        add_custom_target(dfu_${TARGET}
                COMMAND ${DFU_UTIL_BIN} -a 0 -i 0 -s 0x08000000:leave -D ${CMAKE_CURRENT_BINARY_DIR}/${TARGET}.bin
                WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                DEPENDS ${TARGET}_bin)
    endif()
endfunction()

function(stm32_add_gdb_target TARGET)
    if (TARGET gdb_${TARGET})
        return()
    endif ()

    # Start debugging with GDB (openocd needs to run in background)
    add_custom_target(gdb_${TARGET}
            COMMAND ${CMAKE_GDB} -X ${STM32_CMAKE_DIR}/gdb.cfg $<TARGET_FILE:${TARGET}>
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            DEPENDS ${OPENOCD_CFG} $<TARGET_FILE:${TARGET}>)
endfunction()

function(stm32_convert_to_hex TARGET)
    #     add_custom_command(TARGET ${TARGET}
    #             POST_BUILD
    #             COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${TARGET}> $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.hex
    #             COMMENT "Generating HEX file for ${TARGET}")
    add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${TARGET}.hex"
            COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${TARGET}>
            $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.hex
            COMMENT "Generating HEX file for ${TARGET}")
    add_custom_target(${TARGET}_hex
            COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${TARGET}>
            $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.hex
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            DEPENDS ${TARGET} $<TARGET_FILE:${TARGET}>
            COMMENT "Generating HEX file for ${TARGET}")
    set_property(
            TARGET ${TARGET}
            APPEND
            PROPERTY ADDITIONAL_CLEAN_FILES
            $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.hex # add the hex file to be removed when running 'make clean'
    )
endfunction()

function(stm32_convert_to_binary TARGET)
    #    add_custom_command(TARGET ${TARGET}
    #            POST_BUILD
    #            COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET}> $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.bin
    #            COMMENT "Generating BIN file for ${TARGET}")
    add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${TARGET}.bin"
            COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET}>
            $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.bin
            COMMENT "Generating BIN file for ${TARGET}")
    add_custom_target(${TARGET}_bin
            COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET}>
            $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.bin
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            DEPENDS ${TARGET} $<TARGET_FILE:${TARGET}>
            COMMENT "Generating BIN file for ${TARGET}")
    set_property(
            TARGET ${TARGET}
            APPEND
            PROPERTY
            ADDITIONAL_CLEAN_FILES
            $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.bin # add the binary file to be removed when running 'make clean'
    )
endfunction()

function(stm32_print_target_size_after_build TARGET)
    add_custom_command(TARGET ${TARGET} POST_BUILD COMMAND echo "\n" COMMAND ${CMAKE_SIZE} "$<TARGET_FILE:${TARGET}>")
endfunction()

function(add_device_family_property TARGET)
    get_property(TMP TARGET ${TARGET} PROPERTY TYPE)
    if (NOT TMP STREQUAL "EXECUTABLE")
        return()
    endif ()

    get_property(DEVICE_FAMILY TARGET ${TARGET} PROPERTY DEVICE_FAMILY)
    if (DEVICE_FAMILY)
        return()
    endif ()

    get_target_property(LIBRARIES ${TARGET} LINK_LIBRARIES)
    list(FILTER LIBRARIES INCLUDE REGEX "CMSIS::STM32")
    foreach (library ${LIBRARIES})
        string(REGEX MATCH
                "^CMSIS::STM32::([A-Z][0-9])([0-9A-Z][0-9])?([A-Z][0-9A-Z])?(::)?(M[47])?$"
                match
                ${library})
        if (match)
            set(FAMILY STM32${CMAKE_MATCH_1})
            set(FAMILY_SHORT ${CMAKE_MATCH_1})
            set(NAME ${CMAKE_MATCH_1}${CMAKE_MATCH_2}${CMAKE_MATCH_3})
            set(CORE ${CMAKE_MATCH_5})

            if (CORE)
                message("[${TARGET}] Device Family: ${FAMILY}, Chip: ${NAME}, Core: ${CORE}")
            else ()
                message("[${TARGET}] Device Family: ${FAMILY}, Chip: ${NAME}")
            endif ()

            set_property(TARGET ${TARGET} PROPERTY DEVICE_FAMILY ${FAMILY})
            set_property(TARGET ${TARGET} PROPERTY DEVICE_FAMILY_SHORT ${FAMILY_SHORT})
            set_property(TARGET ${TARGET} PROPERTY DEVICE_NAME ${NAME})
            if (CMAKE_MATCH_5)
                set_property(TARGET ${TARGET} PROPERTY DEVICE_CORE ${CORE})
            endif ()
            return()
        endif ()
    endforeach ()
endfunction()

function(ADD_LIBRARY_OPTION_WITH_DEPENDENCY LIBRARY OPTION EXTRA_DEPENDENCIES)
    #if(${LIBRARY}_OPTIONS)
    #    set(${LIBRARY}_OPTIONS "${${LIBRARY}_OPTIONS} ${OPTION}" PARENT_SCOPE)
    #else()
    #    set(${LIBRARY}_OPTIONS "${OPTION}" PARENT_SCOPE)
    #endif()
    #set_property(GLOBAL APPEND PROPERTY LIBRARY_OPTIONS "${LIBRARY}_USE_${OPTION}")

    string(TOUPPER ${LIBRARY} LIBRARY_U)
    string(TOUPPER ${OPTION} OPTION_U)

    target_compile_definitions(
            ${LIBRARY} INTERFACE
            $<$<IN_LIST:${LIBRARY_U}_USE_${OPTION_U},$<TARGET_PROPERTY:LIBRARY_OPTIONS>>:"${LIBRARY_U}_USE_${OPTION_U}">
    )

    target_link_libraries(
            ${LIBRARY}
            INTERFACE
            $<$<IN_LIST:${LIBRARY_U}_USE_${OPTION_U},$<TARGET_PROPERTY:LIBRARY_OPTIONS>>:${EXTRA_DEPENDENCIES}>
    )
endfunction()

function(ADD_LIBRARY_OPTION LIBRARY OPTION)
    string(TOUPPER ${LIBRARY} LIBRARY_U)
    string(TOUPPER ${OPTION} OPTION_U)

    target_compile_definitions(
            ${LIBRARY} INTERFACE
            $<$<IN_LIST:${LIBRARY_U}_USE_${OPTION_U},$<TARGET_PROPERTY:LIBRARY_OPTIONS>>:"${LIBRARY_U}_USE_${OPTION_U}">
    )
endfunction()

function(ENABLE_OPTION TARGET LIBRARY OPTION)
    # get_property(options GLOBAL PROPERTY LIBRARY_OPTIONS)
    # list(FILTER options INCLUDE REGEX "^${LIBRARY}_USE_(.*)$")

    # if(NOT options)
    #     message(FATAL_ERROR "No options found for library ${LIBRARY}")
    # endif()

    # if(NOT (${LIBRARY}_USE_${OPTION} IN_LIST options))
    #     message(FATAL_ERROR "Option ${OPTION} not found for library ${LIBRARY}")
    # endif()
    # if(NOT TARGET ${TARGET})
    #     message(FATAL_ERROR "Target ${TARGET} not found")
    # endif()
    string(TOUPPER ${LIBRARY} LIBRARY_U)
    string(TOUPPER ${OPTION} OPTION_U)

    message(STATUS "Adding compile definition: ${LIBRARY_U}_USE_${OPTION_U}")
    #target_compile_definitions(${TARGET} PUBLIC ${LIBRARY_U}_USE_${OPTION})
    set_property(TARGET ${TARGET}
            APPEND
            PROPERTY LIBRARY_OPTIONS "${LIBRARY_U}_USE_${OPTION_U}")

    # For debugging
    if(NOT TARGET print_options_${TARGET})
        add_custom_target(
                print_options_${TARGET}
                COMMAND ${CMAKE_COMMAND} -E echo
                '$<TARGET_PROPERTY:${TARGET},LIBRARY_OPTIONS>')
    endif()
endfunction()

function(ADD_CONFIGURABLE_LIBRARY_OPTION LIBRARY OPTION)
    string(TOUPPER ${LIBRARY} LIBRARY_U)
    string(TOUPPER ${OPTION} OPTION_U)
    target_compile_definitions(
            ${LIBRARY} INTERFACE
            $<FILTER:$<TARGET_PROPERTY:CONFIGURABLE_LIBRARY_OPTIONS>,INCLUDE,^${LIBRARY_U}_${OPTION_U}=.*$>
    )
endfunction()

function(SET_CONFIGURABLE_OPTION TARGET LIBRARY OPTION VALUE)
    string(TOUPPER ${LIBRARY} LIBRARY_U)
    string(TOUPPER ${OPTION} OPTION_U)
    message(
            STATUS "Adding compile definition: ${LIBRARY_U}_${OPTION_U}=${VALUE}")
    set_property(TARGET ${TARGET}
            APPEND
            PROPERTY CONFIGURABLE_LIBRARY_OPTIONS
            "${LIBRARY_U}_${OPTION_U}=${VALUE}")

    # For debugging
    if(NOT TARGET print_configurable_options_${TARGET})
        add_custom_target(
                print_configurable_options_${TARGET}
                COMMAND ${CMAKE_COMMAND} -E echo
                '$<TARGET_PROPERTY:${TARGET},CONFIGURABLE_LIBRARY_OPTIONS>')
    endif()
endfunction()

# Tap into (override) add_executable to add a few extra functions
function(add_executable TARGET)
    _add_executable(${TARGET} ${ARGN}) # Call the original function
    stm32_print_target_size_after_build(${TARGET})
    #stm32_add_flash_target(${TARGET})
    stm32_convert_to_hex(${TARGET})
    stm32_convert_to_binary(${TARGET})
    stm32_add_gdb_target(${TARGET})

    # Generate map file
    target_link_options(${TARGET} PUBLIC "-Wl,-Map=${TARGET}.map")
    set_property(TARGET ${TARGET}
            APPEND
            PROPERTY ADDITIONAL_CLEAN_FILES
            ${TARGET}.map # add the map file to be removed when running 'make clean'
            )
endfunction()

function(target_link_libraries TARGET)
    _target_link_libraries(${TARGET} ${ARGN}) # Call the original function
    add_device_family_property(${TARGET})
    stm32_add_flash_target(${TARGET})
endfunction()

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/misc/DefaultInterrupts misc/DefaultInterrupts)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/misc/Syscalls misc/Syscalls)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/misc/FreeRTOSHeapNewlib misc/FreeRTOSHeapNewlib)