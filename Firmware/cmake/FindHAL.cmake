# cmake-format: off
set(HAL_DRIVERS_F0
    adc can cec comp cortex crc dac dma exti flash gpio i2c i2s irda iwdg pcd 
    pwr rcc rtc smartcard smbus spi tim tsc uart usart wwdg
)
set(HAL_EX_DRIVERS_F0
    adc crc dac flash i2c pcd pwr rcc rtc smartcard spi tim uart usart
)
set(HAL_LL_DRIVERS_F0
    adc comp crc crs dac dma exti gpio i2c pwr rcc rtc spi tim usart usb utils
)

set(HAL_DRIVERS_F1
    adc can cec cortex crc dac dma eth exti flash gpio hcd i2c i2s irda iwdg 
    mmc nand nor pccard pcd pwr rcc rtc sd smartcard spi sram tim uart usart 
    wwdg
)
set(HAL_EX_DRIVERS_F1
    adc dac flash gpio pcd rcc rtc tim
)
set(HAL_LL_DRIVERS_F1
    adc crc dac dma exti fsmc gpio i2c pwr rcc rtc sdmmc spi tim usart usb utils
)

set(HAL_DRIVERS_F2
    adc can cortex crc cryp dac dcmi dma eth exti flash gpio hash hcd i2c i2s 
    irda iwdg mmc nand nor pccard pcd pwr rcc rng rtc sd smartcard spi sram tim 
    uart usart wwdg
)
set(HAL_EX_DRIVERS_F2
    adc dac dcmi dma flash pcd pwr rcc rtc tim
)
set(HAL_LL_DRIVERS_F2
    adc crc dac dma exti fsmc gpio i2c pwr rcc rng rtc sdmmc spi tim usart usb 
    utils
)

set(HAL_DRIVERS_F3
    adc can cec comp cortex crc dac dma exti flash gpio hrtim i2c i2s irda iwdg 
    nand nor opamp pccard pcd pwr rcc rtc sdadc smartcard smbus spi sram tim tsc 
    uart usart wwdg
)
set(HAL_EX_DRIVERS_F3
    adc crc dac flash i2c i2s opamp pcd pwr rcc rtc smartcard spi tim uart usart
)
set(HAL_LL_DRIVERS_F3
    adc comp crc dac dma exti fmc gpio hrtim i2c opamp pwr rcc rtc spi tim usart 
    usb utils
)

set(HAL_DRIVERS_F4
    adc can cec cortex crc cryp dac dcmi dfsdm dma dma2d dsi eth exti flash
    flash_ramfunc fmpi2c gpio hash hcd i2c i2s irda iwdg lptim ltdc mmc nand nor 
    pccard pcd pwr qspi rcc rng rtc sai sd sdram smartcard smbus spdifrx spi 
    sram tim uart usart wwdg
)
set(HAL_EX_DRIVERS_F4
    adc cryp dac dcmi dma flash fmpi2c hash i2c i2s ltdc pcd pwr rcc rtc sai tim 
)
set(HAL_LL_DRIVERS_F4
    adc crc dac dma dma2d exti fmc fsmc gpio i2c lptim pwr rcc rng rtc sdmmc spi 
    tim usart usb utils 
)

set(HAL_DRIVERS_F7
    adc can cec cortex crc cryp dac dcmi dfsdm dma dma2d dsi eth exti flash 
    gpio hash hcd i2c i2s irda iwdg jpeg lptim ltdc mdios mmc nand nor pcd pwr 
    qspi rcc rng rtc sai sd sdram smartcard smbus spdifrx spi sram tim uart 
    usart wwdg
)
set(HAL_EX_DRIVERS_F7
    adc crc cryp dac dcmi dma flash hash i2c ltdc pcd pwr rcc rtc sai smartcard 
    spi tim uart
)
set(HAL_LL_DRIVERS_F7
    adc crc dac dma dma2d exti fmc gpio i2c lptim pwr rcc rng rtc sdmmc spi tim 
    usart usb utils
)

set(HAL_DRIVERS_G0
    adc cec comp cortex crc cryp dac dma exti flash gpio i2c i2s irda iwdg lptim 
    pwr rcc rng rtc smartcard smbus spi tim uart usart wwdg
)
set(HAL_EX_DRIVERS_G0
    adc crc cryp dac dma flash i2c pwr rcc rtc smartcard spi tim uart usart
)
set(HAL_LL_DRIVERS_G0
    adc comp crc dac dma exti gpio i2c lptim lpuart pwr rcc rng rtc spi tim ucpd 
    usart utils
)

set(HAL_DRIVERS_G4
    adc comp cordic cortex crc cryp dac dma exti fdcan flash flash_ramfunc fmac 
    gpio hrtim i2c i2s irda iwdg lptim nand nor opamp pcd pwr qspi rcc rng rtc 
    sai smartcard smbus spi sram tim uart usart wwdg
)
set(HAL_EX_DRIVERS_G4
    adc crc cryp dac dma flash i2c opamp pcd pwr rcc rtc sai smartcard spi tim 
    uart usart
)
set(HAL_LL_DRIVERS_G4
    adc comp cordic crc crs dac dma exti fmac fmc gpio hrtim i2c lptim lpuart 
    opamp pwr rcc rng rtc spi tim ucpd usart usb utils
)

set(HAL_DRIVERS_H7
    adc cec comp cortex crc cryp dac dcmi dfsdm dma dma2d dsi dts eth exti fdcan
    flash gfxmmu gpio hash hrtim hsem i2c i2s irda iwdg jpeg lptim ltdc mdios 
    mdma mmc nand nor opamp ospi otfdec pcd pssi pwr qspi ramecc rcc rng rtc sai 
    sd sdram smartcard smbus spdifrx spi sram swpmi tim uart usart wwdg
)
set(HAL_EX_DRIVERS_H7
    adc crc cryp dac dfsdm dma eth flash hash i2c i2s ltdc mmc opamp pcd pwr rcc 
    rng rtc sai sd smartcard spi tim uart usart
)
set(HAL_LL_DRIVERS_H7
    adc bdma comp crc crs dac delayblock dma dma2d exti fmc gpio hrtim i2c lptim 
    lpuart mdma opamp pwr rcc rng rtc sdmmc spi swpmi tim usart usb utils
)

set(HAL_DRIVERS_L0
    adc comp cortex crc cryp dac dma firewall flash flash_ramfunc gpio i2c i2s 
    irda iwdg lcd lptim pcd pwr rcc rng rtc smartcard smbus spi tim tsc uart 
    usart wwdg
)
set(HAL_EX_DRIVERS_L0
    adc comp crc cryp dac flash i2c pcd pwr rcc rtc smartcard tim uart
)
set(HAL_LL_DRIVERS_L0
    adc comp crc crs dac dma exti gpio i2c lptim lpuart pwr rcc rng rtc spi tim 
    usart usb utils
)

set(HAL_DRIVERS_L1
    adc comp cortex crc cryp dac dma flash flash_ramfunc gpio i2c i2s irda iwdg 
    lcd nor opamp pcd pwr rcc rtc sd smartcard spi sram tim uart usart wwdg
)
set(HAL_EX_DRIVERS_L1
    adc cryp dac flash opamp pcd pcd pwr rcc rtc tim
)
set(HAL_LL_DRIVERS_L1
    adc comp crc dac dma exti fsmc gpio i2c opamp pwr rcc rtc sdmmc spi tim 
    usart usb utils
)

set(HAL_DRIVERS_L4
    adc can comp cortex crc cryp dac dcmi dfsdm dma dma2d dsi exti firewall 
    flash flash_ramfunc gfxmmu gpio hash hcd i2c irda iwdg lcd lptim ltdc mmc 
    nand nor opamp ospi pcd pka pssi pwr qspi rcc rng rtc sai sd smartcard smbus 
    spi sram swpmi tim tsc uart usart wwdg
)
set(HAL_EX_DRIVERS_L4
    adc crc cryp dac dfsdm dma flash hash i2c ltdc mmc opamp pcd pwr rcc rng rtc 
    sai sd smartcard spi tim uart usart
)
set(HAL_LL_DRIVERS_L4
    adc comp crc crs dac dma dma2d exti fmc gpio i2c lptim lpuart opamp pka pwr 
    rcc rng rtc sdmmc spi swpmi tim usart usb utils
)

set(HAL_DRIVERS_L5
    adc comp cortex crc cryp dac dfsdm dma exti fdcan flash flash_ramfunc gpio 
    gtzc hash i2c icache irda iwdg lptim mmc nand nor opamp ospi pcd pka pwr rcc 
    rng rtc sai sd smartcard smbus spi sram tim tsc uart usart wwdg
)
set(HAL_EX_DRIVERS_L5
    adc crc cryp dac dfsdm dma flash hash i2c mmc opamp pcd pwr rcc
    rng rtc sai sd smartcard spi tim uart usart
)
set(HAL_LL_DRIVERS_L5
    adc comp crc crs dac dma exti fmc gpio i2c lptim lpuart opamp pka pwr rcc 
    rng rtc sdmmc spi tim ucpd usart usb utils
)
# cmake-format: on

foreach(FAMILY_SUFFIX ${STM32_SUPPORTED_FAMILIES_SHORT_NAME})
    list(APPEND HAL_DRIVERS ${HAL_DRIVERS_${FAMILY_SUFFIX}})
    list(APPEND HAL_LL_DRIVERS ${HAL_LL_DRIVERS_${FAMILY_SUFFIX}})
endforeach()
list(REMOVE_DUPLICATES HAL_DRIVERS)
list(REMOVE_DUPLICATES HAL_LL_DRIVERS)

# For each COMP in the list ${HAL_FIND_COMPONENTS}
foreach(COMP ${HAL_FIND_COMPONENTS})
    string(TOLOWER ${COMP} COMP_L)
    string(TOUPPER ${COMP} COMP_U)

    string(REGEX MATCH
                 "^STM32([A-Z][0-9])([0-9A-Z][0-9][A-Z][0-9A-Z])?_?(M[47])?.*$"
                 COMP_U
                 ${COMP_U})
    if(CMAKE_MATCH_1)
        list(APPEND HAL_FIND_COMPONENTS_FAMILIES ${COMP})
        continue()
    endif()
    if(${COMP_L} IN_LIST HAL_DRIVERS)
        list(APPEND HAL_FIND_COMPONENTS_DRIVERS ${COMP})
        continue()
    endif()
    string(REGEX
           REPLACE "^ll_"
                   ""
                   COMP_L
                   ${COMP_L})
    if(${COMP_L} IN_LIST HAL_LL_DRIVERS)
        list(APPEND HAL_FIND_COMPONENTS_DRIVERS_LL ${COMP})
        continue()
    endif()
    message(FATAL_ERROR "Unknown HAL component: ${COMP}")
endforeach()

if(NOT HAL_FIND_COMPONENTS_FAMILIES)
    set(HAL_FIND_COMPONENTS_FAMILIES ${STM32_SUPPORTED_FAMILIES_LONG_NAME})
endif()

if(STM32H7 IN_LIST HAL_FIND_COMPONENTS_FAMILIES)
    list(REMOVE_ITEM HAL_FIND_COMPONENTS_FAMILIES STM32H7)
    list(APPEND HAL_FIND_COMPONENTS_FAMILIES STM32H7_M7 STM32H7_M4)
endif()
list(REMOVE_DUPLICATES HAL_FIND_COMPONENTS_FAMILIES)

if((NOT HAL_FIND_COMPONENTS_DRIVERS) AND (NOT HAL_FIND_COMPONENTS_DRIVERS_LL))
    set(HAL_FIND_COMPONENTS_DRIVERS ${HAL_DRIVERS})
    set(HAL_FIND_COMPONENTS_DRIVERS_LL ${HAL_LL_DRIVERS})
endif()
list(REMOVE_DUPLICATES HAL_FIND_COMPONENTS_DRIVERS)
list(REMOVE_DUPLICATES HAL_FIND_COMPONENTS_DRIVERS_LL)

if(${VERBOSE})
    message(STATUS "Search for HAL families: ${HAL_FIND_COMPONENTS_FAMILIES}")
    message(STATUS "Search for HAL drivers: ${HAL_FIND_COMPONENTS_DRIVERS}")
    message(STATUS "Search for HAL LL drivers: ${HAL_FIND_COMPONENTS_DRIVERS_LL}")
endif()

foreach(COMP ${HAL_FIND_COMPONENTS_FAMILIES})
    string(TOUPPER ${COMP} COMP_U)

    #string(REGEX MATCH "^STM32([A-Z][0-9])([0-9A-Z][0-9][A-Z][0-9A-Z])?_?(M[47])?.*$" COMP_U ${COMP_U})
    string(REGEX MATCH
                 "^STM32([A-Z][0-9])([0-9A-Z][0-9])?([A-Z][0-9A-Z])?_?(M[47])?.*$"
                 COMP
                 ${COMP})
    if(CMAKE_MATCH_4)
        set(CORE ${CMAKE_MATCH_4})
        set(CORE_C "::${CORE}")
        set(CORE_D "::${CORE}")
        set(CORE_U "_${CORE}")
    else()
        unset(CORE)
        unset(CORE_C)
        unset(CORE_D)
        unset(CORE_U)
    endif()

    set(FAMILY ${CMAKE_MATCH_1})
    string(TOLOWER ${FAMILY} FAMILY_L)

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

    load_from_environment(STM32_HAL_${FAMILY}_PATH)
    load_from_environment(STM32_CUBE_${FAMILY}_PATH)
    if((NOT STM32_HAL_${FAMILY}_PATH)
       AND (NOT STM32_CUBE_${FAMILY}_PATH)
       OR (NOT EXISTS "${STM32_HAL_${FAMILY}_PATH}")
       AND (NOT EXISTS "${STM32_CUBE_${FAMILY}_PATH}"))
        set(SUBMODULE_FOLDER stm32${FAMILY_L}xx_hal_driver)
        message(
            STATUS
                "Neither STM32_CUBE_${FAMILY}_PATH nor STM32_HAL_${FAMILY}_PATH specified. Looking after Git Submodule '${SUBMODULE_FOLDER}' otherwise cloning."
            )

        load_from_environment(STM32_HAL_${FAMILY}_GIT_URL)
        load_from_environment(STM32_HAL_${FAMILY}_GIT_TAG)
        if(NOT STM32_HAL_${FAMILY}_GIT_URL AND NOT STM32_HAL_${FAMILY}_GIT_TAG)
            load_git_submodule(hal ${SUBMODULE_FOLDER} Inc STM32_HAL_${FAMILY}_PATH)
        endif()

        if(NOT STM32_HAL_${FAMILY}_PATH)
            if(STM32_HAL_${FAMILY}_GIT_URL)
                git_clone(${STM32_HAL_${FAMILY}_GIT_URL}
                          hal/${SUBMODULE_FOLDER}
                          "${STM32_HAL_${FAMILY}_GIT_TAG}"
                          Inc
                          STM32_HAL_${FAMILY}_PATH)
            else()
                git_clone_st(hal ${SUBMODULE_FOLDER} "${STM32_HAL_${FAMILY}_GIT_TAG}" Inc STM32_HAL_${FAMILY}_PATH)
            endif()
        endif()

        if(STM32_HAL_${FAMILY}_PATH AND ${VERBOSE})
            message("Submodule path: ${STM32_HAL_${FAMILY}_PATH}")
        endif()
    endif()

    #Checking HAL patch or release version
    unset(VERSION_INFO)
    find_file(PACKAGE_FILE NAMES package.xml PATHS ${STM32_CUBE_${FAMILY}_PATH})
    if(PACKAGE_FILE)
        file(READ ${PACKAGE_FILE} PACKAGE_FILE_CONTENT)
        string(REGEX MATCH
                     "PackDescription Release=\"FW.${FAMILY}.([0-9.]+)\"( Patch=\"FW.${FAMILY}.([0-9.]+)\")?"
                     VERSION_INFO
                     ${PACKAGE_FILE_CONTENT})
        if(CMAKE_MATCH_3) # This is the "Patch" revision
            set(HAL_${COMP}_VERSION ${CMAKE_MATCH_3})
            set(HAL_VERSION ${CMAKE_MATCH_3})
        else(CMAKE_MATCH_1) #This is the "Release" version
            set(HAL_${COMP}_VERSION ${CMAKE_MATCH_1})
            set(HAL_VERSION ${CMAKE_MATCH_1})
        endif()
    endif()
    if(NOT VERSION_INFO)
        message(STATUS "Could not read the HAL version from package.xml for ${COMP}")
    endif()

    find_path(HAL_${FAMILY}_PATH
              NAMES Inc/stm32${FAMILY_L}xx_hal.h
              PATHS "${STM32_HAL_${FAMILY}_PATH}" "${STM32_CUBE_${FAMILY}_PATH}/Drivers/STM32${FAMILY}xx_HAL_Driver"
              NO_DEFAULT_PATH)
    if(NOT HAL_${FAMILY}_PATH)
        continue()
    endif()

    find_path(HAL_${FAMILY}${CORE_U}_INCLUDE
              NAMES stm32${FAMILY_L}xx_hal.h
              PATHS "${HAL_${FAMILY}_PATH}/Inc"
              NO_DEFAULT_PATH)
    find_file(HAL_${FAMILY}${CORE_U}_SOURCE
              NAMES stm32${FAMILY_L}xx_hal.c
              PATHS "${HAL_${FAMILY}_PATH}/Src"
              NO_DEFAULT_PATH)

    if((NOT HAL_${FAMILY}${CORE_U}_INCLUDE) OR (NOT HAL_${FAMILY}${CORE_U}_SOURCE))
        set(HAL_${COMP}_FOUND FALSE)
        continue()
    endif()

    if(NOT (TARGET HAL::STM32::${FAMILY}${CORE_C}))
        add_library(HAL::STM32::${FAMILY}${CORE_C} INTERFACE IMPORTED)
        list(APPEND HAL_LIBRARIES "HAL::STM32::${FAMILY}${CORE_C}")
        target_link_libraries(HAL::STM32::${FAMILY}${CORE_C}
                              INTERFACE STM32::${FAMILY}${CORE_D} CMSIS::STM32::${FAMILY}${CORE_C})
        target_include_directories(HAL::STM32::${FAMILY}${CORE_C} INTERFACE "${HAL_${FAMILY}${CORE_U}_INCLUDE}")
        target_sources(HAL::STM32::${FAMILY}${CORE_C} INTERFACE "${HAL_${FAMILY}${CORE_U}_SOURCE}")
        target_compile_definitions(HAL::STM32::${FAMILY}${CORE_C} INTERFACE USE_HAL_DRIVER)
        target_compile_definitions(HAL::STM32::${FAMILY}${CORE_C} INTERFACE HAL_MODULE_ENABLED)
        target_compile_definitions(HAL::STM32::${FAMILY}${CORE_C} INTERFACE "__weak=__attribute__((weak))")
        target_compile_definitions(HAL::STM32::${FAMILY}${CORE_C} INTERFACE "__packed=__attribute__((__packed__))")
        target_include_directories(HAL::STM32::${FAMILY}${CORE_C} INTERFACE "${CMAKE_CURRENT_LIST_DIR}/misc/STM32_HAL") # for generic stm32_hal.h include
        if(${VERBOSE})
            message("Adding general library HAL::STM32::${FAMILY}${CORE_C}")
        endif()
    endif()

    foreach(DRV_COMP ${HAL_FIND_COMPONENTS_DRIVERS})
        string(TOLOWER ${DRV_COMP} DRV_L)
        string(TOUPPER ${DRV_COMP} DRV)

        if(NOT (DRV_L IN_LIST HAL_DRIVERS_${FAMILY}))
            continue()
        endif()

        find_file(HAL_${FAMILY}${CORE_U}_${DRV}_SOURCE
                  NAMES stm32${FAMILY_L}xx_hal_${DRV_L}.c
                  PATHS "${HAL_${FAMILY}_PATH}/Src"
                  NO_DEFAULT_PATH)
        list(APPEND HAL_${FAMILY}${CORE_U}_SOURCES "${HAL_${FAMILY}_${DRV}_SOURCE}")
        if(NOT HAL_${FAMILY}${CORE_U}_${DRV}_SOURCE)
            message(WARNING "Cannot find ${DRV} driver for ${FAMILY}${CORE_U}")
            set(HAL_${DRV_COMP}_FOUND FALSE)
            continue()
        endif()

        set(HAL_${DRV_COMP}_FOUND TRUE)
        if(HAL_${FAMILY}${CORE_U}_${DRV}_SOURCE AND (NOT (TARGET HAL::STM32::${FAMILY}::${DRV})))
            add_library(HAL::STM32::${FAMILY}${CORE_C}::${DRV} INTERFACE IMPORTED)
            list(APPEND HAL_LIBRARIES "HAL::STM32::${FAMILY}${CORE_C}::${DRV}")
            target_link_libraries(HAL::STM32::${FAMILY}${CORE_C}::${DRV}
                                  INTERFACE HAL::STM32::${FAMILY}${CORE_C})
            target_sources(HAL::STM32::${FAMILY}${CORE_C}::${DRV} INTERFACE "${HAL_${FAMILY}${CORE_U}_${DRV}_SOURCE}")
            target_compile_definitions(HAL::STM32::${FAMILY}${CORE_C}::${DRV} INTERFACE HAL_${DRV}_MODULE_ENABLED)
            if(${VERBOSE})
                message("Adding driver library HAL::STM32::${FAMILY}${CORE_C}::${DRV}")
            endif()
        endif()

        if(HAL_${FAMILY}${CORE_U}_${DRV}_SOURCE AND (${DRV_L} IN_LIST HAL_EX_DRIVERS_${FAMILY}))
            find_file(HAL_${FAMILY}${CORE_U}_${DRV}_EX_SOURCE
                      NAMES stm32${FAMILY_L}xx_hal_${DRV_L}_ex.c
                      PATHS "${HAL_${FAMILY}_PATH}/Src"
                      NO_DEFAULT_PATH)
            list(APPEND HAL_${FAMILY}${CORE_U}_SOURCES "${HAL_${FAMILY}${CORE_U}_${DRV}_EX_SOURCE}")
            if(NOT HAL_${FAMILY}${CORE_U}_${DRV}_EX_SOURCE)
                message(WARNING "Cannot find ${DRV}Ex driver for ${FAMILY}${CORE_U}")
            endif()

            if((TARGET HAL::STM32::${FAMILY}${CORE_C}::${DRV})
               AND (NOT (TARGET HAL::STM32::${FAMILY}${CORE_C}::${DRV}Ex)))
                add_library(HAL::STM32::${FAMILY}${CORE_C}::${DRV}Ex INTERFACE IMPORTED)
                list(APPEND HAL_LIBRARIES "HAL::STM32::${FAMILY}${CORE_C}::${DRV}Ex")
                target_link_libraries(HAL::STM32::${FAMILY}${CORE_C}::${DRV}Ex
                                      INTERFACE HAL::STM32::${FAMILY}${CORE_C}::${DRV})
                target_sources(HAL::STM32::${FAMILY}${CORE_C}::${DRV}Ex INTERFACE
                               "${HAL_${FAMILY}${CORE_U}_${DRV}_EX_SOURCE}")
                if(${VERBOSE})
                    message("Adding driver library HAL::STM32::${FAMILY}${CORE_C}::${DRV}Ex")
                endif()
            endif()
        endif()
    endforeach()

    foreach(DRV_COMP ${HAL_FIND_COMPONENTS_DRIVERS_LL})
        string(TOLOWER ${DRV_COMP} DRV_L)
        string(REGEX
               REPLACE "^ll_"
                       ""
                       DRV_L
                       ${DRV_L})
        string(TOUPPER ${DRV_L} DRV)

        if(NOT (DRV_L IN_LIST HAL_LL_DRIVERS_${FAMILY}))
            continue()
        endif()

        find_file(HAL_${FAMILY}${CORE_U}_${DRV}_LL_SOURCE
                  NAMES stm32${FAMILY_L}xx_ll_${DRV_L}.c
                  PATHS "${HAL_${FAMILY}_PATH}/Src"
                  NO_DEFAULT_PATH)
        list(APPEND HAL_${FAMILY}${CORE_U}_SOURCES "${HAL_${FAMILY}_${DRV}_LL_SOURCE}")
        if(NOT HAL_${FAMILY}${CORE_U}_${DRV}_LL_SOURCE)
            message(WARNING "Cannot find LL_${DRV} driver for ${FAMILY}${CORE_U}")
            set(HAL_${DRV_COMP}_FOUND FALSE)
            continue()
        endif()

        set(HAL_${DRV_COMP}_FOUND TRUE)
        if(HAL_${FAMILY}${CORE_U}_${DRV}_LL_SOURCE AND (NOT (TARGET HAL::STM32::${FAMILY}${CORE_C}::LL_${DRV})))
            add_library(HAL::STM32::${FAMILY}${CORE_C}::LL_${DRV} INTERFACE IMPORTED)
            list(APPEND HAL_LIBRARIES "HAL::STM32::${FAMILY}${CORE_C}::LL_${DRV}")
            target_include_directories(HAL::STM32::${FAMILY}${CORE_C}::LL_${DRV}
                                       INTERFACE "${HAL_${FAMILY}${CORE_U}_INCLUDE}")
            target_sources(HAL::STM32::${FAMILY}${CORE_C}::LL_${DRV} INTERFACE
                           "${HAL_${FAMILY}${CORE_U}_${DRV}_LL_SOURCE}")
            if(${VERBOSE})
                message("Adding low-level library HAL::STM32::${FAMILY}${CORE_C}::LL_${DRV}")
            endif()
        endif()
    endforeach()

    set(HAL_${COMP}_FOUND TRUE)
    list(APPEND HAL_INCLUDE_DIRS "${HAL_${FAMILY}${CORE_U}_INCLUDE}")
    list(APPEND HAL_SOURCES "${HAL_${FAMILY}${CORE_U}_SOURCES}")
endforeach()

list(REMOVE_DUPLICATES HAL_INCLUDE_DIRS)
list(REMOVE_DUPLICATES HAL_SOURCES)
list(REMOVE_DUPLICATES HAL_LIBRARIES)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(HAL
                                  REQUIRED_VARS
                                  HAL_INCLUDE_DIRS
                                  HAL_SOURCES
                                  HAL_LIBRARIES
                                  FOUND_VAR
                                  HAL_FOUND
                                  HANDLE_COMPONENTS)
