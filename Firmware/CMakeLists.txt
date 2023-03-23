cmake_minimum_required(VERSION 3.13)

include(STM32-libraries/versions.cmake) # Defines HAL, CMSIS and FreeRTOS versions (Git tags) to use
include(cmake/stm32.cmake)

project(G431-ESC-MotorDriver C CXX ASM)
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

# The Fetch feature can also be used, but that will fetch the whole repository of the components
# instead of just the needed tag
#stm32_fetch_cmsis(G4)
#stm32_fetch_hal(G4)

find_package(CMSIS
             COMPONENTS STM32G431CB
             REQUIRED)
find_package(HAL
             COMPONENTS STM32G431CB
             REQUIRED)
find_package(FreeRTOS
             COMPONENTS ARM_CM4F
             REQUIRED)

# Add STM32-libraries
add_subdirectory(STM32-libraries stm32-libraries) # has to come after find_package

# Add common directory (includes MessageTypes etc.)
include_directories(../Common)
add_subdirectory(../Common "Common")

add_executable(firmware
               Src/main.c
               #Src/syscalls.c
               #Src/freertos.c
               Src/ProcessorInit.c
               #Src/stm32g4xx_it.c
               #Src/TestTask.cpp
               #Src/heap_useNewlib.c # Src/MemoryManagement.c
               Src/MainTask.cpp
               #Src/FreeRTOS-openocd.c
               )
include_directories(Inc)
target_link_libraries(firmware PRIVATE
                      # Processor dependencies
                      CMSIS::STM32::G431CB # generates and adds the linker file and startup script
                      STM32::Nano # NoSys or Nano specs
                      # HAL libraries for CPU configuration (defines USE_HAL_DRIVER)
                      HAL::STM32::G4::RCC
                      HAL::STM32::G4::RCCEx
                      HAL::STM32::G4::PWR
                      HAL::STM32::G4::PWREx
                      HAL::STM32::G4::TIM
                      HAL::STM32::G4::CORTEX
                      # FreeRTOS libraries
                      FreeRTOS::ARM_CM4F # defines USE_FREERTOS
                      FreeRTOS::Timers
                      #FreeRTOS::Heap::4
                      FreeRTOS::Heap::Newlib
                      #FreeRTOS::CMSIS # defines USE_FREERTOS_CMSIS
                      FreeRTOS::OpenOCD
                      #FreeRTOS::MallocOverload
                      # Periphiral libraries
                      STM32G4_CANBus
                      STM32G4_Encoder
                      #STM32G4_InputCapture
                      #STM32G4_IO STM32G4_PWM
                      STM32G4_SyncedPWMADC
                      STM32G4_Timer
                      STM32G4_UART
                      STM32G4_PrecisionSysTick
                      STM32G4_EEPROM
                      # Other libraries
                      MATLABCoderInit
        Debug
        LSPC
        Matrix
        LeastSquares
        FirstOrderLPF
        CPULoad
        Syscalls
        #Sysmem
        DefaultInterrupts
        )

stm32_add_linker_script(firmware Src/G431CB_WithEEProm.ld) # override linker file
stm32_add_startup_script(firmware Src/startup_stm32g431xx.s) # override default startup file

target_compile_definitions(firmware PUBLIC FREERTOS_USE_NEWLIB)

# Enable Floating point support in the Newlib Nano libary (STM32::Nano) - takes up roughly 8 KB of FLASH
enable_option(firmware STM32::Nano FLOAT)

# Override CMSIS RTOS library (since the CMSIS RTOS library has been changed to support 10 KHz FreeRTOS tick rate)
target_include_directories(firmware PUBLIC Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS)
target_sources(firmware PUBLIC Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c)
target_compile_definitions(firmware PUBLIC USE_FREERTOS_CMSIS)

target_compile_definitions(firmware PUBLIC HSE_VALUE=8000000)
set_configurable_option(firmware STM32G4_PrecisionSysTick FREQUENCY 100000) # OBS! Precision systick is used for task timing in CPU Load, so it should be equal to or faster than the FreeRTOS tick rate of 10 KHz

target_compile_options(firmware
                       PUBLIC "-Wdouble-promotion"
                              "-Wfloat-conversion"
                              "-fsingle-precision-constant")

target_compile_options(firmware PUBLIC "-Wstack-usage=256") # Warning at stack usage above 256 bytes

#include(print_target_properties)
#print_target_properties(firmware)

enable_option(firmware LSPC  UART)
set_configurable_option(firmware LSPC TX_QUEUE_LENGTH 50)
enable_option(firmware Debug LSPC)
enable_option(firmware Debug PRINTF)
#enable_option(firmware CPULoad VERBOSE)
#enable_option(firmware STM32G4_IO DEBUG)
enable_option(firmware STM32G4_SyncedPWMADC DEBUG)

#get_linked_libraries(OUT firmware)
#message("Linked libraries = ${OUT}")

add_custom_target(
        print_openocd_firmware
        COMMAND ${CMAKE_COMMAND} -E echo
        '$<$<BOOL:$<FILTER:$<TARGET_PROPERTY:firmware,LINK_LIBRARIES>,INCLUDE,FreeRTOS>>:FreeRTOS>')
        #'$<$<IN_LIST:FreeRTOS::ARM_CM4F,$<TARGET_PROPERTY:firmware,LINK_LIBRARIES>>:FreeRTOS>')

# Always include debug symbols
#add_compile_options(-g)
#add_compile_options(-ggdb -flto)

target_link_options(firmware PUBLIC "-Wl,--wrap=malloc")
target_link_options(firmware PUBLIC "-Wl,--wrap=_malloc_r")
target_link_options(firmware PUBLIC "-Wl,--wrap=free")
target_link_options(firmware PUBLIC "-Wl,--wrap=_free_r")

#target_compile_definitions(firmware PUBLIC "new=new(__FILE__, __LINE__)")