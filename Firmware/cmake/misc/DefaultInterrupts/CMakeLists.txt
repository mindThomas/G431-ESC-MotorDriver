add_library(DefaultInterrupts INTERFACE)
target_sources(DefaultInterrupts INTERFACE DefaultInterrupts.c)
get_filename_component(INCLUDE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
target_include_directories(DefaultInterrupts INTERFACE ${INCLUDE_PATH})

# Defines LSPC_USE_DEBUG when option is enabled
add_library_option(DefaultInterrupts SYSTICK)