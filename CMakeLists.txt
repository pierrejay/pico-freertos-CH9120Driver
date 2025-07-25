cmake_minimum_required(VERSION 3.13)

# Check if we're being built as a standalone project or as a subdirectory
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    # Standalone build - initialize everything
    
    set(CMAKE_C_STANDARD 11)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

    # Board selection
    set(PICO_BOARD pico2 CACHE STRING "Board type")

    # Pull in Raspberry Pi Pico SDK (must be before project)
    include(pico_sdk_import.cmake)

    project(CH9120Driver_Standalone C CXX ASM)

    # Initialise the Raspberry Pi Pico SDK
    pico_sdk_init()

    message(STATUS "Building CH9120Driver in STANDALONE mode")
    message(STATUS "Board: ${PICO_BOARD}")
    
    # Add tests when building standalone
    add_subdirectory(tests)
    
else()
    # Subdirectory build - just define the library
    message(STATUS "Building CH9120Driver as SUBDIRECTORY")
endif()

# Add UartDmaDriver dependency
add_subdirectory(lib/UartDmaDriver)

# CH9120Driver library (always built)
add_library(CH9120Driver STATIC
    src/CH9120Driver.cpp
)

# Public include directories
target_include_directories(CH9120Driver PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

# C++ standard
target_compile_features(CH9120Driver PUBLIC
    cxx_std_17
)

# Link required Pico SDK libraries
target_link_libraries(CH9120Driver PUBLIC
    pico_stdlib
    hardware_uart
    hardware_dma
    hardware_gpio
    hardware_irq
    freertos
    UartDmaDriver
)