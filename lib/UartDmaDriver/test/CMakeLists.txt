cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(uart_dma_tests C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add FreeRTOS (from test subdirectory)
add_subdirectory(freertos)

# Add UartDmaDriver (parent directory contains the lib)
add_subdirectory(.. UartDmaDriver)

# Add test executable
add_executable(uart_tests test_main.cpp)

pico_set_program_name(uart_tests "UartDmaDriver Tests")
pico_set_program_version(uart_tests "1.0")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(uart_tests 0)
pico_enable_stdio_usb(uart_tests 1)

# Add the standard library to the build
target_link_libraries(uart_tests
    pico_stdlib
    freertos
    UartDmaDriver
)

# Add the standard include files to the build
target_include_directories(uart_tests PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(uart_tests)