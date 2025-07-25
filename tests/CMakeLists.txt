# CH9120Driver Test Suite CMake Configuration

# Only initialize if we're being built directly (not via parent CMakeLists.txt)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    # This is a direct standalone build of tests only
    cmake_minimum_required(VERSION 3.13)
    
    if(NOT DEFINED PICO_SDK_PATH AND NOT DEFINED ENV{PICO_SDK_PATH})
        message(FATAL_ERROR "PICO_SDK_PATH not defined. Set PICO_SDK_PATH environment variable or pass -DPICO_SDK_PATH to cmake")
    endif()

    set(CMAKE_C_STANDARD 11)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
    
    # Board selection
    set(PICO_BOARD pico2 CACHE STRING "Board type")
    
    include(pico_sdk_import.cmake)
    project(CH9120_Tests_Standalone C CXX ASM)
    pico_sdk_init()
    
    message(STATUS "Building CH9120Driver tests in DIRECT STANDALONE mode")
    
    # Add parent directory for CH9120Driver (but exclude its tests to avoid recursion)
    add_subdirectory(.. CH9120Driver EXCLUDE_FROM_ALL)
    
    # Add FreeRTOS
    add_subdirectory(freertos)
    
else()
    # We're being built as part of parent CH9120Driver or main project
    message(STATUS "Building CH9120Driver tests as SUBDIRECTORY")
    
    # Add FreeRTOS if not already added
    if(NOT TARGET freertos)
        add_subdirectory(freertos)
    endif()
endif()

# Common settings for all test executables
function(configure_test_executable target_name)
    # Link libraries
    target_link_libraries(${target_name}
        pico_stdlib
        pico_multicore
        freertos
        CH9120Driver
        hardware_uart
        hardware_gpio
        hardware_dma
        hardware_irq
    )
    
    # Enable USB serial output
    pico_enable_stdio_usb(${target_name} 1)
    pico_enable_stdio_uart(${target_name} 0)
    
    # Create UF2 output
    pico_add_extra_outputs(${target_name})
    
    # Set binary info
    pico_set_program_name(${target_name} "CH9120Driver ${target_name}")
    pico_set_program_description(${target_name} "CH9120Driver Test Suite - ${target_name}")
    pico_set_program_version(${target_name} "1.0")
    pico_set_program_url(${target_name} "https://github.com/pierrejay/pico-freertos-CH9120Driver")
endfunction()

# ===================================================================================
# TEST EXECUTABLES
# ===================================================================================

# Client Tests
add_executable(test_client
    test_client_main.cpp
)
configure_test_executable(test_client)

# Server Tests  
add_executable(test_server
    test_server_main.cpp
)
configure_test_executable(test_server)

# ===================================================================================
# CUSTOM TARGETS FOR TEST MANAGEMENT
# ===================================================================================

# Flash client test to Pico
add_custom_target(flash_client_test
    COMMAND ${CMAKE_COMMAND} -E echo "Flashing client test to Pico..."
    COMMAND picotool load test_client.uf2 -f
    DEPENDS test_client
    COMMENT "Flash client test firmware to connected Pico"
)

# Flash server test to Pico
add_custom_target(flash_server_test
    COMMAND ${CMAKE_COMMAND} -E echo "Flashing server test to Pico..."
    COMMAND picotool load test_server.uf2 -f
    DEPENDS test_server
    COMMENT "Flash server test firmware to connected Pico"
)

# Start mock Python servers for testing
add_custom_target(start_mock_servers
    COMMAND ${CMAKE_COMMAND} -E echo "Starting mock servers..."
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/mock_server.py --mode dual &
    COMMENT "Start Python mock servers on ports 8000 and 8001"
)

# Quick test setup - builds and provides instructions
add_custom_target(setup_tests
    COMMAND ${CMAKE_COMMAND} -E echo "===========================================" 
    COMMAND ${CMAKE_COMMAND} -E echo "CH9120Driver Test Setup Complete!"
    COMMAND ${CMAKE_COMMAND} -E echo "==========================================="
    COMMAND ${CMAKE_COMMAND} -E echo ""
    COMMAND ${CMAKE_COMMAND} -E echo "Test binaries created:"
    COMMAND ${CMAKE_COMMAND} -E echo "  - test_client.uf2 (Client test suite)"
    COMMAND ${CMAKE_COMMAND} -E echo "  - test_server.uf2 (Server test suite)"
    COMMAND ${CMAKE_COMMAND} -E echo ""
    COMMAND ${CMAKE_COMMAND} -E echo "Next steps:"
    COMMAND ${CMAKE_COMMAND} -E echo "1. Flash test firmware: make flash_client_test"
    COMMAND ${CMAKE_COMMAND} -E echo "2. Start mock server: python3 tests/mock_server.py"
    COMMAND ${CMAKE_COMMAND} -E echo "3. Connect to Pico serial console to see results"
    COMMAND ${CMAKE_COMMAND} -E echo ""
    COMMAND ${CMAKE_COMMAND} -E echo "For detailed instructions, see: tests/README.md"
    DEPENDS test_client test_server
    COMMENT "Setup complete - ready for testing"
)

# Clean test outputs
add_custom_target(clean_tests
    COMMAND ${CMAKE_COMMAND} -E remove -f test_client.uf2
    COMMAND ${CMAKE_COMMAND} -E remove -f test_server.uf2
    COMMAND ${CMAKE_COMMAND} -E remove -f test_client.elf
    COMMAND ${CMAKE_COMMAND} -E remove -f test_server.elf
    COMMENT "Clean test build outputs"
)

# ===================================================================================
# DEVELOPMENT HELPERS
# ===================================================================================

# Quick build and size report
add_custom_target(test_sizes
    COMMAND ${CMAKE_COMMAND} -E echo "Test binary sizes:"
    COMMAND size test_client.elf
    COMMAND size test_server.elf
    DEPENDS test_client test_server
    COMMENT "Show test binary sizes"
)

# Validation target - ensures everything builds
add_custom_target(validate_tests
    COMMAND ${CMAKE_COMMAND} -E echo "✓ Client test builds successfully"
    COMMAND ${CMAKE_COMMAND} -E echo "✓ Server test builds successfully"
    COMMAND ${CMAKE_COMMAND} -E echo "✓ All test dependencies satisfied"
    COMMAND ${CMAKE_COMMAND} -E echo ""
    COMMAND ${CMAKE_COMMAND} -E echo "Test suite is ready for deployment!"
    DEPENDS test_client test_server
    COMMENT "Validate that all tests build correctly"
)