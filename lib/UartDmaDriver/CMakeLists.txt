# UartDmaDriver library
add_library(UartDmaDriver STATIC
    src/UartDmaDriver.cpp
)

# Public include directories
target_include_directories(UartDmaDriver PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

# C++ standard
target_compile_features(UartDmaDriver PUBLIC
    cxx_std_17
)

# Link required Pico SDK libraries
target_link_libraries(UartDmaDriver PUBLIC
    pico_stdlib
    hardware_uart
    hardware_dma
    hardware_gpio
    hardware_irq
    freertos
)

# Optional configuration overrides
if(DEFINED UART_RX_MAX_BAUDRATE)
    target_compile_definitions(UartDmaDriver PUBLIC UART_RX_MAX_BAUDRATE=${UART_RX_MAX_BAUDRATE})
endif()

if(DEFINED UART_DMA_RING_BITS)
    target_compile_definitions(UartDmaDriver PUBLIC UART_DMA_RING_BITS=${UART_DMA_RING_BITS})
endif()

if(DEFINED UART_RX_MAX_EVT_DATA_SIZE)
    target_compile_definitions(UartDmaDriver PUBLIC UART_RX_MAX_EVT_DATA_SIZE=${UART_RX_MAX_EVT_DATA_SIZE})
endif()

if(DEFINED UART_EVENT_Q_SIZE)
    target_compile_definitions(UartDmaDriver PUBLIC UART_EVENT_Q_SIZE=${UART_EVENT_Q_SIZE})
endif()

if(DEFINED UART_WD_TICK_US)
    target_compile_definitions(UartDmaDriver PUBLIC UART_WD_TICK_US=${UART_WD_TICK_US})
endif()

if(DEFINED UART_WD_SILENCE_TICKS)
    target_compile_definitions(UartDmaDriver PUBLIC UART_WD_SILENCE_TICKS=${UART_WD_SILENCE_TICKS})
endif()

if(DEFINED UART_TX_BUFFER_SIZE)
    target_compile_definitions(UartDmaDriver PUBLIC UART_TX_BUFFER_SIZE=${UART_TX_BUFFER_SIZE})
endif()

if(DEFINED UART_DMA_DRIVER_NO_INSTALL_IRQ)
    target_compile_definitions(UartDmaDriver PUBLIC UART_DMA_DRIVER_NO_INSTALL_IRQ=${UART_DMA_DRIVER_NO_INSTALL_IRQ})
endif()