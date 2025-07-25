/**
 * Basic UART Example
 * 
 * This example shows the minimal code required to:
 * - Initialize and start the UART driver
 * - Process incoming data events
 * - Handle basic error conditions
 * 
 * Hardware: Any RP2040/RP2350 board
 * Connections: Connect a USB-serial adapter or terminal to GPIO pins
 */

#include "UartDmaDriver.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include <cstdio>
#include <algorithm>

// UART configuration - adapt these for your hardware
constexpr uart_inst_t* UART_INSTANCE = uart0;
constexpr uint8_t UART_TX_PIN = 0;  // Change to your TX pin
constexpr uint8_t UART_RX_PIN = 1;  // Change to your RX pin
constexpr uint32_t BAUD_RATE = 115200;

// Create driver instance
UartDmaDriver uart_driver(UART_INSTANCE, UART_TX_PIN, UART_RX_PIN, BAUD_RATE);

void uart_processing_task(void* param) {
    // Initialize the driver
    if (uart_driver.init() != UartDmaDriver::SUCCESS) {
        printf("ERROR: UART driver initialization failed\n");
        vTaskDelete(NULL);
        return;
    }
    
    // Start UART operations
    if (uart_driver.start() != UartDmaDriver::SUCCESS) {
        printf("ERROR: UART driver start failed\n");
        vTaskDelete(NULL);
        return;
    }
    
    printf("UART driver started successfully on pins TX=%d, RX=%d\n", 
           UART_TX_PIN, UART_RX_PIN);
    
    // Event and data buffers
    UartDmaDriver::Event event;
    uint8_t buffer[256];  // Match max event data size
    
    while (true) {
        // Wait for next event - yields CPU until data arrives
        if (!uart_driver.popEvent(&event, portMAX_DELAY)) {
            continue; // Timeout (shouldn't happen with portMAX_DELAY)
        }
        
        switch (event.type) {
            case UartDmaDriver::EVT_DATA: { // Data is available - read it from internal buffer
                size_t to_read = std::min(event.size, sizeof(buffer));
                size_t bytes_read = uart_driver.read(buffer, to_read);
                if (bytes_read > 0) {
                    printf("Received %zu bytes", bytes_read);
                    if (event.silenceFlag) {
                        printf(" (frame end detected)");
                    }
                    printf("\n");
                    
                    // Process your data here...
                    // For example, echo it back:
                    // uart_driver.send(buffer, bytes_read);
                }
                break;
            }
            
            case UartDmaDriver::EVT_OVERFLOW: { // Handle buffer overflow: drain it to recover
                printf("WARNING: UART buffer overflow - starting recovery\n");
                
                // Step 1: Drain all pending events from the queue
                UartDmaDriver::Event drain_event;
                while (uart_driver.popEvent(&drain_event, 0)) {
                    // Discard overflow and old data events
                    if (drain_event.type == UartDmaDriver::EVT_DATA) {
                        uart_driver.read(buffer, sizeof(buffer));
                    }
                }
                
                // Step 2: Drain the ring buffer until it's empty
                size_t total_discarded = 0;
                size_t bytes_read;
                do {
                    bytes_read = uart_driver.read(buffer, sizeof(buffer));
                    total_discarded += bytes_read;
                } while (bytes_read == sizeof(buffer)); // Continue until we read the last chunk of data
                
                printf("Recovery complete: discarded %zu bytes\n", total_discarded);
                printf("Consider reading data faster or reducing baud rate\n");
                break;
            }
        }

        // No need to yield here - FreeRTOS will handle task switching when looping back to `popEvent()`
    }
}

int main() {
    // Initialize stdio for debug output
    stdio_init_all();
    
    printf("Basic UART Example Starting...\n");
    
    // Create the UART processing task
    xTaskCreate(uart_processing_task, "UART", 2048, NULL, 1, NULL);
    
    // Start FreeRTOS scheduler
    vTaskStartScheduler();
    
    // Should never reach here
    printf("FATAL: Scheduler returned\n");
    while (1) {
        sleep_ms(1000);
    }
}