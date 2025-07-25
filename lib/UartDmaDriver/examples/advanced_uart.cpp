/**
 * Advanced UART Example
 * 
 * This example shows advanced event handling using FreeRTOS QueueSet:
 * - Multiple event sources in a single efficient loop
 * - UART events + inter-task communication queues
 * - Protocol bridge (ASCII Modbus ↔ RTU) as real-world example
 * - "Superloop" pattern but with zero CPU waste
 * 
 * Features demonstrated:
 * - QueueSet for multi-source event handling
 * - Cross-UART communication pattern
 * - Protocol translation (ASCII ↔ RTU)
 * - Frame completion detection using silenceFlag
 * 
 * Hardware: RP2040/RP2350 with 2 UART interfaces
 * Use case: Efficient event-driven gateway architecture
 */

#include "UartDmaDriver.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "pico/stdlib.h"
#include <cstdio>
#include <algorithm>

// UART0 configuration - ASCII Modbus (PC/Terminal side)
constexpr uart_inst_t* ASCII_UART = uart0;
constexpr uint8_t ASCII_TX_PIN = 0;   // Change to your TX pin
constexpr uint8_t ASCII_RX_PIN = 1;   // Change to your RX pin
constexpr uint32_t ASCII_BAUD = 115200;

// UART1 configuration - Modbus RTU (Field devices side)
constexpr uart_inst_t* RTU_UART = uart1;
constexpr uint8_t RTU_TX_PIN = 4;     // Change to your TX pin
constexpr uint8_t RTU_RX_PIN = 5;     // Change to your RX pin
constexpr uint32_t RTU_BAUD = 9600;

// Create driver instances
UartDmaDriver ascii_uart(ASCII_UART, ASCII_TX_PIN, ASCII_RX_PIN, ASCII_BAUD);
UartDmaDriver rtu_uart(RTU_UART, RTU_TX_PIN, RTU_RX_PIN, RTU_BAUD);

// Inter-task communication
QueueHandle_t ascii_to_rtu_queue;
QueueHandle_t rtu_to_ascii_queue;

struct BridgeMessage {
    uint8_t data[256];
    size_t length;
};

// Protocol conversion functions (implementation details omitted for clarity)
size_t convert_ascii_to_rtu(const uint8_t* ascii_cmd, size_t ascii_len, uint8_t* rtu_frame) {
    // Example: "01 03 0000 0002" (ASCII) → [01 03 00 00 00 02 C4 0B] (RTU with CRC)
    // Implementation would parse ASCII hex, add CRC, return RTU frame length
    // For this example, we'll just copy and add dummy CRC
    size_t rtu_len = ascii_len / 3;  // Simplified conversion
    for (size_t i = 0; i < rtu_len; i++) {
        rtu_frame[i] = ascii_cmd[i * 3];  // Extract hex bytes (simplified)
    }
    return rtu_len;
}

size_t convert_rtu_to_ascii(const uint8_t* rtu_frame, size_t rtu_len, uint8_t* ascii_resp) {
    // Example: [01 03 04 00 0A 00 0B B8 FA] (RTU) → "01 03 04 000A 000B" (ASCII)
    // Implementation would format as hex string, validate CRC
    size_t ascii_len = 0;
    for (size_t i = 0; i < rtu_len - 2; i++) {  // Skip CRC for demo
        ascii_len += snprintf((char*)ascii_resp + ascii_len, 32, "%02X ", rtu_frame[i]);
    }
    return ascii_len;
}

void gateway_task(void* param) {
    // Initialize both UARTs
    if (ascii_uart.init() != UartDmaDriver::SUCCESS || 
        ascii_uart.start() != UartDmaDriver::SUCCESS) {
        printf("ERROR: ASCII UART initialization failed\n");
        vTaskDelete(NULL);
    }
    
    if (rtu_uart.init() != UartDmaDriver::SUCCESS || 
        rtu_uart.start() != UartDmaDriver::SUCCESS) {
        printf("ERROR: RTU UART initialization failed\n");
        vTaskDelete(NULL);
    }
    
    printf("ASCII Bridge ready on pins TX=%d, RX=%d at %lu baud\n", 
           ASCII_TX_PIN, ASCII_RX_PIN, ASCII_BAUD);
    printf("RTU Bridge ready on pins TX=%d, RX=%d at %lu baud\n", 
           RTU_TX_PIN, RTU_RX_PIN, RTU_BAUD);
    
    // Create QueueSet for efficient multi-source event handling
    QueueSetHandle_t queueSet = xQueueCreateSet(20);
    if (!queueSet) {
        printf("ERROR: Failed to create QueueSet\n");
        vTaskDelete(NULL);
    }
    
    // Add all event sources to the queue set
    xQueueAddToSet(ascii_uart.getEventQueueHandle(), queueSet);
    xQueueAddToSet(rtu_uart.getEventQueueHandle(), queueSet);
    xQueueAddToSet(ascii_to_rtu_queue, queueSet);
    xQueueAddToSet(rtu_to_ascii_queue, queueSet);
    
    UartDmaDriver::Event event;
    uint8_t buffer[256];
    BridgeMessage bridge_msg;
    
    printf("Gateway running - efficient superloop with QueueSet\n");
    
    while (true) {
        // Wait for ANY event source - yields CPU until something happens
        QueueSetMemberHandle_t activeQueue = xQueueSelectFromSet(queueSet, portMAX_DELAY);
        
        if (activeQueue == ascii_uart.getEventQueueHandle()) {
            // ASCII UART has data
            if (xQueueReceive(ascii_uart.getEventQueueHandle(), &event, 0) == pdPASS) {
                if (event.type == UartDmaDriver::EVT_DATA) {
                    size_t to_read = std::min(event.size, sizeof(buffer));
                    size_t bytes_read = ascii_uart.read(buffer, to_read);
                    
                    if (bytes_read > 0) {
                        printf("[BRIDGE] ASCII → RTU: %zu bytes\n", bytes_read);
                        
                        // Convert ASCII command to RTU frame
                        bridge_msg.length = convert_ascii_to_rtu(buffer, bytes_read, bridge_msg.data);
                        
                        // Send to RTU side
                        xQueueSend(ascii_to_rtu_queue, &bridge_msg, 0);
                    }
                }
            }
        }
        else if (activeQueue == rtu_uart.getEventQueueHandle()) {
            // RTU UART has data
            if (xQueueReceive(rtu_uart.getEventQueueHandle(), &event, 0) == pdPASS) {
                if (event.type == UartDmaDriver::EVT_DATA && event.silenceFlag) {
                    size_t to_read = std::min(event.size, sizeof(buffer));
                    size_t bytes_read = rtu_uart.read(buffer, to_read);
                    
                    if (bytes_read > 0) {
                        printf("[BRIDGE] RTU RX: %zu bytes (frame complete)\n", bytes_read);
                        
                        // Convert RTU response to ASCII format
                        bridge_msg.length = convert_rtu_to_ascii(buffer, bytes_read, bridge_msg.data);
                        
                        // Send to ASCII side
                        xQueueSend(rtu_to_ascii_queue, &bridge_msg, 0);
                    }
                }
            }
        }
        else if (activeQueue == ascii_to_rtu_queue) {
            // Command from ASCII side to transmit on RTU
            if (xQueueReceive(ascii_to_rtu_queue, &bridge_msg, 0) == pdPASS) {
                rtu_uart.send(bridge_msg.data, bridge_msg.length);
                printf("[BRIDGE] RTU TX: %zu bytes\n", bridge_msg.length);
            }
        }
        else if (activeQueue == rtu_to_ascii_queue) {
            // Response from RTU side to transmit on ASCII
            if (xQueueReceive(rtu_to_ascii_queue, &bridge_msg, 0) == pdPASS) {
                ascii_uart.send(bridge_msg.data, bridge_msg.length);
                printf("[BRIDGE] ASCII ← RTU: %zu bytes\n", bridge_msg.length);
            }
        }
        
        // Loop back immediately - QueueSet ensures we only wake up when needed
    }
}

int main() {
    stdio_init_all();
    printf("Advanced UART Example - QueueSet Gateway Starting...\n");
    
    // Create inter-task communication queues
    ascii_to_rtu_queue = xQueueCreate(5, sizeof(BridgeMessage));
    rtu_to_ascii_queue = xQueueCreate(5, sizeof(BridgeMessage));
    
    if (!ascii_to_rtu_queue || !rtu_to_ascii_queue) {
        printf("ERROR: Failed to create communication queues\n");
        return -1;
    }
    
    // Create single gateway task using QueueSet
    xTaskCreate(gateway_task, "Gateway", 3072, NULL, 1, NULL);
    
    vTaskStartScheduler();
    
    printf("FATAL: Scheduler returned\n");
    while (1) {
        sleep_ms(1000);
    }
}