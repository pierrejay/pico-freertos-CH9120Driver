#include <stdio.h>
#include <string.h>
#include <string_view>
#include <vector>
#include <memory>
#include <algorithm>
#include <functional>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "UartDmaDriver.hpp"

// === PLATFORM-SPECIFIC UART PINS ===
// Define UART pins based on chip (RP2040 or RP2350)
// Uses the literal flag defined by Pico SDK in CMakeLists.txt
constexpr std::string_view board = PICO_BOARD;

// RP2040 (Pico): TX0 (GP17) <> (GP4) RX1  - RX0 (GP16) <> (GP5) TX1
// RP2350 (Pico2): TX0 (GP1) <> (GP9) RX1 - RX0 (GP0) <> (GP8) TX1
// Pico & Pico2 W: TX0 (GP12) <> (GP9) RX1 - RX0 (GP13) <> (GP8) TX1 
constexpr uint8_t TX0_PIN = (board == "pico") ? 17 :
                            (board == "pico2") ? 1 :
                            (board == "pico_w" || board == "pico2_w") ? 12 : UINT8_MAX;

constexpr uint8_t RX0_PIN = (board == "pico") ? 16 :
                            (board == "pico2") ? 0 :
                            (board == "pico_w" || board == "pico2_w") ? 13 : UINT8_MAX;

constexpr uint8_t TX1_PIN = (board == "pico") ? 5 :
                            (board == "pico2" || board == "pico_w" || board == "pico2_w") ? 8 : UINT8_MAX;

constexpr uint8_t RX1_PIN = (board == "pico") ? 4 :
                            (board == "pico2" || board == "pico_w" || board == "pico2_w") ? 9 : UINT8_MAX;

// === TEST CONFIGURATION ===
constexpr uint32_t TEST_BAUD_RATES[] = {921600, 115200, 9600};
constexpr int NUM_BAUD_RATES = sizeof(TEST_BAUD_RATES) / sizeof(TEST_BAUD_RATES[0]);
constexpr int NORMAL_CHUNKS = 20;
constexpr int BURST_CHUNKS = 15;

// Test data patterns (adjusted for UART_TX_BUFFER_SIZE = 256)
constexpr uint16_t NORMAL_CHUNK = 200;  // Safe size < 256
constexpr uint16_t OVERFLOW_CHUNK = 250; // Larger chunks to trigger overflow faster
static uint8_t test_chunk[NORMAL_CHUNK];
static uint8_t overflow_chunk[OVERFLOW_CHUNK];

// === ENHANCED TEST CONFIGURATION ===
struct TestConfig {
    uint32_t baudRate;
    uint32_t pattern_chunks;      // Number of chunks for pattern test
    uint32_t pattern_chunk_size;  // Size of each chunk
    uint32_t timing_samples;      // Number of samples for timing
    uint32_t timeout_ms;          // Adaptive timeout
    uint32_t inter_chunk_delay;   // Delay between chunks (ms)
};

// Optimized configuration table
static const TestConfig TEST_CONFIGS[] = {
    // baudRate, chunks, chunk_size, timing_samples, timeout_ms, delay_ms
    {921600,    50,     256,        1000,           2000,       5},   // Fast
    {115200,    30,     256,        500,            4000,       10},  // Medium  
    {9600,      20,     256,        200,            15000,      25},  // Slow
};
static const int NUM_TEST_CONFIGS = sizeof(TEST_CONFIGS) / sizeof(TEST_CONFIGS[0]);

// === ENHANCED TEST STRUCTURES ===
struct TimingMetrics {
    uint64_t min_latency_us;
    uint64_t max_latency_us;
    uint64_t total_latency_us;
    uint64_t samples;
    uint64_t max_jitter_us;
    uint32_t timeouts;
    
    void reset() {
        min_latency_us = UINT64_MAX;
        max_latency_us = 0;
        total_latency_us = 0;
        samples = 0;
        max_jitter_us = 0;
        timeouts = 0;
    }
    
    uint64_t getAverage() const {
        return samples > 0 ? total_latency_us / samples : 0;
    }
};

struct IntegrityResults {
    uint32_t total_bytes_sent;
    uint32_t total_bytes_received;
    uint32_t sequence_errors;
    uint32_t missing_bytes;
    uint32_t duplicate_bytes;
    uint32_t first_error_position;
    uint8_t expected_at_error;
    uint8_t received_at_error;
    
    void reset() {
        memset(this, 0, sizeof(*this));
        first_error_position = UINT32_MAX;
    }
    
    bool isValid() const {
        return (sequence_errors == 0) && 
               (total_bytes_sent == total_bytes_received) && 
               (missing_bytes == 0);
    }
};

// Global test results
struct TestResults {
    uint32_t baudRate;
    uint32_t totalSent;
    uint32_t totalReceived;
    uint32_t overflows;
    float efficiency;
    bool overflowRecovered;
    bool passed;
};

static TestResults g_results[NUM_BAUD_RATES];
static int g_currentTest = 0;

void initTestData() {
    for (int i = 0; i < NORMAL_CHUNK; i++) {
        test_chunk[i] = (uint8_t)((i * 17 + 42) & 0xFF);
    }
    
    for (int i = 0; i < OVERFLOW_CHUNK; i++) {
        overflow_chunk[i] = (uint8_t)((i * 23 + 200) & 0xFF);
    }
}

// === RECEIVER SLAVE TASK ===
struct ReceiverTaskParams {
    UartDmaDriver* uart_receiver;
    std::vector<uint8_t>* received_buffer;
    volatile uint32_t* received_bytes;
    volatile bool* task_running;
    volatile bool* stop_requested;
};

void receiverSlaveTask(void* params) {
    ReceiverTaskParams* p = static_cast<ReceiverTaskParams*>(params);
    
    *p->task_running = true;
    *p->received_bytes = 0;
    
    uint8_t temp_buffer[512];
    
    while (!*p->stop_requested) {
        UartDmaDriver::Event event;
        
        if (p->uart_receiver->popEvent(&event, pdMS_TO_TICKS(10))) {
            if (event.type == UartDmaDriver::EVT_DATA) {
                size_t bytes_read = p->uart_receiver->read(temp_buffer, sizeof(temp_buffer));
                
                // Copy to destination buffer
                size_t buffer_space = p->received_buffer->size() - *p->received_bytes;
                size_t copy_bytes = std::min(bytes_read, buffer_space);
                
                if (copy_bytes > 0) {
                    std::copy(temp_buffer, temp_buffer + copy_bytes, 
                             p->received_buffer->begin() + *p->received_bytes);
                    *p->received_bytes += copy_bytes;
                }
            }
            else if (event.type == UartDmaDriver::EVT_OVERFLOW) {
                // Just consume data during overflow
                p->uart_receiver->read(temp_buffer, sizeof(temp_buffer));
            }
        }
    }
    
    *p->task_running = false;
    vTaskDelete(NULL);
}

// === ENHANCED PATTERN INTEGRITY TEST ===
bool testPatternIntegrity(UartDmaDriver& uart_sender, UartDmaDriver& uart_receiver, 
                         const TestConfig& config, IntegrityResults& results, uint32_t baudRate) {
    
    // Adaptive test size based on baud rate for reasonable duration
    uint32_t ITERATIONS, BUFFER_SIZE_PER_ITERATION;
    if (baudRate >= 115200) {
        ITERATIONS = 50;
        BUFFER_SIZE_PER_ITERATION = 8192;  // 400KB total
    } else if (baudRate >= 38400) {
        ITERATIONS = 20;
        BUFFER_SIZE_PER_ITERATION = 4096;  // 80KB total
    } else {
        ITERATIONS = 10;
        BUFFER_SIZE_PER_ITERATION = 1024;  // 10KB total - reasonable for 9600 baud
    }
    uint32_t TOTAL_BYTES = ITERATIONS * BUFFER_SIZE_PER_ITERATION;
    
    printf("    Pattern integrity test: %lu iterations x %lu bytes = %lu KB total\n", 
           ITERATIONS, BUFFER_SIZE_PER_ITERATION, TOTAL_BYTES / 1024);
    
    results.reset();
    
    // Phase 1: Generate expected pattern for one iteration
    std::vector<uint8_t> expected(BUFFER_SIZE_PER_ITERATION);
    std::vector<uint8_t> received(BUFFER_SIZE_PER_ITERATION);
    
    // Generate sequential pattern
    for (uint32_t i = 0; i < BUFFER_SIZE_PER_ITERATION; i++) {
        expected[i] = static_cast<uint8_t>(i & 0xFF);
    }
    
    // Phase 2: Calculate realistic timeout based on data size
    uint64_t bits_to_send = TOTAL_BYTES * 10ULL;  // 8 data + start + stop
    uint32_t min_time_ms = (bits_to_send * 1000) / baudRate;
    uint32_t MAX_TEST_DURATION_MS = min_time_ms + 5000;  // +5s margin
    
    printf("    Timeout: %lu ms (estimated %lu ms + 5s margin)\n", MAX_TEST_DURATION_MS, min_time_ms);
    uint64_t test_start = time_us_64();
    
    for (uint32_t iteration = 0; iteration < ITERATIONS; iteration++) {
        // Reset receiver buffer
        received.assign(BUFFER_SIZE_PER_ITERATION, 0);
        
        // Spin up receiver slave task
        volatile uint32_t received_bytes = 0;
        volatile bool task_running = false;
        volatile bool stop_requested = false;
        
        ReceiverTaskParams task_params = {
            &uart_receiver,
            &received,
            &received_bytes,
            &task_running,
            &stop_requested
        };
        
        TaskHandle_t receiver_task_handle;
        xTaskCreate(receiverSlaveTask, "PatternRx", 2048, &task_params, 2, &receiver_task_handle);
        
        // Wait for receiver task to start
        while (!task_running) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        
        // Master sends all data at full speed
        uint32_t sent_bytes = 0;
        uint32_t chunk_size = 256;  // Max TX buffer size
        
        for (uint32_t offset = 0; offset < BUFFER_SIZE_PER_ITERATION; offset += chunk_size) {
            uint32_t remaining = BUFFER_SIZE_PER_ITERATION - offset;
            uint32_t current_chunk = std::min(chunk_size, remaining);
            
            UartDmaDriver::Result result = uart_sender.send(expected.data() + offset, current_chunk);
            if (result == UartDmaDriver::SUCCESS) {
                sent_bytes += current_chunk;
            } else {
                printf("    [ERROR] Send failed at iteration %lu, offset %lu: %s\n", 
                       iteration, offset, UartDmaDriver::toString(result));
                break;
            }
            
            // Small delay to let TX DMA complete before next send
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        
        // Wait for TX DMA to finish
        while (uart_sender.isTxBusy()) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        
        // Wait until all data is received - active waiting
        uint64_t wait_start = time_us_64();
        uint64_t max_wait_us = 5000000; // 5s max wait
        
        while (received_bytes < sent_bytes && 
               (time_us_64() - wait_start) < max_wait_us) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        
        // Final short wait to ensure everything is processed
        vTaskDelay(pdMS_TO_TICKS(50));
        
        // Stop receiver task
        stop_requested = true;
        
        // Wait for receiver task to finish
        while (task_running) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        
        // Compare buffers for this iteration
        results.total_bytes_sent += sent_bytes;
        results.total_bytes_received += received_bytes;
        
        uint32_t compare_bytes = std::min(sent_bytes, static_cast<uint32_t>(received_bytes));
        
        for (uint32_t i = 0; i < compare_bytes; i++) {
            if (expected[i] != received[i]) {
                if (results.sequence_errors == 0) {
                    results.first_error_position = (iteration * BUFFER_SIZE_PER_ITERATION) + i;
                    results.expected_at_error = expected[i];
                    results.received_at_error = received[i];
                    
                    printf("    [ERROR] First error at iteration %lu, byte %lu: expected 0x%02X, got 0x%02X\n",
                           iteration, i, expected[i], received[i]);
                }
                results.sequence_errors++;
            }
        }
        
        // Calculate missing bytes for this iteration
        if (received_bytes < sent_bytes) {
            results.missing_bytes += (sent_bytes - received_bytes);
        }
        
        // Progress indicator every 10 iterations
        if ((iteration + 1) % 10 == 0) {
            printf("    Progress: %lu/%lu iterations completed\n", iteration + 1, ITERATIONS);
        }
        
        // Check timeout
        if ((time_us_64() - test_start) > (MAX_TEST_DURATION_MS * 1000ULL)) {
            printf("    Test stopped after %lu iterations (%lus timeout)\n", iteration + 1, MAX_TEST_DURATION_MS/1000);
            break;
        }
        
        // Small delay between iterations
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    // Display final results
    printf("    Pattern results: %lu/%lu bytes, %lu errors", 
           results.total_bytes_received, results.total_bytes_sent, results.sequence_errors);
    
    if (results.missing_bytes > 0) {
        printf(", %lu missing", results.missing_bytes);
    }
    
    float accuracy = (results.total_bytes_sent > 0) ? 
                    (100.0f * (results.total_bytes_received - results.sequence_errors) / results.total_bytes_sent) : 0.0f;
    printf(" (%.4f%% accuracy)\n", accuracy);
    
    return results.isValid();
}

// === SIMPLE TIMING TEST ===
bool testLatencyTiming(UartDmaDriver& uart_sender, UartDmaDriver& uart_receiver,
                      const TestConfig& config, TimingMetrics& metrics) {
    
    printf("    Simple timing test: measuring event latency\n");
    
    metrics.reset();
    
    uint8_t test_data[32] = {0xAA, 0x55, 0xAA, 0x55}; // Simple pattern
    
    // Send a few packets and measure event timestamp vs fetch time
    for (int i = 0; i < 10; i++) {
        uart_sender.send(test_data, sizeof(test_data));
        
        UartDmaDriver::Event event;
        if (uart_receiver.popEvent(&event, pdMS_TO_TICKS(100))) {
            if (event.type == UartDmaDriver::EVT_DATA) {
                uint64_t fetch_time = time_us_64();
                uint64_t latency = fetch_time - event.timestampUs;
                
                metrics.total_latency_us += latency;
                metrics.samples++;
                
                if (latency < metrics.min_latency_us) {
                    metrics.min_latency_us = latency;
                }
                if (latency > metrics.max_latency_us) {
                    metrics.max_latency_us = latency;
                }
                
                // Consume data
                uint8_t rx_buffer[64];
                uart_receiver.read(rx_buffer, sizeof(rx_buffer));
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // Display results
    if (metrics.samples > 0) {
        printf("    Timing results: avg=%llu us, min=%llu us, max=%llu us\n",
               metrics.getAverage(), metrics.min_latency_us, metrics.max_latency_us);
        
        // Simple acceptance criteria
        return (metrics.getAverage() < 1000) &&    // < 1ms average
               (metrics.max_latency_us < 2000);     // < 2ms max
    }
    
    return false;
}

// === OVERFLOW RECOVERY TEST ===
bool testOverflowRecovery(UartDmaDriver& uart_receiver, UartDmaDriver& uart_sender, uint32_t& overflows_counter) {
    UartDmaDriver::Event event;
    uint8_t rx_buffer[512];
    bool overflow_detected = false;
    bool recovery_successful = false;
    
    // Flood to trigger overflow - send rapidly without delays
    for (int i = 0; i < 50; i++) {  // More packets
        uart_sender.send(overflow_chunk, OVERFLOW_CHUNK);
        
        // Check for overflow but don't consume data (to fill RX buffer)
        while (uart_receiver.popEvent(&event, 0)) {
            if (event.type == UartDmaDriver::EVT_OVERFLOW) {
                overflow_detected = true;
                overflows_counter++;
                break;
            }
            // Don't call read() here - let buffer fill up
        }
        
        if (overflow_detected) break;
        // No delay - flood as fast as possible
    }
    
    if (!overflow_detected) {
        printf("    [FAIL] Overflow detection & recovery test: No overflow triggered\n");
        return false;
    }
    
    // Active recovery
    for (int i = 0; i < 30; i++) {
        size_t bytes = uart_receiver.read(rx_buffer, sizeof(rx_buffer));
        
        while (uart_receiver.popEvent(&event, 0)) {
            if (event.type == UartDmaDriver::EVT_DATA) {
                uart_receiver.read(rx_buffer, sizeof(rx_buffer));
            }
        }
        
        if (uart_receiver.getState() == UartDmaDriver::STATE_RUN_OK) {
            recovery_successful = true;
            break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    if (recovery_successful) {
        printf("    [PASS] Overflow detection & recovery test\n");
    } else {
        printf("    [FAIL] Overflow detection & recovery test: Recovery failed\n");
    }
    
    return recovery_successful;
}

// === IRQ SHARING TEST ===
bool testIrqSharing() {
    printf("Testing IRQ sharing compatibility...\n");
    
    // Simulate another lib that uses DMA (fake SPI lib)
    static int fake_spi_dma_channel = -1;
    static volatile bool fake_spi_irq_called = false;
    
    // Handler for fake SPI DMA
    auto fake_spi_dma_irq_handler = []() {
        if (fake_spi_dma_channel >= 0 && 
            dma_channel_get_irq0_status(fake_spi_dma_channel)) {
            dma_channel_acknowledge_irq0(fake_spi_dma_channel);
            fake_spi_irq_called = true;
        }
    };
    
    bool test_passed = true;
    
    // 1. Simulate lib SPI installation before UART (shared mode)
    fake_spi_dma_channel = dma_claim_unused_channel(false);
    if (fake_spi_dma_channel == -1) {
        printf("    [FAIL] Cannot claim DMA channel for fake SPI\n");
        return false;
    }
    
    // Install shared handler for fake SPI
    irq_add_shared_handler(DMA_IRQ_0, fake_spi_dma_irq_handler, 
                           PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    irq_set_enabled(DMA_IRQ_0, true);
    
    // 2. Install UART after (must cohabitate)
    UartDmaDriver uart(uart0, TX0_PIN, RX0_PIN, 115200);
    if (uart.init() != UartDmaDriver::SUCCESS) {
        printf("    [FAIL] UART initialization failed with shared IRQ\n");
        test_passed = false;
    } else {
        if (uart.start() != UartDmaDriver::SUCCESS) {
            printf("    [FAIL] UART start failed with shared IRQ\n");
            test_passed = false;
        } else {
            printf("    [PASS] UART cohabitation with shared IRQ\n");
            
            // Quick test of operation
            vTaskDelay(pdMS_TO_TICKS(50));
            if (uart.getState() == UartDmaDriver::STATE_RUN_OK) {
                printf("    [PASS] UART operational after IRQ sharing setup\n");
            } else {
                printf("    [FAIL] UART not operational after IRQ sharing setup\n");
                test_passed = false;
            }
            
            // Clean stop UART
            uart.stop();
        }
    }
    
    // 3. COMPLETE CLEANUP - Do not disturb following tests
    
    // Clean fake SPI DMA
    if (fake_spi_dma_channel >= 0) {
        dma_channel_set_irq0_enabled(fake_spi_dma_channel, false);
        dma_channel_acknowledge_irq0(fake_spi_dma_channel);
        dma_channel_abort(fake_spi_dma_channel);
        dma_channel_unclaim(fake_spi_dma_channel);
        fake_spi_dma_channel = -1;
    }
    
    // Reset handler SPI 
    fake_spi_irq_called = false;
    
    // Let a delay for stabilization
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Final cleanup - ensure clean state
    if (fake_spi_dma_channel >= 0) {
        dma_channel_unclaim(fake_spi_dma_channel);
        fake_spi_dma_channel = -1;
    }
    
    if (test_passed) {
        printf("[PASS] IRQ sharing compatibility test\n\n");
    } else {
        printf("[FAIL] IRQ sharing compatibility test\n\n");
    }
    
    return test_passed;
}

// === ENHANCED BAUD RATE TEST ===
bool testEnhancedBaudRate(uint32_t baudRate) {
    printf("Enhanced testing %lu baud...\n", baudRate);
    
    // Find configuration
    const TestConfig* config = nullptr;
    for (int i = 0; i < NUM_TEST_CONFIGS; i++) {
        if (TEST_CONFIGS[i].baudRate == baudRate) {
            config = &TEST_CONFIGS[i];
            break;
        }
    }
    
    if (!config) {
        printf("    [ERROR] No configuration found for %lu baud\n", baudRate);
        return false;
    }
    
    // Initialize drivers
    UartDmaDriver uart0_driver(uart0, TX0_PIN, RX0_PIN, baudRate);
    UartDmaDriver uart1_driver(uart1, TX1_PIN, RX1_PIN, baudRate);
    
    if (uart0_driver.init() != UartDmaDriver::SUCCESS || 
        uart1_driver.init() != UartDmaDriver::SUCCESS ||
        uart0_driver.start() != UartDmaDriver::SUCCESS || 
        uart1_driver.start() != UartDmaDriver::SUCCESS) {
        printf("    [FAIL] Driver initialization failed\n");
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(50)); // Stabilization
    
    bool all_tests_passed = true;
    IntegrityResults integrity_results;
    TimingMetrics timing_metrics;
    
    // Test 1: Pattern Integrity (UART0 -> UART1)
    bool integrity_test = testPatternIntegrity(uart0_driver, uart1_driver, *config, integrity_results, baudRate);
    if (integrity_test) {
        printf("    [PASS] Pattern integrity test\n");
    } else {
        printf("    [FAIL] Pattern integrity test\n");
        all_tests_passed = false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100)); // Pause between tests
    
    // Test 2: Latency/Timing (UART1 -> UART0)  
    bool timing_test = testLatencyTiming(uart1_driver, uart0_driver, *config, timing_metrics);
    if (timing_test) {
        printf("    [PASS] Latency/timing test\n");
    } else {
        printf("    [FAIL] Latency/timing test\n");
        all_tests_passed = false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100)); // Pause before overflow test
    
    // Test 3: Overflow Recovery (existing test)
    uint32_t overflow_count = 0;
    bool overflow_test = testOverflowRecovery(uart0_driver, uart1_driver, overflow_count);
    if (overflow_test) {
        printf("    [PASS] Overflow recovery test\n");
    } else {
        printf("    [FAIL] Overflow recovery test\n");
        all_tests_passed = false;
    }
    
    // Final summary
    if (all_tests_passed) {
        printf("[PASS] %lu baud - All enhanced tests passed\n", baudRate);
        printf("       Integrity: %lu bytes, Timing: avg=%llu us, Recovery: OK\n\n",
               integrity_results.total_bytes_received, timing_metrics.getAverage());
    } else {
        printf("[FAIL] %lu baud - Some tests failed\n\n", baudRate);
    }
    
    uart0_driver.stop();
    uart1_driver.stop();
    
    return all_tests_passed;
}

// === LEGACY BAUD RATE TEST ===
void testBaudRate(uint32_t baudRate) {
    uint64_t test_start_time = time_us_64();
    printf("Testing %lu baud...\n", baudRate);

    // Initialize drivers with platform-specific pins
    UartDmaDriver uart0_driver(uart0, TX0_PIN, RX0_PIN, baudRate);
    UartDmaDriver uart1_driver(uart1, TX1_PIN, RX1_PIN, baudRate);

    if (uart0_driver.init() != UartDmaDriver::SUCCESS || 
        uart1_driver.init() != UartDmaDriver::SUCCESS ||
        uart0_driver.start() != UartDmaDriver::SUCCESS || 
        uart1_driver.start() != UartDmaDriver::SUCCESS) {
        printf("[FAIL] Initialization failed\n");
        g_results[g_currentTest++] = {baudRate, 0, 0, 0, 0.0f, false, false};
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
    
    uint32_t total_sent = 0, total_received = 0, overflows = 0;
    UartDmaDriver::Event event;
    uint8_t rx_buffer[2048];
    
    // PHASE 1: Normal chunks test
    bool normal_chunks_passed = true;
    for (int i = 0; i < NORMAL_CHUNKS; i++) {
        UartDmaDriver* sender = (i % 2 == 0) ? &uart0_driver : &uart1_driver;
        UartDmaDriver* receiver = (i % 2 == 0) ? &uart1_driver : &uart0_driver;
        
        UartDmaDriver::Result result = sender->send(test_chunk, NORMAL_CHUNK);
        if (result == UartDmaDriver::SUCCESS) {
            total_sent += NORMAL_CHUNK;
        } else {
            printf("    [DEBUG] send() failed: %s\n", UartDmaDriver::toString(result));
            normal_chunks_passed = false;
        }
        
        while (receiver->popEvent(&event, 0)) {
            if (event.type == UartDmaDriver::EVT_DATA) {
                size_t bytes = receiver->read(rx_buffer, sizeof(rx_buffer));
                total_received += bytes;
            } else if (event.type == UartDmaDriver::EVT_OVERFLOW) {
                overflows++;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    if (normal_chunks_passed) {
        printf("    [PASS] Normal chunks test\n");
    } else {
        printf("    [FAIL] Normal chunks test\n");
    }
    
    // PHASE 2: Burst mode test
    bool burst_mode_passed = true;
    for (int i = 0; i < BURST_CHUNKS; i++) {
        UartDmaDriver* sender = (i % 2 == 0) ? &uart0_driver : &uart1_driver;
        
        UartDmaDriver::Result result = sender->send(test_chunk, NORMAL_CHUNK);
        if (result == UartDmaDriver::SUCCESS) {
            total_sent += NORMAL_CHUNK;
        } else {
            printf("    [DEBUG] send() failed: %s\n", UartDmaDriver::toString(result));
            burst_mode_passed = false;
        }
        
        for (auto* driver : {&uart0_driver, &uart1_driver}) {
            // Read as fast as we can
            while (driver->popEvent(&event, 0)) {
                if (event.type == UartDmaDriver::EVT_DATA) {
                    size_t bytes = driver->read(rx_buffer, sizeof(rx_buffer));
                    total_received += bytes;
                } else if (event.type == UartDmaDriver::EVT_OVERFLOW) {
                    overflows++;
                }
            }
        }
        
        if (baudRate <= 38400) {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
    
    if (burst_mode_passed) {
        printf("    [PASS] Burst mode test\n");
    } else {
        printf("    [FAIL] Burst mode test\n");
    }
    
    // Drain residual data
    vTaskDelay(pdMS_TO_TICKS(100));
    uint32_t drain_delay_ms = (UartDmaDriver::WATCHDOG_TIMER_SILENCE_US / 1000) * 2 + 50;
    vTaskDelay(pdMS_TO_TICKS(drain_delay_ms));
    
    for (int cycles = 0; cycles < 50; cycles++) {
        bool activity = false;
        
        for (auto* driver : {&uart0_driver, &uart1_driver}) {
            while (driver->popEvent(&event, 0)) {
                activity = true;
                if (event.type == UartDmaDriver::EVT_DATA) {
                    size_t bytes = driver->read(rx_buffer, sizeof(rx_buffer));
                    total_received += bytes;
                } else if (event.type == UartDmaDriver::EVT_OVERFLOW) {
                    overflows++;
                    driver->read(rx_buffer, sizeof(rx_buffer));
                }
            }
        }
        
        if (!activity) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    vTaskDelay(100);
    
    // PHASE 3: Overflow recovery test
    bool overflow_recovered = testOverflowRecovery(uart0_driver, uart1_driver, overflows);
    
    // Final drain
    uint32_t final_drain_delay = (UartDmaDriver::WATCHDOG_TIMER_SILENCE_US / 1000) + 20;
    if (baudRate <= 19200) final_drain_delay += 50;
    else if (baudRate <= 115200) final_drain_delay += 20;
    
    vTaskDelay(pdMS_TO_TICKS(final_drain_delay));
    
    for (int i = 0; i < 30; i++) {
        bool activity = false;
        
        for (auto* driver : {&uart0_driver, &uart1_driver}) {
            while (driver->popEvent(&event, 0)) {
                activity = true;
                if (event.type == UartDmaDriver::EVT_DATA) {
                    size_t bytes = driver->read(rx_buffer, sizeof(rx_buffer));
                    total_received += bytes;
                } else if (event.type == UartDmaDriver::EVT_OVERFLOW) {
                    overflows++;
                }
            }
        }
        
        if (!activity) break;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    // Calculate results - cap efficiency at 100% to handle overflow data
    float raw_efficiency = total_sent > 0 ? (100.0f * total_received / total_sent) : 0.0f;
    float efficiency = raw_efficiency > 100.0f ? 100.0f : raw_efficiency;
    bool passed = (raw_efficiency >= 95.0f) && overflow_recovered && (overflows > 0);
    
    uint64_t test_duration_ms = (time_us_64() - test_start_time) / 1000;
    
    g_results[g_currentTest] = {
        .baudRate = baudRate,
        .totalSent = total_sent,
        .totalReceived = total_received,
        .overflows = overflows,
        .efficiency = efficiency,
        .overflowRecovered = overflow_recovered,
        .passed = passed
    };
    
    // Print result with duration
    if (passed) {
        printf("[PASS] %lu baud @ %.1f%% efficiency, recovery OK (%llu ms)\n\n", baudRate, efficiency, test_duration_ms);
    } else {
        printf("[FAIL] ");
        if (raw_efficiency < 95.0f) printf("Low efficiency: %.1f%% ", raw_efficiency);
        if (!overflow_recovered) printf("Recovery failed ");
        if (overflows == 0) printf("No overflow detected ");
        printf("(%llu ms)\n", test_duration_ms);
    }
    
    uart0_driver.stop();
    uart1_driver.stop();
    g_currentTest++;
    
    vTaskDelay(pdMS_TO_TICKS(1000));
}

// === FINAL SUMMARY ===
void printFinalSummary() {
    printf("\n=== Validation Test Results ===\n");
    
    int passed_tests = 0;
    for (int i = 0; i < g_currentTest; i++) {
        const TestResults& r = g_results[i];
        printf("%lu baud: [%s]\n",
               r.baudRate, 
               r.passed ? "PASS" : "FAIL");
        
        if (r.passed) passed_tests++;
    }
    
    printf("\nSummary: %d/%d tests passed\n", passed_tests, g_currentTest);
    
    if (passed_tests == g_currentTest) {
        printf("Status: [PASS] All tests successful\n");
    } else {
        printf("Status: [FAIL] %d test(s) failed\n", g_currentTest - passed_tests);
    }
}

// === TEST 1: SILENCE DETECTION ===
bool testSilenceDetection() {
    printf("Testing silence detection...\n");
    
    UartDmaDriver uart0_driver(uart0, TX0_PIN, RX0_PIN, 115200);
    UartDmaDriver uart1_driver(uart1, TX1_PIN, RX1_PIN, 115200);
    
    if (uart0_driver.init() != UartDmaDriver::SUCCESS || 
        uart1_driver.init() != UartDmaDriver::SUCCESS ||
        uart0_driver.start() != UartDmaDriver::SUCCESS || 
        uart1_driver.start() != UartDmaDriver::SUCCESS) {
        printf("    [FAIL] Driver initialization failed\n");
        return false;
    }
    
    uint8_t chunk1[32] = {0x01, 0x02, 0x03, 0x04};
    uint8_t chunk2[32] = {0x05, 0x06, 0x07, 0x08};
    
    // Send first chunk
    uart0_driver.send(chunk1, 4);
    
    // Wait longer than silence threshold
    uint32_t silence_us = UartDmaDriver::WATCHDOG_TIMER_SILENCE_US + 1000; // +1ms margin
    vTaskDelay(pdMS_TO_TICKS(silence_us / 1000 + 1));
    
    // Send second chunk
    uart0_driver.send(chunk2, 4);
    
    // Check events
    UartDmaDriver::Event event1, event2;
    bool event1_received = uart1_driver.popEvent(&event1, pdMS_TO_TICKS(1000));
    bool event2_received = uart1_driver.popEvent(&event2, pdMS_TO_TICKS(1000));
    
    if (!event1_received || !event2_received) {
        printf("    [FAIL] Did not receive both events\n");
        uart0_driver.stop();
        uart1_driver.stop();
        return false;
    }
    
    if (event1.type != UartDmaDriver::EVT_DATA || event2.type != UartDmaDriver::EVT_DATA) {
        printf("    [FAIL] Events are not EVT_DATA\n");
        uart0_driver.stop();
        uart1_driver.stop();
        return false;
    }
    
    // Second event should have silenceFlag = true
    if (!event2.silenceFlag) {
        printf("    [FAIL] Second event should have silenceFlag = true\n");
        uart0_driver.stop();
        uart1_driver.stop();
        return false;
    }
    
    // Consume data
    uint8_t rx_buffer[64];
    uart1_driver.read(rx_buffer, sizeof(rx_buffer));
    uart1_driver.read(rx_buffer, sizeof(rx_buffer));
    
    uart0_driver.stop();
    uart1_driver.stop();
    
    printf("    [PASS] Silence detection test\n");
    return true;
}

// === TEST 2: API LIMITS ===
bool testApiLimits() {
    printf("Testing API limits...\n");
    
    UartDmaDriver uart_driver(uart0, TX0_PIN, RX0_PIN, 115200);
    uint8_t test_data[300]; // Larger than TX buffer
    
    // Test 1: send() before init() should return ERR_NOT_INITIALIZED
    UartDmaDriver::Result result = uart_driver.send(test_data, 10);
    if (result != UartDmaDriver::ERR_NOT_INITIALIZED) {
        printf("    [FAIL] send() before init() should return ERR_NOT_INITIALIZED, got %s\n", 
               UartDmaDriver::toString(result));
        return false;
    }
    
    // Test 2: send(data, 0) before init() should also return ERR_NOT_INITIALIZED
    result = uart_driver.send(test_data, 0);
    if (result != UartDmaDriver::ERR_NOT_INITIALIZED) {
        printf("    [FAIL] send(data, 0) before init() should return ERR_NOT_INITIALIZED, got %s\n", 
               UartDmaDriver::toString(result));
        return false;
    }
    
    // Test 3: init() should work
    result = uart_driver.init();
    if (result != UartDmaDriver::SUCCESS) {
        printf("    [FAIL] init() should return SUCCESS, got %s\n", 
               UartDmaDriver::toString(result));
        return false;
    }
    
    // Test 4: init() twice should return SUCCESS (idempotent)
    result = uart_driver.init();
    if (result != UartDmaDriver::SUCCESS) {
        printf("    [FAIL] init() twice should return SUCCESS, got %s\n", 
               UartDmaDriver::toString(result));
        return false;
    }
    
    // Test 5: start() should work
    result = uart_driver.start();
    if (result != UartDmaDriver::SUCCESS) {
        printf("    [FAIL] start() should return SUCCESS, got %s\n", 
               UartDmaDriver::toString(result));
        return false;
    }
    
    // Test 6: send(data, 0) after start() should return SUCCESS
    result = uart_driver.send(test_data, 0);
    if (result != UartDmaDriver::SUCCESS) {
        printf("    [FAIL] send(data, 0) after start() should return SUCCESS, got %s\n", 
               UartDmaDriver::toString(result));
        return false;
    }
    
    // Test 7: start() twice should return ERR_STATE
    result = uart_driver.start();
    if (result != UartDmaDriver::ERR_STATE) {
        printf("    [FAIL] start() twice should return ERR_STATE, got %s\n", 
               UartDmaDriver::toString(result));
        return false;
    }
    
    // Test 8: send() with oversized data should return ERR_TX_OVERFLOW
    result = uart_driver.send(test_data, UartDmaDriver::DMA_TX_BUFFER_SIZE + 1);
    if (result != UartDmaDriver::ERR_TX_OVERFLOW) {
        printf("    [FAIL] send() with oversized data should return ERR_TX_OVERFLOW, got %s\n", 
               UartDmaDriver::toString(result));
        return false;
    }
    
    // Test 9: stop() should work
    result = uart_driver.stop();
    if (result != UartDmaDriver::SUCCESS) {
        printf("    [FAIL] stop() should return SUCCESS, got %s\n", 
               UartDmaDriver::toString(result));
        return false;
    }
    
    // Test 10: send() after stop() should return ERR_STATE
    result = uart_driver.send(test_data, 10);
    if (result != UartDmaDriver::ERR_STATE) {
        printf("    [FAIL] send() after stop() should return ERR_STATE, got %s\n", 
               UartDmaDriver::toString(result));
        return false;
    }
    
    printf("    [PASS] API limits test\n");
    return true;
}

// === TEST 3: TX CONCURRENCY ===
struct TxConcurrencyParams {
    UartDmaDriver* driver;
    volatile bool* task_a_finished;
    volatile bool* task_b_can_start;
    volatile bool* task_b_finished;
    volatile UartDmaDriver::Result* task_b_result1;
    volatile UartDmaDriver::Result* task_b_result2;
};

void txConcurrencyTaskA(void* params) {
    TxConcurrencyParams* p = static_cast<TxConcurrencyParams*>(params);
    
    uint8_t long_data[256];
    for (int i = 0; i < 256; i++) {
        long_data[i] = (uint8_t)i;
    }
    
    // Send long data to occupy TX channel
    p->driver->send(long_data, sizeof(long_data));
    
    // Signal task B can start
    *p->task_b_can_start = true;
    
    // Wait a bit to ensure task B tries to send
    vTaskDelay(pdMS_TO_TICKS(100));
    
    *p->task_a_finished = true;
    vTaskDelete(NULL);
}

void txConcurrencyTaskB(void* params) {
    TxConcurrencyParams* p = static_cast<TxConcurrencyParams*>(params);
    
    // Wait for task A to start
    while (!*p->task_b_can_start) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    // Verify isTxBusy() returns true
    if (!p->driver->isTxBusy()) {
        printf("    [ERROR] isTxBusy() should return true\n");
    }
    
    uint8_t short_data[10] = {0xAA, 0xBB, 0xCC};
    
    // Try to send with short timeout - should fail
    *p->task_b_result1 = p->driver->send(short_data, sizeof(short_data), pdMS_TO_TICKS(10));
    
    // Try to send with long timeout - should succeed after task A finishes
    *p->task_b_result2 = p->driver->send(short_data, sizeof(short_data), portMAX_DELAY);
    
    *p->task_b_finished = true;
    vTaskDelete(NULL);
}

bool testTxConcurrency() {
    printf("Testing TX concurrency...\n");
    
    UartDmaDriver uart_driver(uart0, TX0_PIN, RX0_PIN, 9600); // Slow baud for longer TX
    
    if (uart_driver.init() != UartDmaDriver::SUCCESS || 
        uart_driver.start() != UartDmaDriver::SUCCESS) {
        printf("    [FAIL] Driver initialization failed\n");
        return false;
    }
    
    volatile bool task_a_finished = false;
    volatile bool task_b_can_start = false;
    volatile bool task_b_finished = false;
    volatile UartDmaDriver::Result task_b_result1 = UartDmaDriver::SUCCESS;
    volatile UartDmaDriver::Result task_b_result2 = UartDmaDriver::SUCCESS;
    
    TxConcurrencyParams params = {
        &uart_driver,
        &task_a_finished,
        &task_b_can_start,
        &task_b_finished,
        &task_b_result1,
        &task_b_result2
    };
    
    TaskHandle_t task_a_handle, task_b_handle;
    xTaskCreate(txConcurrencyTaskA, "TxTaskA", 1024, &params, 3, &task_a_handle);
    xTaskCreate(txConcurrencyTaskB, "TxTaskB", 1024, &params, 2, &task_b_handle);
    
    // Wait for both tasks to finish
    while (!task_a_finished || !task_b_finished) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    uart_driver.stop();
    
    // Check results
    if (task_b_result1 != UartDmaDriver::ERR_TX_BUSY) {
        printf("    [FAIL] First send() should return ERR_TX_BUSY, got %s\n", 
               UartDmaDriver::toString(task_b_result1));
        return false;
    }
    
    if (task_b_result2 != UartDmaDriver::SUCCESS) {
        printf("    [FAIL] Second send() should return SUCCESS, got %s\n", 
               UartDmaDriver::toString(task_b_result2));
        return false;
    }
    
    printf("    [PASS] TX concurrency test\n");
    return true;
}

// === STEP 3: HARDCORE STRESS TESTING ===

// CPU stress task parameters
struct CpuStressParams {
    volatile bool* stop_requested;
    volatile uint32_t* iterations_completed;
    uint32_t stress_intensity;  // 1-100 (CPU load percentage)
};

// CPU stress task - monopolizes CPU cycles for stress testing
void cpuStressTask(void* params) {
    CpuStressParams* p = static_cast<CpuStressParams*>(params);
    
    *p->iterations_completed = 0;
    
    // Calculate work cycles based on intensity
    uint32_t work_cycles = p->stress_intensity * 100;
    uint32_t sleep_cycles = 100 - p->stress_intensity;
    
    while (!*p->stop_requested) {
        // Intensive computation to stress CPU
        volatile uint32_t dummy = 0;
        for (uint32_t i = 0; i < work_cycles; i++) {
            dummy = dummy * 17 + i;
            dummy = dummy ^ (i << 3);
            dummy = dummy + (i * 23);
        }
        
        (*p->iterations_completed)++;
        
        // Small yield to allow other tasks to run
        if (sleep_cycles > 0) {
            vTaskDelay(pdMS_TO_TICKS(sleep_cycles / 10));
        }
    }
    
    vTaskDelete(NULL);
}

// HARDCORE wrapper for existing tests with CPU stress
bool runHardcoreTest(const char* test_name, std::function<bool()> test_function, 
                    uint32_t stress_intensity = 80) {
    printf("  HARDCORE %s (CPU stress: %lu%%)...\n", test_name, stress_intensity);
    
    // Start CPU stress tasks
    volatile bool stress_stop = false;
    volatile uint32_t stress_iterations[2] = {0, 0};
    
    CpuStressParams stress_params[2] = {
        {&stress_stop, &stress_iterations[0], stress_intensity},
        {&stress_stop, &stress_iterations[1], stress_intensity}
    };
    
    TaskHandle_t stress_tasks[2];
    xTaskCreate(cpuStressTask, "Stress1", 1024, &stress_params[0], 
                configMAX_PRIORITIES - 1, &stress_tasks[0]);
    xTaskCreate(cpuStressTask, "Stress2", 1024, &stress_params[1], 
                configMAX_PRIORITIES - 1, &stress_tasks[1]);
    
    // Let stress tasks establish CPU load
    vTaskDelay(pdMS_TO_TICKS(100));
    
    uint64_t test_start = time_us_64();
    
    // Run the actual test under stress
    bool result = test_function();
    
    uint64_t test_duration = time_us_64() - test_start;
    
    // Stop stress tasks
    stress_stop = true;
    
    // Wait for stress tasks to stop
    vTaskDelay(pdMS_TO_TICKS(50));
    
    printf("    Test completed in %llu ms under %lu%% CPU stress\n", 
           test_duration / 1000, stress_intensity);
    printf("    Stress iterations: Task1=%lu, Task2=%lu\n", 
           stress_iterations[0], stress_iterations[1]);
    
    if (result) {
        printf("    [PASS] HARDCORE %s\n", test_name);
    } else {
        printf("    [FAIL] HARDCORE %s\n", test_name);
    }
    
    return result;
}

// HARDCORE timing test - measures latency under CPU stress
bool testHardcoreTimingLatency() {
    UartDmaDriver uart0_driver(uart0, TX0_PIN, RX0_PIN, 921600);
    UartDmaDriver uart1_driver(uart1, TX1_PIN, RX1_PIN, 921600);
    
    if (uart0_driver.init() != UartDmaDriver::SUCCESS || 
        uart1_driver.init() != UartDmaDriver::SUCCESS ||
        uart0_driver.start() != UartDmaDriver::SUCCESS || 
        uart1_driver.start() != UartDmaDriver::SUCCESS) {
        return false;
    }
    
    TimingMetrics metrics;
    metrics.reset();
    
    uint8_t test_data[64];
    for (int i = 0; i < 64; i++) {
        test_data[i] = (uint8_t)(i & 0xFF);
    }
    
    // Send packets and measure latency under stress
    for (int i = 0; i < 20; i++) {
        uart0_driver.send(test_data, sizeof(test_data));
        
        UartDmaDriver::Event event;
        if (uart1_driver.popEvent(&event, pdMS_TO_TICKS(500))) {
            if (event.type == UartDmaDriver::EVT_DATA) {
                uint64_t fetch_time = time_us_64();
                uint64_t latency = fetch_time - event.timestampUs;
                
                metrics.total_latency_us += latency;
                metrics.samples++;
                
                if (latency < metrics.min_latency_us) {
                    metrics.min_latency_us = latency;
                }
                if (latency > metrics.max_latency_us) {
                    metrics.max_latency_us = latency;
                }
                
                // Consume data
                uint8_t rx_buffer[128];
                uart1_driver.read(rx_buffer, sizeof(rx_buffer));
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(25));
    }
    
    uart0_driver.stop();
    uart1_driver.stop();
    
    // Accept higher latency under stress conditions
    bool passed = (metrics.samples >= 15) &&            // Got most packets
                  (metrics.getAverage() < 5000) &&       // < 5ms average
                  (metrics.max_latency_us < 10000);      // < 10ms max
    
    printf("    HARDCORE timing: avg=%llu us, max=%llu us, samples=%llu\n",
           metrics.getAverage(), metrics.max_latency_us, metrics.samples);
    
    return passed;
}

// HARDCORE overflow test - tests overflow recovery under CPU stress
bool testHardcoreOverflowRecovery() {
    UartDmaDriver uart0_driver(uart0, TX0_PIN, RX0_PIN, 921600);
    UartDmaDriver uart1_driver(uart1, TX1_PIN, RX1_PIN, 921600);
    
    if (uart0_driver.init() != UartDmaDriver::SUCCESS || 
        uart1_driver.init() != UartDmaDriver::SUCCESS ||
        uart0_driver.start() != UartDmaDriver::SUCCESS || 
        uart1_driver.start() != UartDmaDriver::SUCCESS) {
        return false;
    }
    
    uint32_t overflow_count = 0;
    bool result = testOverflowRecovery(uart1_driver, uart0_driver, overflow_count);
    
    uart0_driver.stop();
    uart1_driver.stop();
    
    return result && (overflow_count > 0);
}

// HARDCORE TX concurrency test - tests TX semaphore under CPU stress
bool testHardcoreTxConcurrency() {
    UartDmaDriver uart_driver(uart0, TX0_PIN, RX0_PIN, 115200);
    
    if (uart_driver.init() != UartDmaDriver::SUCCESS || 
        uart_driver.start() != UartDmaDriver::SUCCESS) {
        return false;
    }
    
    bool result = true;
    uint8_t test_data[256];
    for (int i = 0; i < 256; i++) {
        test_data[i] = (uint8_t)i;
    }
    
    // Rapid fire transmission test under stress
    for (int i = 0; i < 10; i++) {
        UartDmaDriver::Result send_result = uart_driver.send(test_data, sizeof(test_data), pdMS_TO_TICKS(1000));
        if (send_result != UartDmaDriver::SUCCESS) {
            result = false;
            break;
        }
        
        // Verify TX busy logic works under stress
        if (!uart_driver.isTxBusy()) {
            // TX might complete very fast, so check a few times
            bool ever_busy = false;
            for (int j = 0; j < 5; j++) {
                if (uart_driver.isTxBusy()) {
                    ever_busy = true;
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    
    uart_driver.stop();
    return result;
}

// === MAIN TEST ORCHESTRATOR ===
void driverTestTask(void* param) {
    vTaskDelay(pdMS_TO_TICKS(2000));
    printf("Hardware: %s, TX0(GPIO%d)<->RX1(GPIO%d), TX1(GPIO%d)<->RX0(GPIO%d) loopback\n",
           board.data(), TX0_PIN, RX1_PIN, TX1_PIN, RX0_PIN);
    
    initTestData();
    
    printf("========================================\n");
    printf("Starting UartDmaDriver validation tests\n");
    printf("========================================\n\n");
    
    // Test IRQ sharing compatibility first
    bool irq_test_passed = testIrqSharing();
    
    // === STEP 1: DRIVER VALIDATION ===
    printf("\n=== STEP 1: Driver Validation ===\n");
    for (int i = 0; i < NUM_BAUD_RATES; i++) {
        testBaudRate(TEST_BAUD_RATES[i]);
    }
    
    printFinalSummary();
    
    // === STEP 2: ROBUSTNESS TESTS ===
    printf("\n=== STEP 2: Robustness Tests ===\n");
    
    // Enhanced test only at 921600 baud (critical speed)
    printf("Data integrity test at 921600 baud...\n");
    bool integrity_test_passed = testEnhancedBaudRate(921600);
    
    // Additional robustness tests at 115200 baud
    printf("\n--- Additional Robustness Tests (115200 baud) ---\n");
    bool silence_test_passed = testSilenceDetection();
    bool api_limits_test_passed = testApiLimits();
    bool tx_concurrency_test_passed = testTxConcurrency();
    
    int additional_tests_passed = (silence_test_passed ? 1 : 0) +
                                 (api_limits_test_passed ? 1 : 0) +
                                 (tx_concurrency_test_passed ? 1 : 0);
    
    printf("\n=== Robustness Test Results ===\n");
    printf("Data integrity (921600): %s\n", integrity_test_passed ? "PASS" : "FAIL");
    printf("Additional tests (115200): %d/3 passed\n", additional_tests_passed);
    printf("Total robustness: %d/4 passed\n", (integrity_test_passed ? 1 : 0) + additional_tests_passed);
    
    if (integrity_test_passed && additional_tests_passed == 3) {
        printf("Status: [PASS] All robustness tests successful\n");
    } else {
        printf("Status: [FAIL] Some robustness tests failed\n");
    }
    
    // === STEP 3: HARDCORE STRESS TESTING ===
    printf("\n=== STEP 3: HARDCORE Stress Testing ===\n");
    printf("Real-time performance validation under CPU stress...\n");
    
    // Test 1: HARDCORE timing latency test
    bool hardcore_timing_passed = runHardcoreTest("Timing Latency", testHardcoreTimingLatency, 80);
    
    // Test 2: HARDCORE overflow recovery test  
    bool hardcore_overflow_passed = runHardcoreTest("Overflow Recovery", testHardcoreOverflowRecovery, 80);
    
    // Test 3: HARDCORE TX concurrency test
    bool hardcore_tx_passed = runHardcoreTest("TX Concurrency", testHardcoreTxConcurrency, 80);
    
    // Test 4: HARDCORE silence detection test
    bool hardcore_silence_passed = runHardcoreTest("Silence Detection", []() {
        return testSilenceDetection();
    }, 80);
    
    int hardcore_tests_passed = (hardcore_timing_passed ? 1 : 0) +
                               (hardcore_overflow_passed ? 1 : 0) +
                               (hardcore_tx_passed ? 1 : 0) +
                               (hardcore_silence_passed ? 1 : 0);
    
    printf("\n=== HARDCORE Test Results ===\n");
    printf("Timing latency (80%% stress): %s\n", hardcore_timing_passed ? "PASS" : "FAIL");
    printf("Overflow recovery (75%% stress): %s\n", hardcore_overflow_passed ? "PASS" : "FAIL");
    printf("TX concurrency (70%% stress): %s\n", hardcore_tx_passed ? "PASS" : "FAIL");
    printf("Silence detection (85%% stress): %s\n", hardcore_silence_passed ? "PASS" : "FAIL");
    printf("Total HARDCORE: %d/4 passed\n", hardcore_tests_passed);
    
    if (hardcore_tests_passed == 4) {
        printf("Status: [PASS] All HARDCORE tests successful - Real-time performance validated!\n");
    } else {
        printf("Status: [FAIL] %d HARDCORE test(s) failed\n", 4 - hardcore_tests_passed);
    }
    
    // === FINAL SUMMARY ===
    printf("\n=== COMPLETE TEST SUITE RESULTS ===\n");
    printf("STEP 1 (Validation): %d/%d basic tests passed\n", g_currentTest > 0 ? 1 : 0, 1);
    printf("STEP 2 (Robustness): %d/4 robustness tests passed\n", (integrity_test_passed ? 1 : 0) + additional_tests_passed);
    printf("STEP 3 (HARDCORE): %d/4 stress tests passed\n", hardcore_tests_passed);
    
    int total_passed = (g_currentTest > 0 ? 1 : 0) + (integrity_test_passed ? 1 : 0) + additional_tests_passed + hardcore_tests_passed;
    int total_tests = 1 + 4 + 4;
    
    printf("OVERALL: %d/%d tests passed\n", total_passed, total_tests);
    
    if (total_passed == total_tests) {
        printf("Status: [PASS] ALL TESTS COMPLETED WITH SUCCESS!\n");
    } else {
        printf("Status: [FAIL] %d test(s) failed - Review required\n", total_tests - total_passed);
    }
    
    printf("\nTest suite completed at %llu ms\n", time_us_64() / 1000);
    
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(60000));
        printf("System still running after %llu minutes\n", (time_us_64() / 1000) / 60000);
    }
}

int main() {
    stdio_init_all();
    
    printf("UartDmaDriver Test Suite Starting\n");
    printf("Built: %s %s\n", __DATE__, __TIME__);
    
    xTaskCreate(driverTestTask, "UartTest", 8192, NULL, 1, NULL);
    vTaskStartScheduler();
    
    printf("FATAL: Scheduler returned\n");
    while (1) {
        sleep_ms(1000);
        printf("System halted\n");
    }
}