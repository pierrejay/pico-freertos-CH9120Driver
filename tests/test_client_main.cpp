/**
 * @file test_client_main.cpp
 * @brief Comprehensive test suite for CH9120Driver TCP Client functionality
 * 
 * This file implements the complete test specification for CH9120Driver client mode,
 * including connectivity, performance, robustness and lifecycle tests.
 * 
 * Test Prerequisites:
 * - Python mock_server.py running on target network
 * - CH9120 module connected to same network segment  
 * - Network configuration matching test constants below
 */

#include "CH9120Driver.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <cstdio>
#include <cstring>
#include <cassert>
#include <algorithm>
#include "pico/stdlib.h"

// ===================================================================================
// TEST CONFIGURATION
// ===================================================================================

// Network configuration - MODIFY THESE FOR YOUR TEST SETUP
static constexpr uint8_t TEST_SERVER_IP[4] = {192, 168, 0, 234};  // Python mock server IP
static constexpr uint16_t TEST_SERVER_PORT_1 = 8000;              // Primary test server port
static constexpr uint16_t TEST_SERVER_PORT_2 = 8001;              // Secondary server for reconfig tests
static constexpr uint8_t PICO_LOCAL_IP[4] = {192, 168, 0, 124};   // Pico IP (if not using DHCP)
static constexpr uint16_t PICO_LOCAL_PORT = 9000;                 // Pico local port

// Test timing configuration
static constexpr TickType_t TEST_CONNECT_TIMEOUT = pdMS_TO_TICKS(10000);  // 10s connect timeout
static constexpr TickType_t TEST_RECV_TIMEOUT = pdMS_TO_TICKS(5000);      // 5s recv timeout
static constexpr TickType_t TEST_SHORT_TIMEOUT = pdMS_TO_TICKS(2000);     // 2s for failure tests

// Test constants - realistic for industrial/Modbus traffic  
static constexpr size_t LARGE_DATA_SIZE = 1024;  // Safe size under CH9120 buffer limits
static constexpr size_t BURST_DELAY_MS = 50;     // 50ms between bursts for CH9120 processing
static constexpr size_t PING_PONG_ITERATIONS = 100;
static constexpr size_t LIFECYCLE_ITERATIONS = 10;

// Global driver instance (using default Waveshare pins)
CH9120Driver client_socket(uart1);


// Helper to completely purge RX buffer between tests
void purge_rx_buffer() {
    printf("Purging RX buffer...\n");
    uint8_t purge_buffer[256];
    int purged_total = 0;
    while (true) {
        int r = client_socket.recv(purge_buffer, sizeof(purge_buffer), pdMS_TO_TICKS(100));
        if (r <= 0) {
            // Wait 200ms to ensure no more data is coming
            vTaskDelay(pdMS_TO_TICKS(500));
            int again = client_socket.recv(purge_buffer, sizeof(purge_buffer), pdMS_TO_TICKS(0));
            if (again <= 0) break; // Really nothing left
            r = again; // Continue purging
        }
        purged_total += r;
        printf("Purged %d bytes\n", r);
    }
    if (purged_total > 0) {
        printf("*** WARNING: Purged %d stale bytes from RX buffer! ***\n", purged_total);
    }
}

// Test results tracking
struct TestResults {
    int tests_run = 0;
    int tests_passed = 0;
    int tests_failed = 0;
    
    void record_pass(const char* test_name) {
        tests_run++;
        tests_passed++;
        printf("✓ PASS: %s\n", test_name);
    }
    
    void record_fail(const char* test_name, const char* reason = nullptr) {
        tests_run++;
        tests_failed++;
        printf("✗ FAIL: %s", test_name);
        if (reason) printf(" - %s", reason);
        printf("\n");
    }
    
    void print_summary() {
        printf("\n=== TEST SUMMARY ===\n");
        printf("Tests run: %d\n", tests_run);
        printf("Passed: %d\n", tests_passed);
        printf("Failed: %d\n", tests_failed);
        printf("Success rate: %.1f%%\n", tests_run > 0 ? (100.0f * tests_passed / tests_run) : 0.0f);
        if (tests_failed == 0) {
            printf("🎉 ALL TESTS PASSED!\n");
        }
    }
};

static TestResults g_results;

// ===================================================================================
// TEST UTILITIES
// ===================================================================================

/**
 * @brief Create a standard client configuration for testing
 */
CH9120Driver::NetworkConfig create_client_config(const uint8_t server_ip[4], uint16_t server_port, bool use_dhcp = true) {
    CH9120Driver::NetworkConfig config;
    memcpy(config.targetIp, server_ip, 4);
    memcpy(config.localIp, PICO_LOCAL_IP, 4);
    config.targetPort = server_port;
    config.localPort = PICO_LOCAL_PORT;
    config.mode = CH9120Driver::Mode::TCP_CLIENT;
    config.useDhcp = use_dhcp;
    return config;
}

/**
 * @brief Initialize the driver (common setup for all tests)
 */
bool setup_driver() {
    if (client_socket.begin() != CH9120Driver::SUCCESS) {
        printf("ERROR: Failed to initialize CH9120Driver\n");
        return false;
    }
    return true;
}

/**
 * @brief Robust connection with automatic retry
 */
int connect_with_retry(const CH9120Driver::NetworkConfig& config, const char* test_name) {
    printf("Connecting to server (will retry until successful)...\n");
    
    int attempt = 1;
    while (true) {
        printf("  Attempt %d: Connecting to %d.%d.%d.%d:%d...\n", 
               attempt,
               config.targetIp[0], config.targetIp[1], config.targetIp[2], config.targetIp[3],
               config.targetPort);
        
        int result = client_socket.connect(config, TEST_CONNECT_TIMEOUT);
        if (result == 0) {
            printf("✅ Connected successfully on attempt %d\n", attempt);
            return 0;
        }
        
        printf("  ❌ Connection failed (error %d), retrying in 3s...\n", result);
        vTaskDelay(pdMS_TO_TICKS(3000));  // Wait 3s before retry
        attempt++;
        
        // For safety, don't retry forever in case of config issues
        if (attempt > 100) {  // Max 5 minutes of retries
            printf("⚠️ Giving up after %d attempts\n", attempt - 1);
            return result;
        }
    }
}

/**
 * @brief Cleanup between tests
 */
void cleanup_test() {
    client_socket.close();
    vTaskDelay(pdMS_TO_TICKS(100));  // Short delay between tests
}

// ===================================================================================
// CATEGORY 1: BASIC CONNECTIVITY TESTS ("HAPPY PATH")
// ===================================================================================

/**
 * @brief Test 1.1: Client Connection, Send/Receive, Close
 */
void test_1_1_basic_client_lifecycle() {
    const char* test_name = "1.1 Basic Client Lifecycle";
    printf("\n--- Running %s ---\n", test_name);
    
    auto config = create_client_config(TEST_SERVER_IP, TEST_SERVER_PORT_1);
    
    // Step 1: Connect with retry
    int result = connect_with_retry(config, test_name);
    if (result != 0) {
        g_results.record_fail(test_name, "Failed to connect after retries");
        return;
    }
    
    // Step 2: Send data
    const char* test_msg = "Hello";
    result = client_socket.send(reinterpret_cast<const uint8_t*>(test_msg), strlen(test_msg));
    if (result != static_cast<int>(strlen(test_msg))) {
        g_results.record_fail(test_name, "Failed to send data");
        return;
    }
    printf("Sent: %s\n", test_msg);
    
    // Step 3: Receive echo
    uint8_t recv_buffer[64];
    result = client_socket.recv(recv_buffer, sizeof(recv_buffer), TEST_RECV_TIMEOUT);
    if (result != static_cast<int>(strlen(test_msg))) {
        g_results.record_fail(test_name, "Failed to receive expected data size");
        return;
    }
    
    recv_buffer[result] = '\0';  // Null terminate for comparison
    if (strcmp(reinterpret_cast<const char*>(recv_buffer), test_msg) != 0) {
        g_results.record_fail(test_name, "Received data doesn't match sent data");
        return;
    }
    printf("Received echo: %s\n", reinterpret_cast<const char*>(recv_buffer));
    
    // Step 4: Close
    client_socket.close();
    printf("Connection closed\n");
    
    g_results.record_pass(test_name);
}

/**
 * @brief Test 1.3: Client with DHCP
 */
void test_1_3_client_with_dhcp() {
    const char* test_name = "1.3 Client with DHCP";
    printf("\n--- Running %s ---\n", test_name);
    
    auto config = create_client_config(TEST_SERVER_IP, TEST_SERVER_PORT_1, true);  // DHCP enabled
    
    // Connect with DHCP
    int result = connect_with_retry(config, test_name);
    if (result != 0) {
        g_results.record_fail(test_name, "Failed to connect with DHCP after retries");
        return;
    }
    printf("Connected with DHCP\n");
    
    // Validate communication
    const char* test_msg = "DHCP_TEST";
    result = client_socket.send(reinterpret_cast<const uint8_t*>(test_msg), strlen(test_msg));
    if (result != static_cast<int>(strlen(test_msg))) {
        g_results.record_fail(test_name, "Failed to send with DHCP connection");
        return;
    }
    
    uint8_t recv_buffer[64];
    result = client_socket.recv(recv_buffer, sizeof(recv_buffer), TEST_RECV_TIMEOUT);
    if (result != static_cast<int>(strlen(test_msg))) {
        g_results.record_fail(test_name, "Failed to receive with DHCP connection");
        return;
    }
    
    client_socket.close();
    g_results.record_pass(test_name);
}

// ===================================================================================
// CATEGORY 2: PERFORMANCE AND LOAD TESTS
// ===================================================================================

/**
 * @brief Test 2.1: Large Data Transmission
 */
void test_2_1_large_data_send() {
    const char* test_name = "2.1 Large Data Send";
    printf("\n--- Running %s ---\n", test_name);
    
    purge_rx_buffer();
    
    auto config = create_client_config(TEST_SERVER_IP, TEST_SERVER_PORT_1);
    
    int result = connect_with_retry(config, test_name);
    if (result != 0) {
        g_results.record_fail(test_name, "Failed to connect after retries");
        return;
    }
    
    // Create large buffer with test pattern
    uint8_t large_buffer[LARGE_DATA_SIZE];
    for (size_t i = 0; i < LARGE_DATA_SIZE; i++) {
        large_buffer[i] = static_cast<uint8_t>((i % 256));  // Repeating pattern 0-255
    }
    
    // Send in chunks (driver's internal buffer might be smaller)
    size_t total_sent = 0;
    const size_t chunk_size = 256;  // Send in 256-byte chunks
    
    printf("Sending %zu bytes in chunks of %zu...\n", LARGE_DATA_SIZE, chunk_size);
    
    for (size_t offset = 0; offset < LARGE_DATA_SIZE; offset += chunk_size) {
        size_t this_chunk = std::min(chunk_size, LARGE_DATA_SIZE - offset);
        
        result = client_socket.send(large_buffer + offset, this_chunk);
        if (result != static_cast<int>(this_chunk)) {
            g_results.record_fail(test_name, "Failed to send chunk");
            return;
        }
        total_sent += result;
        printf("Sent chunk: %zu bytes (total: %zu)\n", this_chunk, total_sent);
    }
    
    if (total_sent != LARGE_DATA_SIZE) {
        g_results.record_fail(test_name, "Total sent doesn't match expected");
        return;
    }
    
    // CRITICAL: Drain the echo response to prevent contamination of next test
    printf("Draining echo response...\n");
    uint8_t tmp[256];
    int total_echo = 0;
    int r;
    do {
        r = client_socket.recv(tmp, sizeof(tmp), pdMS_TO_TICKS(500));
        if (r > 0) {
            total_echo += r;
        }
    } while (r > 0);
    printf("Echo drained: %d bytes total\n", total_echo);
    
    client_socket.close();
    g_results.record_pass(test_name);
}

/**
 * @brief Test 2.2: Large Data Reception
 */
void test_2_2_large_data_recv() {
    const char* test_name = "2.2 Large Data Receive";
    printf("\n--- Running %s ---\n", test_name);

    
    auto config = create_client_config(TEST_SERVER_IP, TEST_SERVER_PORT_1);
    
    int result = connect_with_retry(config, test_name);
    if (result != 0) {
        g_results.record_fail(test_name, "Failed to connect after retries");
        return;
    }

    
    // CRITICAL: Purge any leftover data from previous tests
    purge_rx_buffer();
    
    // Send START command to trigger large data response from server
    const char* start_cmd = "START";
    printf("DEBUG: Sending START command (%zu bytes)\n", strlen(start_cmd));
    result = client_socket.send(reinterpret_cast<const uint8_t*>(start_cmd), strlen(start_cmd));
    if (result != static_cast<int>(strlen(start_cmd))) {
        g_results.record_fail(test_name, "Failed to send START command");
        return;
    }
    printf("Sent START command (sent %d bytes)\n", result);
    
    // Receive both echo + large data mixed together
    size_t total_received = 0;
    size_t echo_bytes_to_skip = 0;  // Skip "START" echo
    uint8_t recv_buffer[256];  // Small buffer to force multiple reads
    
    printf("Receiving large data in chunks...\n");
    
    while (total_received < LARGE_DATA_SIZE) {
        result = client_socket.recv(recv_buffer, sizeof(recv_buffer), TEST_RECV_TIMEOUT);
        if (result <= 0) {
            // If we're close to the target, try one more time with longer timeout
            if (total_received >= LARGE_DATA_SIZE - 10) {
                printf("Close to target (%zu/%zu), trying longer timeout...\n", total_received, LARGE_DATA_SIZE);
                result = client_socket.recv(recv_buffer, sizeof(recv_buffer), TEST_RECV_TIMEOUT * 2);
            }
            
            if (result <= 0) {
                char error_msg[64];
                snprintf(error_msg, sizeof(error_msg), "Failed to receive data chunk at %zu/%zu bytes", total_received, LARGE_DATA_SIZE);
                g_results.record_fail(test_name, error_msg);
                return;
            }
        }
        
        // Skip echo bytes if needed
        int data_start = 0;
        int actual_data = result;
        
        if (echo_bytes_to_skip > 0) {
            int skip_this_chunk = std::min(static_cast<size_t>(result), echo_bytes_to_skip);
            data_start = skip_this_chunk;
            actual_data = result - skip_this_chunk;
            echo_bytes_to_skip -= skip_this_chunk;
            printf("Skipped %d echo bytes, %zu remaining to skip\n", skip_this_chunk, echo_bytes_to_skip);
        }
        
        total_received += actual_data;
        printf("Received chunk: %d bytes (%d data after echo skip, total: %zu)\n", result, actual_data, total_received);
        
        // Debug: Show actual data bytes (after echo skip)
        if (actual_data > 0) {
            printf("  Data bytes: ");
            for (int i = data_start; i < std::min(result, data_start + 8); i++) {
                printf("0x%02X ", recv_buffer[i]);
            }
            if (actual_data > 8) {
                printf("... Last bytes: ");
                for (int i = std::max(data_start, result - 4); i < result; i++) {
                    printf("0x%02X ", recv_buffer[i]);
                }
            }
            printf("\n");
            
            // Check if we're getting unexpected data beyond target
            if (total_received > LARGE_DATA_SIZE) {
                printf("*** WARNING: Receiving more than expected! ***\n");
                printf("*** This chunk content: ");
                for (int i = data_start; i < result; i++) {
                    printf("%02X", recv_buffer[i]);
                    if (i > data_start + 32) { printf("..."); break; }
                }
                printf(" ***\n");
            }
        }
        
        // Small delay to let CH9120 process the data properly
        vTaskDelay(pdMS_TO_TICKS(BURST_DELAY_MS));
        
        // Prevent infinite loop in case of server issues
        if (total_received > LARGE_DATA_SIZE * 2) {
            g_results.record_fail(test_name, "Received too much data");
            return;
        }
    }
    
    if (total_received != LARGE_DATA_SIZE) {
        char error_msg[64];
        snprintf(error_msg, sizeof(error_msg), "Expected %zu, got %zu bytes", LARGE_DATA_SIZE, total_received);
        g_results.record_fail(test_name, error_msg);
        return;
    }
    
    printf("Perfect! Received exactly %zu bytes\n", total_received);
    
    client_socket.close();
    g_results.record_pass(test_name);
}

/**
 * @brief Test 2.3: Fast Ping-Pong Communication
 */
void test_2_3_ping_pong() {
    const char* test_name = "2.3 Ping-Pong Performance";
    printf("\n--- Running %s ---\n", test_name);
    
    auto config = create_client_config(TEST_SERVER_IP, TEST_SERVER_PORT_1);
    
    int result = connect_with_retry(config, test_name);
    if (result != 0) {
        g_results.record_fail(test_name, "Failed to connect after retries");
        return;
    }
    
    printf("Starting %zu ping-pong iterations...\n", PING_PONG_ITERATIONS);
    
    for (size_t i = 0; i < PING_PONG_ITERATIONS; i++) {
        const char* ping_msg = "ping";
        
        // Send ping
        result = client_socket.send(reinterpret_cast<const uint8_t*>(ping_msg), strlen(ping_msg));
        if (result != static_cast<int>(strlen(ping_msg))) {
            g_results.record_fail(test_name, "Failed to send ping");
            return;
        }
        
        // Receive pong
        uint8_t recv_buffer[16];
        result = client_socket.recv(recv_buffer, sizeof(recv_buffer), TEST_RECV_TIMEOUT);
        if (result != static_cast<int>(strlen(ping_msg))) {
            g_results.record_fail(test_name, "Failed to receive pong");
            return;
        }
        
        // Verify echo
        recv_buffer[result] = '\0';
        if (strcmp(reinterpret_cast<const char*>(recv_buffer), ping_msg) != 0) {
            g_results.record_fail(test_name, "Ping-pong data mismatch");
            return;
        }
        
        if ((i + 1) % 25 == 0) {
            printf("Completed %zu iterations\n", i + 1);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms delay between iterations
    }
    
    client_socket.close();
    g_results.record_pass(test_name);
}

// ===================================================================================
// CATEGORY 3: ROBUSTNESS AND ERROR HANDLING TESTS
// ===================================================================================

/**
 * @brief Test 3.1: Connection Timeout
 */
void test_3_1_connection_timeout() {
    const char* test_name = "3.1 Connection Timeout";
    printf("\n--- Running %s ---\n", test_name);
    
    // Use non-existent server IP
    uint8_t fake_server_ip[4] = {192, 168, 99, 99};
    auto config = create_client_config(fake_server_ip, TEST_SERVER_PORT_1);
    
    printf("Attempting connection to non-existent server (should timeout)...\n");
    
    int result = client_socket.connect(config, TEST_SHORT_TIMEOUT);
    
    // Should fail with timeout error
    if (result == 0) {
        g_results.record_fail(test_name, "Connection unexpectedly succeeded");
        return;
    }
    
    // Check for expected timeout error (ETIMEDOUT = -116)
    if (result != -116) {  // -ETIMEDOUT
        printf("Got error code: %d (expected -116 for ETIMEDOUT)\n", result);
        g_results.record_fail(test_name, "Unexpected error code");
        return;
    }
    
    printf("Connection correctly timed out with error: %d\n", result);
    g_results.record_pass(test_name);
}

/**
 * @brief Test 3.2: Server Closes Connection Abruptly
 * Note: This test requires the mock server to be run with --mode disconnect
 */
void test_3_2_server_disconnect() {
    const char* test_name = "3.2 Server Disconnect";
    printf("\n--- Running %s ---\n", test_name);
    printf("NOTE: This test requires mock server running with --mode disconnect\n");
    
    auto config = create_client_config(TEST_SERVER_IP, TEST_SERVER_PORT_1);
    
    // The disconnect server will close immediately after accept
    int result = client_socket.connect(config, TEST_CONNECT_TIMEOUT);
    if (result != 0) {
        g_results.record_fail(test_name, "Failed initial connection");
        return;
    }
    
    // Give a moment for the server to close
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Try to send data (should fail)
    const char* test_msg = "data";
    result = client_socket.send(reinterpret_cast<const uint8_t*>(test_msg), strlen(test_msg));
    
    // Should fail with connection error
    if (result >= 0) {
        printf("WARNING: Send succeeded even though server disconnected\n");
        // Try recv to see if connection is really dead
        uint8_t recv_buffer[16];
        result = client_socket.recv(recv_buffer, sizeof(recv_buffer), pdMS_TO_TICKS(1000));
        if (result != 0) {
            g_results.record_fail(test_name, "Connection appears to still be active");
            return;
        }
    }
    
    printf("Send/recv correctly failed after server disconnect\n");
    client_socket.close();
    g_results.record_pass(test_name);
}

// ===================================================================================
// CATEGORY 4: LIFECYCLE AND RECONFIGURATION TESTS
// ===================================================================================

/**
 * @brief Test 4.1: Multiple Connect/Disconnect Cycles
 */
void test_4_1_connection_cycles() {
    const char* test_name = "4.1 Connection Lifecycle";
    printf("\n--- Running %s ---\n", test_name);
    
    auto config = create_client_config(TEST_SERVER_IP, TEST_SERVER_PORT_1);
    
    printf("Running %zu connect/disconnect cycles...\n", LIFECYCLE_ITERATIONS);
    
    for (size_t i = 0; i < LIFECYCLE_ITERATIONS; i++) {
        // Connect (first iteration uses retry, others use normal timeout)
        int result;
        if (i == 0) {
            result = connect_with_retry(config, test_name);
        } else {
            result = client_socket.connect(config, TEST_CONNECT_TIMEOUT);
        }
        if (result != 0) {
            printf("Failed to connect on iteration %zu\n", i + 1);
            g_results.record_fail(test_name, "Connection failed during cycles");
            return;
        }
        
        // Verify connected state
        if (!client_socket.isConnected()) {
            g_results.record_fail(test_name, "isConnected() returned false after connect");
            return;
        }
        
        // Close
        client_socket.close();
        
        // Verify disconnected state
        if (client_socket.isConnected()) {
            g_results.record_fail(test_name, "isConnected() returned true after close");
            return;
        }
        
        if ((i + 1) % 3 == 0) {
            printf("Completed %zu cycles\n", i + 1);
        }
        
        vTaskDelay(pdMS_TO_TICKS(200));  // Delay between cycles
    }
    
    g_results.record_pass(test_name);
}

/**
 * @brief Test 4.2: Configuration Change (Hot Reconfiguration)
 */
void test_4_2_hot_reconfiguration() {
    const char* test_name = "4.2 Hot Reconfiguration";
    printf("\n--- Running %s ---\n", test_name);
    printf("NOTE: This test requires two mock servers on ports %d and %d\n", 
           TEST_SERVER_PORT_1, TEST_SERVER_PORT_2);
    
    // Connect to first server
    auto config1 = create_client_config(TEST_SERVER_IP, TEST_SERVER_PORT_1);
    int result = connect_with_retry(config1, test_name);
    if (result != 0) {
        g_results.record_fail(test_name, "Failed to connect to first server after retries");
        return;
    }
    
    // Test communication with first server
    const char* msg1 = "to server 1";
    result = client_socket.send(reinterpret_cast<const uint8_t*>(msg1), strlen(msg1));
    if (result != static_cast<int>(strlen(msg1))) {
        g_results.record_fail(test_name, "Failed to send to first server");
        return;
    }
    
    uint8_t recv_buffer[64];
    result = client_socket.recv(recv_buffer, sizeof(recv_buffer), TEST_RECV_TIMEOUT);
    if (result != static_cast<int>(strlen(msg1))) {
        g_results.record_fail(test_name, "Failed to receive from first server");
        return;
    }
    printf("Successfully communicated with first server\n");
    
    // Close and reconfigure
    client_socket.close();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Connect to second server
    auto config2 = create_client_config(TEST_SERVER_IP, TEST_SERVER_PORT_2);
    result = client_socket.connect(config2, TEST_CONNECT_TIMEOUT);
    if (result != 0) {
        g_results.record_fail(test_name, "Failed to connect to second server");
        return;
    }
    
    // Test communication with second server
    const char* msg2 = "to server 2";
    result = client_socket.send(reinterpret_cast<const uint8_t*>(msg2), strlen(msg2));
    if (result != static_cast<int>(strlen(msg2))) {
        g_results.record_fail(test_name, "Failed to send to second server");
        return;
    }
    
    result = client_socket.recv(recv_buffer, sizeof(recv_buffer), TEST_RECV_TIMEOUT);
    if (result != static_cast<int>(strlen(msg2))) {
        g_results.record_fail(test_name, "Failed to receive from second server");
        return;
    }
    printf("Successfully communicated with second server\n");
    
    client_socket.close();
    g_results.record_pass(test_name);
}

/**
 * @brief Test 4.3: Duplicate Connection Attempt (EISCONN)
 */
void test_4_3_duplicate_connection() {
    const char* test_name = "4.3 Duplicate Connection";
    printf("\n--- Running %s ---\n", test_name);
    
    auto config = create_client_config(TEST_SERVER_IP, TEST_SERVER_PORT_1);
    
    // First connection should succeed
    int result = connect_with_retry(config, test_name);
    if (result != 0) {
        g_results.record_fail(test_name, "First connection failed after retries");
        return;
    }
    printf("First connection successful\n");
    
    // Second connection with same config should return EISCONN (-127)
    result = client_socket.connect(config, TEST_CONNECT_TIMEOUT);
    if (result != -127) {  // -EISCONN
        printf("Expected EISCONN (-127), got: %d\n", result);
        g_results.record_fail(test_name, "Duplicate connection didn't return EISCONN");
        client_socket.close();
        return;
    }
    
    printf("Duplicate connection correctly returned EISCONN (%d)\n", result);
    client_socket.close();
    g_results.record_pass(test_name);
}

// ===================================================================================
// TEST RUNNER TASK
// ===================================================================================

void test_runner_task(void* params) {
    printf("\n============================================================\n");
    printf("CH9120Driver TCP Client Test Suite\n");
    printf("============================================================\n");
    
    // Initialize driver
    if (!setup_driver()) {
        printf("FATAL: Driver initialization failed\n");
        vTaskDelete(NULL);
        return;
    }
    
    printf("Driver initialized successfully\n");
    printf("Test server should be at %d.%d.%d.%d:%d\n", 
           TEST_SERVER_IP[0], TEST_SERVER_IP[1], TEST_SERVER_IP[2], TEST_SERVER_IP[3], 
           TEST_SERVER_PORT_1);
    
    // Run all tests
    printf("\n🚀 Starting test execution...\n");
    
    // Category 1: Basic connectivity
    test_1_1_basic_client_lifecycle();
    cleanup_test();
    
    test_1_3_client_with_dhcp();
    cleanup_test();
    
    // Category 2: Performance tests
    test_2_1_large_data_send();
    cleanup_test();
    
    test_2_2_large_data_recv();
    cleanup_test();
    
    test_2_3_ping_pong();
    cleanup_test();
    
    // Category 3: Robustness tests
    test_3_1_connection_timeout();
    cleanup_test();
    
    // Note: test_3_2 requires special server mode - uncomment if testing
    // test_3_2_server_disconnect();
    // cleanup_test();
    
    // Category 4: Lifecycle tests
    test_4_1_connection_cycles();
    cleanup_test();
    
    test_4_2_hot_reconfiguration();
    cleanup_test();
    
    test_4_3_duplicate_connection();
    cleanup_test();
    
    // Print final results
    g_results.print_summary();
    
    printf("\n🏁 Test execution completed.\n");
    printf("============================================================\n");
    
    // Keep task alive
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

// ===================================================================================
// MAIN FUNCTION
// ===================================================================================

int main() {
    stdio_init_all();
    
    // Wait for USB to initialize
    for (int i = 0; i < 50 && !stdio_usb_connected(); i++) {
        sleep_ms(100);
    }
    
    printf("\n\n=== CH9120Driver Test Client Starting ===\n");
    printf("USB Console initialized\n");
    printf("Creating test runner task...\n");
    
    // Create test runner task
    if (xTaskCreate(test_runner_task, "TestRunner", 8192, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        printf("FATAL: Failed to create test runner task!\n");
        while(1) { sleep_ms(1000); }
    }
    
    printf("Starting FreeRTOS scheduler...\n");
    
    // Start FreeRTOS scheduler
    vTaskStartScheduler();
    
    // Should never reach here
    printf("ERROR: vTaskStartScheduler() returned!\n");
    while(1) { sleep_ms(1000); }
    
    return 0;
}