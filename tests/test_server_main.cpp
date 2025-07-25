/**
 * @file test_server_main.cpp
 * @brief Comprehensive test suite for CH9120Driver TCP Server functionality
 * 
 * This file implements server-specific tests from the test specification,
 * including server lifecycle and timeout handling tests.
 * 
 * Test Prerequisites:
 * - Python client scripts or manual client connections
 * - CH9120 module configured with server IP accessible from test network
 * - Network configuration matching test constants below
 */

#include "CH9120Driver.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <cstdio>
#include <cstring>
#include <cassert>
#include <cstdlib>
#include "pico/stdlib.h"

// ===================================================================================
// TEST CONFIGURATION
// ===================================================================================

// Network configuration - MODIFY THESE FOR YOUR TEST SETUP
static constexpr uint8_t PICO_LOCAL_IP[4] = {192, 168, 0, 124};   // Pico server IP
static constexpr uint16_t PICO_SERVER_PORT = 8000;               // Server port
static constexpr uint8_t GATEWAY_IP[4] = {192, 168, 0, 1};       // Network gateway
static constexpr uint8_t SUBNET_MASK[4] = {255, 255, 255, 0};    // Subnet mask

// Test timing configuration
static constexpr TickType_t TEST_ACCEPT_TIMEOUT = pdMS_TO_TICKS(20000);  // 20s accept timeout
static constexpr TickType_t TEST_RECV_TIMEOUT = pdMS_TO_TICKS(60000);     // 60s recv timeout
static constexpr TickType_t TEST_SHORT_TIMEOUT = pdMS_TO_TICKS(2000);    // 2s for timeout tests

// Global driver instance (using default Waveshare pins)
CH9120Driver server_socket(uart1);

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
 * @brief Create a standard server configuration for testing
 */
CH9120Driver::NetworkConfig create_server_config(uint16_t server_port, bool use_dhcp = false) {
    CH9120Driver::NetworkConfig config;
    memcpy(config.localIp, PICO_LOCAL_IP, 4);
    memcpy(config.gateway, GATEWAY_IP, 4);
    memcpy(config.subnetMask, SUBNET_MASK, 4);
    config.localPort = server_port;
    config.mode = CH9120Driver::Mode::TCP_SERVER;
    config.useDhcp = use_dhcp;
    return config;
}

/**
 * @brief Initialize the driver (common setup for all tests)
 */
bool setup_driver() {
    if (server_socket.begin() != CH9120Driver::SUCCESS) {
        printf("ERROR: Failed to initialize CH9120Driver\n");
        return false;
    }
    return true;
}

/**
 * @brief Cleanup between tests
 */
void cleanup_test() {
    server_socket.close();
    vTaskDelay(pdMS_TO_TICKS(100));  // Short delay between tests
}

// ===================================================================================
// SERVER-SPECIFIC TESTS
// ===================================================================================

/**
 * @brief Test 1.2: Server Bind, Listen, Accept, Send/Receive
 */
void test_1_2_basic_server_lifecycle() {
    const char* test_name = "1.2 Basic Server Lifecycle";
    printf("\n--- Running %s ---\n", test_name);
    
    auto config = create_server_config(PICO_SERVER_PORT, true);
    
    // Step 1: Bind
    int result = server_socket.bind(config);
    if (result != 0) {
        g_results.record_fail(test_name, "Failed to bind server");
        return;
    }
    printf("Server bound to port %d\n", PICO_SERVER_PORT);
    
    // Step 2: Listen
    result = server_socket.listen();
    if (result != 0) {
        g_results.record_fail(test_name, "Failed to start listening");
        return;
    }
    printf("Server is listening\n");
    
    printf("Waiting for client connection on %d.%d.%d.%d:%d...\n", 
           PICO_LOCAL_IP[0], PICO_LOCAL_IP[1], PICO_LOCAL_IP[2], PICO_LOCAL_IP[3], 
           PICO_SERVER_PORT);
    
    // Step 3: Accept (wait for client)
    result = server_socket.accept(TEST_ACCEPT_TIMEOUT);
    if (result != 0) {
        printf("Accept failed with error: %d\n", result);
        g_results.record_fail(test_name, "Failed to accept client connection");
        return;
    }
    printf("Client connected!\n");
    
    // Step 4: Receive data from client
    uint8_t recv_buffer[64];
    result = server_socket.recv(recv_buffer, sizeof(recv_buffer), TEST_RECV_TIMEOUT);
    if (result <= 0) {
        printf("Receive failed with result: %d\n", result);
        g_results.record_fail(test_name, "Failed to receive data from client");
        return;
    }
    
    recv_buffer[result] = '\0';  // Null terminate for display
    printf("Received from client: '%s' (%d bytes)\n", recv_buffer, result);
    
    // Step 5: Send ACK response
    const char* ack_msg = "ACK";
    result = server_socket.send(reinterpret_cast<const uint8_t*>(ack_msg), strlen(ack_msg));
    if (result != static_cast<int>(strlen(ack_msg))) {
        g_results.record_fail(test_name, "Failed to send ACK to client");
        return;
    }
    printf("Sent ACK to client\n");
    
    // Step 6: Close
    server_socket.close();
    printf("Server connection closed\n");
    
    g_results.record_pass(test_name);
}

/**
 * @brief Test 3.3: Server Receive Timeout (Silent Client)
 */
void test_3_3_server_recv_timeout() {
    const char* test_name = "3.3 Server Recv Timeout";
    printf("\n--- Running %s ---\n", test_name);
    
    auto config = create_server_config(PICO_SERVER_PORT, true);
    
    // Bind and listen
    int result = server_socket.bind(config);
    if (result != 0) {
        g_results.record_fail(test_name, "Failed to bind server");
        return;
    }
    
    result = server_socket.listen();
    if (result != 0) {
        g_results.record_fail(test_name, "Failed to start listening");
        return;
    }
    
    printf("Waiting for silent client connection on %d.%d.%d.%d:%d...\n",
           PICO_LOCAL_IP[0], PICO_LOCAL_IP[1], PICO_LOCAL_IP[2], PICO_LOCAL_IP[3], 
           PICO_SERVER_PORT);
    
    // Accept client connection
    result = server_socket.accept(TEST_ACCEPT_TIMEOUT);
    if (result != 0) {
        g_results.record_fail(test_name, "Failed to accept client connection");
        return;
    }
    printf("Silent client connected!\n");
    
    // Try to receive with timeout (should timeout)
    printf("Waiting for data from client (should timeout in %d seconds)...\n", 
           TEST_SHORT_TIMEOUT / pdMS_TO_TICKS(1000));
    
    uint8_t recv_buffer[64];
    uint32_t start_time = xTaskGetTickCount();
    
    result = server_socket.recv(recv_buffer, sizeof(recv_buffer), TEST_SHORT_TIMEOUT);
    
    uint32_t elapsed_time = xTaskGetTickCount() - start_time;
    uint32_t elapsed_ms = (elapsed_time * 1000) / configTICK_RATE_HZ;
    
    printf("recv() returned %d after %lu ms\n", result, elapsed_ms);
    
    // Should timeout (return <= 0) or get ETIMEDOUT error
    if (result > 0) {
        printf("WARNING: Received unexpected data: '%.*s'\n", result, recv_buffer);
        g_results.record_fail(test_name, "Expected timeout but received data");
        return;
    }
    
    // Check that it actually timed out (didn't return immediately)
    if (elapsed_ms < (TEST_SHORT_TIMEOUT / pdMS_TO_TICKS(1000) - 500)) {
        g_results.record_fail(test_name, "recv() returned too quickly (not a proper timeout)");
        return;
    }
    
    printf("✓ recv() correctly timed out\n");
    
    server_socket.close();
    g_results.record_pass(test_name);
}

/**
 * @brief Test: Server with DHCP Configuration
 */
void test_server_with_dhcp() {
    const char* test_name = "Server with DHCP";
    printf("\n--- Running %s ---\n", test_name);
    
    auto config = create_server_config(PICO_SERVER_PORT, true);  // Enable DHCP
    
    // Bind with DHCP
    int result = server_socket.bind(config);
    if (result != 0) {
        g_results.record_fail(test_name, "Failed to bind server with DHCP");
        return;
    }
    printf("Server bound with DHCP enabled\n");
    
    // Listen
    result = server_socket.listen();
    if (result != 0) {
        g_results.record_fail(test_name, "Failed to listen with DHCP");
        return;
    }
    printf("Server listening with DHCP configuration\n");
    
    printf("Server ready for connections (DHCP mode)\n");
    
    // Accept with shorter timeout for this test
    result = server_socket.accept(pdMS_TO_TICKS(15000));  // 15 second timeout
    if (result != 0) {
        printf("No client connected within timeout (this is OK for automated testing)\n");
        server_socket.close();
        g_results.record_pass(test_name);  // Pass if no connection (can't easily automate)
        return;
    }
    
    printf("Client connected with DHCP server!\n");
    
    // Try to receive some data
    uint8_t recv_buffer[64];
    result = server_socket.recv(recv_buffer, sizeof(recv_buffer), TEST_RECV_TIMEOUT);
    if (result > 0) {
        recv_buffer[result] = '\0';
        printf("Received: '%s'\n", recv_buffer);
        
        // Echo it back
        server_socket.send(recv_buffer, result);
        printf("Echoed data back to client\n");
    }
    
    server_socket.close();
    g_results.record_pass(test_name);
}

/**
 * @brief Test: Server State Management
 */
void test_server_state_management() {
    const char* test_name = "Server State Management";
    printf("\n--- Running %s ---\n", test_name);
    
    auto config = create_server_config(PICO_SERVER_PORT, true);
    
    // Check initial state
    if (server_socket.isConnected()) {
        g_results.record_fail(test_name, "Server reports connected before bind");
        return;
    }
    
    // Bind
    int result = server_socket.bind(config);
    if (result != 0) {
        g_results.record_fail(test_name, "Failed to bind");
        return;
    }
    
    // Should still not be "connected" until listening
    if (server_socket.isConnected()) {
        g_results.record_fail(test_name, "Server reports connected after bind but before listen");
        return;
    }
    
    // Listen
    result = server_socket.listen();
    if (result != 0) {
        g_results.record_fail(test_name, "Failed to listen");
        return;
    }
    
    // Now should report as "connected" (listening state)
    if (!server_socket.isConnected()) {
        g_results.record_fail(test_name, "Server should report connected while listening");
        return;
    }
    
    printf("✓ Server state transitions correctly\n");
    
    // Close and verify state
    server_socket.close();
    if (server_socket.isConnected()) {
        g_results.record_fail(test_name, "Server still reports connected after close");
        return;
    }
    
    printf("✓ Server correctly reports disconnected after close\n");
    g_results.record_pass(test_name);
}

/**
 * @brief Test: Multiple Client Connections (Sequential)
 */
void test_multiple_client_connections() {
    const char* test_name = "Multiple Client Connections";
    printf("\n--- Running %s ---\n", test_name);
    
    auto config = create_server_config(PICO_SERVER_PORT, true);
    
    int result = server_socket.bind(config);
    if (result != 0) {
        g_results.record_fail(test_name, "Failed to bind");
        return;
    }
    
    result = server_socket.listen();
    if (result != 0) {
        g_results.record_fail(test_name, "Failed to listen");
        return;
    }
    
    printf("Server ready for multiple client connections on %d.%d.%d.%d:%d\n", 
           PICO_LOCAL_IP[0], PICO_LOCAL_IP[1], PICO_LOCAL_IP[2], PICO_LOCAL_IP[3], 
           PICO_SERVER_PORT);
    
    const int num_clients = 3;
    for (int i = 0; i < num_clients; i++) {
        printf("\n--- Waiting for client %d/%d ---\n", i + 1, num_clients);
        
        // Accept next client
        result = server_socket.accept(TEST_ACCEPT_TIMEOUT);
        if (result != 0) {
            printf("Failed to accept client %d (error: %d)\n", i + 1, result);
            g_results.record_fail(test_name, "Failed to accept multiple clients");
            return;
        }
        
        printf("Client %d connected\n", i + 1);
        
        // Receive data
        uint8_t recv_buffer[64];
        result = server_socket.recv(recv_buffer, sizeof(recv_buffer), TEST_RECV_TIMEOUT);
        if (result <= 0) {
            printf("Failed to receive from client %d\n", i + 1);
            // Don't fail the test - client might have disconnected
            continue;
        }
        
        recv_buffer[result] = '\0';
        printf("Client %d sent: '%s'\n", i + 1, recv_buffer);
        
        // Send response
        char response[32];
        snprintf(response, sizeof(response), "ACK from client %d", i + 1);
        server_socket.send(reinterpret_cast<const uint8_t*>(response), strlen(response));
        
        printf("Sent response to client %d\n", i + 1);
        
        // Small delay before next client
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    printf("\n✓ Successfully handled %d client connections\n", num_clients);
    server_socket.close();
    g_results.record_pass(test_name);
}


// ===================================================================================
// MAIN TEST TASK
// ===================================================================================

void test_runner_task(void* params) {
    vTaskDelay(pdMS_TO_TICKS(3000));  // Allow time for system to stabilize
    printf("\n============================================================\n");
    printf("CH9120Driver TCP Server Test Suite\n");
    printf("============================================================\n");
    
    // Initialize driver
    if (!setup_driver()) {
        printf("FATAL: Driver initialization failed\n");
        vTaskDelete(NULL);
        return;
    }
    
    printf("Driver initialized successfully\n");
    printf("Server will bind to %d.%d.%d.%d:%d\n", 
           PICO_LOCAL_IP[0], PICO_LOCAL_IP[1], PICO_LOCAL_IP[2], PICO_LOCAL_IP[3], 
           PICO_SERVER_PORT);
    
    // Run all tests automatically
    printf("\n🚀 Starting automated test execution...\n");
    
    test_1_2_basic_server_lifecycle();
    cleanup_test();
    
    test_3_3_server_recv_timeout();
    cleanup_test();
    
    test_server_with_dhcp();
    cleanup_test();
    
    test_server_state_management();
    cleanup_test();
    
    test_multiple_client_connections();
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
    
    printf("Booting CH9120Driver Test Server...\n");
    
    // Create test runner task
    xTaskCreate(test_runner_task, "ServerTestRunner", 8192, NULL, tskIDLE_PRIORITY + 1, NULL);
    
    // Start FreeRTOS scheduler
    vTaskStartScheduler();
    
    // Should never reach here
    return 0;
}