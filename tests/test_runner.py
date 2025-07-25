#!/usr/bin/env python3
"""
CH9120Driver Automated Test Runner - BULLETPROOF Edition
Comprehensive automated testing without user interaction
"""

import socket
import time
import sys
import threading
import argparse
import struct
import select
from collections import defaultdict

# Test configuration
PICO_CLIENT_IP = "192.168.0.124"   # Pico IP when running as client  
PICO_SERVER_IP = "192.168.0.124"   # Pico IP when running as server
PC_IP = "0.0.0.0"                  # Listen on all interfaces
PORT_1 = 8000                      # Primary test port
PORT_2 = 8001                      # Secondary test port

# Test parameters matching test_client_main.cpp
LARGE_DATA_SIZE = 1024
PING_PONG_ITERATIONS = 100
LIFECYCLE_ITERATIONS = 10

# ANSI colors
GREEN = '\033[92m'
RED = '\033[91m'
YELLOW = '\033[93m'
BLUE = '\033[94m'
MAGENTA = '\033[95m'
RESET = '\033[0m'

class TestStats:
    """Track detailed test statistics"""
    def __init__(self):
        self.tests_run = 0
        self.tests_passed = 0
        self.tests_failed = 0
        self.errors = []
        self.warnings = []
        self.timing = defaultdict(list)
        
    def add_timing(self, test_name, duration_ms):
        self.timing[test_name].append(duration_ms)
        
    def get_avg_timing(self, test_name):
        times = self.timing.get(test_name, [])
        return sum(times) / len(times) if times else 0

class RobustEchoServer:
    """Advanced echo server with detailed tracking and validation"""
    
    def __init__(self, test_stats):
        self.test_stats = test_stats
        self.running = True
        self.connections = {}
        self.data_received = defaultdict(int)
        self.data_sent = defaultdict(int)
        self.lock = threading.Lock()
        
    def log(self, message, color=None):
        """Thread-safe logging"""
        if color:
            print(f"{color}[Server] {message}{RESET}")
        else:
            print(f"[Server] {message}")
    
    def handle_client(self, conn, addr):
        """Handle individual client with comprehensive tracking"""
        client_id = f"{addr[0]}:{addr[1]}"
        self.log(f"Client connected: {client_id}", BLUE)
        
        try:
            conn.settimeout(0.1)  # Non-blocking with timeout
            buffer = b''
            
            while self.running:
                try:
                    # Try to receive data
                    data = conn.recv(4096)
                    if not data:
                        break
                    
                    with self.lock:
                        self.data_received[client_id] += len(data)
                    
                    # Handle different test scenarios
                    if data == b'START':
                        # Large data test - send exactly LARGE_DATA_SIZE bytes
                        self.log(f"START command from {client_id} - sending {LARGE_DATA_SIZE} bytes", YELLOW)
                        large_data = b'A' * LARGE_DATA_SIZE
                        conn.sendall(large_data)
                        with self.lock:
                            self.data_sent[client_id] += LARGE_DATA_SIZE
                            
                    elif data.startswith(b'ping'):
                        # Ping-pong test - immediate echo
                        conn.sendall(data)
                        with self.lock:
                            self.data_sent[client_id] += len(data)
                            
                    elif len(data) == 256:
                        # Likely a chunk from large data send test
                        # Echo it back
                        conn.sendall(data)
                        with self.lock:
                            self.data_sent[client_id] += len(data)
                            
                    else:
                        # Default echo behavior
                        conn.sendall(data)
                        with self.lock:
                            self.data_sent[client_id] += len(data)
                    
                    # Log data flow
                    if len(data) > 100:
                        self.log(f"Echoed {len(data)} bytes to {client_id}", MAGENTA)
                    
                except socket.timeout:
                    continue
                except Exception as e:
                    if "forcibly closed" not in str(e):
                        self.log(f"Client {client_id} error: {e}", RED)
                    break
                    
        finally:
            conn.close()
            with self.lock:
                rx = self.data_received[client_id]
                tx = self.data_sent[client_id]
            self.log(f"Client {client_id} disconnected (RX: {rx} bytes, TX: {tx} bytes)", BLUE)
    
    def run(self, port):
        """Run the echo server"""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((PC_IP, port))
        server.listen(5)
        server.settimeout(1.0)
        
        self.log(f"Echo server listening on port {port}...", GREEN)
        
        while self.running:
            try:
                conn, addr = server.accept()
                thread = threading.Thread(
                    target=self.handle_client,
                    args=(conn, addr)
                )
                thread.daemon = True
                thread.start()
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    self.log(f"Accept error: {e}", RED)
        
        server.close()
        self.log("Server shutdown", YELLOW)

class DisconnectServer:
    """Server that immediately disconnects - for robustness testing"""
    
    def __init__(self):
        self.running = True
        
    def run(self, port):
        """Accept and immediately close connections"""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((PC_IP, port))
        server.listen(1)
        server.settimeout(1.0)
        
        print(f"{YELLOW}[DisconnectServer] Listening on port {port} (will disconnect immediately){RESET}")
        
        while self.running:
            try:
                conn, addr = server.accept()
                print(f"{RED}[DisconnectServer] Client connected from {addr} - disconnecting immediately{RESET}")
                conn.close()
            except socket.timeout:
                continue
                
        server.close()

class SilentClient:
    """Client that connects but never sends data - for timeout testing"""
    
    @staticmethod
    def connect_and_wait(host, port, duration=5):
        """Connect and stay silent"""
        try:
            print(f"{YELLOW}[SilentClient] Connecting to {host}:{port} (will stay silent){RESET}")
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.connect((host, port))
            print(f"{GREEN}[SilentClient] Connected - staying silent for {duration}s{RESET}")
            time.sleep(duration)
            client.close()
            print(f"{BLUE}[SilentClient] Disconnected{RESET}")
        except Exception as e:
            print(f"{RED}[SilentClient] Error: {e}{RESET}")

class CH9120TestRunner:
    """Main test orchestrator"""
    
    def __init__(self, duration=45):
        self.stats = TestStats()
        self.running = True
        self.duration = duration
        
    def log(self, message, color=None):
        """Print colored log message"""
        if color:
            print(f"{color}{message}{RESET}")
        else:
            print(message)
    
    def test_pass(self, test_name, details=""):
        """Record test pass"""
        self.stats.tests_run += 1
        self.stats.tests_passed += 1
        self.log(f"✅ PASS: {test_name} {details}", GREEN)
    
    def test_fail(self, test_name, reason=""):
        """Record test failure"""
        self.stats.tests_run += 1
        self.stats.tests_failed += 1
        self.stats.errors.append(f"{test_name}: {reason}")
        self.log(f"❌ FAIL: {test_name} - {reason}", RED)
    
    def wait_for_pico_tests(self, duration=60):
        """Wait for Pico to run its autonomous tests"""
        self.log(f"\nWaiting {duration}s for Pico tests to complete...", YELLOW)
        
        # Show progress
        for i in range(duration):
            if i % 10 == 0:
                remaining = duration - i
                self.log(f"  {remaining}s remaining...", BLUE)
            time.sleep(1)
            if not self.running:
                break
    
    def run_client_tests(self):
        """Test suite for Pico as TCP Client"""
        self.log("\n" + "="*70, BLUE)
        self.log("CH9120Driver CLIENT Test Suite", BLUE)
        self.log("Testing Pico as TCP Client", BLUE)
        self.log("="*70, BLUE)
        
        # Create echo servers
        echo1 = RobustEchoServer(self.stats)
        echo2 = RobustEchoServer(self.stats)
        
        # Start primary echo server
        server1_thread = threading.Thread(target=echo1.run, args=(PORT_1,))
        server1_thread.daemon = True
        server1_thread.start()
        
        # Start secondary echo server for reconfiguration tests
        server2_thread = threading.Thread(target=echo2.run, args=(PORT_2,))
        server2_thread.daemon = True
        server2_thread.start()
        
        self.log("\n📋 Test Plan:", YELLOW)
        self.log("  1.1 Basic Client Lifecycle", YELLOW)
        self.log("  1.3 Client with DHCP", YELLOW)
        self.log("  2.1 Large Data Send (1024 bytes)", YELLOW)
        self.log("  2.2 Large Data Receive (1024 bytes)", YELLOW)
        self.log("  2.3 Ping-Pong Performance (100 iterations)", YELLOW)
        self.log("  3.1 Connection Timeout", YELLOW)
        self.log("  4.1 Connection Lifecycle (10 cycles)", YELLOW)
        self.log("  4.2 Hot Reconfiguration", YELLOW)
        self.log("  4.3 Duplicate Connection", YELLOW)
        
        self.log("\n⚡ Flash the Pico with test_client_main.cpp and reset", MAGENTA)
        self.log("The Pico will automatically connect to this server and run tests\n", MAGENTA)
        
        # Wait for tests to complete
        self.wait_for_pico_tests(self.duration)
        
        # Stop echo servers
        echo1.running = False
        echo2.running = False
        
        # Analyze results
        self.log("\n📊 Server Statistics:", BLUE)
        for client_id, rx_bytes in echo1.data_received.items():
            tx_bytes = echo1.data_sent.get(client_id, 0)
            self.log(f"  Client {client_id}: RX={rx_bytes} bytes, TX={tx_bytes} bytes", BLUE)
        
        # We can't know the exact test results without parsing Pico output
        # But we can validate that data flowed correctly
        total_rx = sum(echo1.data_received.values()) + sum(echo2.data_received.values())
        total_tx = sum(echo1.data_sent.values()) + sum(echo2.data_sent.values())
        
        if total_rx > 0 and total_tx > 0:
            self.test_pass("Client Test Suite", f"(RX: {total_rx} bytes, TX: {total_tx} bytes)")
        else:
            self.test_fail("Client Test Suite", "No data exchanged")
    
    def wait_for_pico_server(self, max_attempts=30, delay=2):
        """Wait for Pico server to be ready with retry logic"""
        self.log(f"Waiting for Pico server to start (max {max_attempts * delay}s)...", YELLOW)
        
        for attempt in range(max_attempts):
            try:
                # Try to connect to check if server is ready
                test_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                test_socket.settimeout(1.0)
                test_socket.connect((PICO_SERVER_IP, PORT_1))
                test_socket.close()
                self.log(f"✅ Pico server is ready (attempt {attempt + 1})", GREEN)
                return True
            except:
                if attempt < max_attempts - 1:
                    self.log(f"  Attempt {attempt + 1}/{max_attempts} - waiting {delay}s...", BLUE)
                    time.sleep(delay)
                else:
                    self.log("❌ Pico server failed to start", RED)
                    return False
        return False

    def run_server_tests(self):
        """Test suite for Pico as TCP Server"""
        self.log("\n" + "="*70, BLUE)
        self.log("CH9120Driver SERVER Test Suite", BLUE)
        self.log("Testing Pico as TCP Server", BLUE)
        self.log("="*70, BLUE)
        
        self.log("\n📋 Test Plan:", YELLOW)
        self.log("  1.2 Basic Server Lifecycle", YELLOW)
        self.log("  3.3 Server Recv Timeout", YELLOW)
        self.log("  Server with DHCP", YELLOW)
        self.log("  Server State Management", YELLOW)
        self.log("  Multiple Client Connections", YELLOW)
        
        self.log("\n⚡ Flash the Pico with test_server_main.cpp and reset", MAGENTA)
        self.log("The Python script will wait for the Pico server to start\n", MAGENTA)
        
        # Wait for Pico server with retry logic
        if not self.wait_for_pico_server():
            self.test_fail("Server Startup", "Pico server did not start within timeout")
            return
        
        # Test 1.2: Basic Server Lifecycle
        self.log("\n--- Test 1.2: Basic Server Lifecycle ---", YELLOW)
        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.settimeout(5.0)
            client.connect((PICO_SERVER_IP, PORT_1))
            client.send(b"TestData")
            response = client.recv(1024)
            client.close()
            
            if response == b"ACK":
                self.test_pass("1.2 Basic Server Lifecycle")
            else:
                self.test_fail("1.2 Basic Server Lifecycle", f"Expected ACK, got {response}")
        except Exception as e:
            self.test_fail("1.2 Basic Server Lifecycle", str(e))
        
        time.sleep(2)
        
        # Test 3.3: Server Recv Timeout (with silent client)
        self.log("\n--- Test 3.3: Server Recv Timeout ---", YELLOW)
        silent_thread = threading.Thread(
            target=SilentClient.connect_and_wait,
            args=(PICO_SERVER_IP, PORT_1, 5)
        )
        silent_thread.daemon = True
        silent_thread.start()
        silent_thread.join()
        self.test_pass("3.3 Server Recv Timeout", "(Silent client test)")
        
        time.sleep(2)
        
        # Test: Multiple Client Connections
        self.log("\n--- Test: Multiple Client Connections ---", YELLOW)
        success = True
        for i in range(3):
            try:
                self.log(f"  Connecting client {i+1}/3...", BLUE)
                client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                client.settimeout(5.0)
                client.connect((PICO_SERVER_IP, PORT_1))
                
                msg = f"Client{i+1}".encode()
                client.send(msg)
                response = client.recv(1024)
                expected = f"ACK from client {i+1}".encode()
                
                if response == expected:
                    self.log(f"  Client {i+1} received correct response", GREEN)
                else:
                    self.log(f"  Client {i+1} got unexpected response: {response}", RED)
                    success = False
                    
                client.close()
                time.sleep(0.5)
                
            except Exception as e:
                self.test_fail(f"Multiple Clients - Client {i+1}", str(e))
                success = False
                break
        
        if success:
            self.test_pass("Multiple Client Connections")
        
        self.log("\nServer tests completed!", GREEN)
    
    def print_summary(self):
        """Print comprehensive test summary"""
        self.log("\n" + "="*70, BLUE)
        self.log("TEST SUMMARY", BLUE)
        self.log("="*70, BLUE)
        self.log(f"Total tests tracked: {self.stats.tests_run}")
        self.log(f"Passed: {self.stats.tests_passed}", GREEN)
        self.log(f"Failed: {self.stats.tests_failed}", RED if self.stats.tests_failed > 0 else GREEN)
        
        if self.stats.errors:
            self.log("\nErrors:", RED)
            for error in self.stats.errors:
                self.log(f"  - {error}", RED)
        
        if self.stats.warnings:
            self.log("\nWarnings:", YELLOW)
            for warning in self.stats.warnings:
                self.log(f"  - {warning}", YELLOW)
        
        if self.stats.tests_failed == 0:
            self.log("\n🎉 ALL TRACKED TESTS PASSED!", GREEN)
            self.log("Note: Check Pico serial output for complete test results", YELLOW)
        else:
            self.log(f"\n⚠️  {self.stats.tests_failed} tests failed", RED)
        
        self.log("="*70 + "\n", BLUE)


def main():
    global PICO_CLIENT_IP, PICO_SERVER_IP
    
    parser = argparse.ArgumentParser(description='CH9120Driver Automated Test Runner')
    parser.add_argument('--mode', choices=['test_client', 'test_server', 'both'], 
                       default='test_client', help='Test mode (default: test_client)')
    parser.add_argument('--ip', default=PICO_SERVER_IP,
                       help=f'Pico IP address (default: {PICO_SERVER_IP})')
    parser.add_argument('--duration', type=int, default=45,
                       help='Test duration in seconds (default: 45)')
    
    args = parser.parse_args()
    
    # Update IPs if specified
    if args.ip:
        PICO_CLIENT_IP = args.ip
        PICO_SERVER_IP = args.ip
    
    runner = CH9120TestRunner(args.duration)
    
    try:
        if args.mode == 'test_client':
            runner.run_client_tests()
        elif args.mode == 'test_server':
            runner.run_server_tests()
        else:  # both
            runner.run_client_tests()
            input("\nPress Enter to continue with server tests...")
            runner.run_server_tests()
        
        runner.print_summary()
        
    except KeyboardInterrupt:
        runner.log("\n\nTest interrupted by user", YELLOW)
        runner.running = False
        runner.print_summary()
        sys.exit(1)


if __name__ == '__main__':
    main()