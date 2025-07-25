# CH9120Driver Test Suite

Comprehensive automated testing for the CH9120Driver with decent coverage.

## Quick Start

Open `/tests` directory as a Pico project
```bash
mkdir build && cd build && cmake .. -G Ninja && ninja
```

### Test Client Mode (Pico connects to PC)

When testing the Pico as a **TCP client**, the Python script acts as a **server**, launch it first and then flash the Pico:

```bash
# Terminal 1: Start Python server (waits for Pico to connect)
python3 test_runner.py --mode test_client

# Terminal 2: Flash client test and let it run
cmake .. && make
picotool load pico-blink-eth.uf2 -f
```

The Pico will connect to the Python server and run all 9 client tests automatically.

### Test Server Mode (PC connects to Pico)

When testing the Pico as a **TCP server**, the Python script acts as a **client**. Flash the Pico and then launch the script:

```bash
# Terminal 1: Flash server test first
cmake .. && make
picotool load pico-blink-eth.uf2 -f

# Terminal 2: Start Python client (connects to Pico server)
python3 test_runner.py --mode test_server
```

The Python client will connect to the Pico server and run all 5 server tests automatically.

## Configuration

Default network setup:
- **Pico IP**: `192.168.0.124`
- **PC IP**: `192.168.0.234` 
- **Test ports**: `8000`, `8001`

To use different Pico IP:
```bash
python3 test_runner.py --mode test_client --ip 192.168.1.100
python3 test_runner.py --mode test_server --ip 192.168.1.100
```

## Test Coverage

### Client Tests (9 tests)
- ✅ 1.1 Basic Client Lifecycle
- ✅ 1.3 Client with DHCP
- ✅ 2.1 Large Data Send (1024 bytes)
- ✅ 2.2 Large Data Receive (1024 bytes)
- ✅ 2.3 Ping-Pong Performance (100 iterations)
- ✅ 3.1 Connection Timeout
- ✅ 4.1 Connection Lifecycle (10 cycles)
- ✅ 4.2 Hot Reconfiguration (dual servers)
- ✅ 4.3 Duplicate Connection

### Server Tests (5 tests)
- ✅ 1.2 Basic Server Lifecycle
- ✅ 3.3 Server Recv Timeout
- ✅ Server with DHCP
- ✅ Server State Management
- ✅ Multiple Client Connections

## Advanced Usage

### Run Both Test Suites
```bash
python3 test_runner.py --mode both
```

### Custom Test Duration
```bash
python3 test_runner.py --mode test_client --duration 60
```

### Manual Testing (Legacy)

If you need the old manual approach:

**Client tests**: Flash and connect to serial console  
**Server tests**: Flash and use netcat: `nc 192.168.0.124 8000`

## Files

- `test_runner.py` - Automated test orchestrator (all-in-one)
- `test_client_main.cpp` - Pico client test suite  
- `test_server_main.cpp` - Pico server test suite

## Troubleshooting

- **No connection**: Check IP addresses and network
- **Tests timeout**: Increase duration with `--duration`
- **Build fails**: Check CMakeLists.txt configuration
- **Mixed results**: Check Pico serial output for details

The Python script tracks network traffic and basic connectivity, but check the Pico serial console for complete test results and detailed pass/fail information.

---

## Legacy Documentation

<details>
<summary>Click to see old detailed documentation</summary>

### Test Categories

| Category | Tests | Description |
|----------|-------|-------------|
| **Client Tests** | 1.1, 1.3, 2.1-2.3, 3.1-3.2, 4.1-4.3 | TCP client functionality |
| **Server Tests** | 1.2, 3.3 + additional server tests | TCP server functionality |
| **Multi-Instance** | Built-in to all tests | Validates multi-instance support |

### Detailed Test List

#### Category 1: Basic Connectivity Tests
- **1.1** Client: Connect, Send/Recv, Close - `test_1_1_basic_client_lifecycle()`
- **1.3** Client: DHCP Configuration - `test_1_3_client_with_dhcp()`
- **1.2** Server: Bind, Listen, Accept - `test_1_2_basic_server_lifecycle()`

#### Category 2: Performance Tests
- **2.1** Large Data Send (1024 bytes) - `test_2_1_large_data_send()`
- **2.2** Large Data Receive (1024 bytes) - `test_2_2_large_data_recv()`
- **2.3** Ping-Pong Performance (100x) - `test_2_3_ping_pong()`

#### Category 3: Robustness Tests
- **3.1** Connection Timeout - `test_3_1_connection_timeout()`
- **3.3** Server Recv Timeout - `test_3_3_server_recv_timeout()`

#### Category 4: Lifecycle Tests
- **4.1** Connection Cycles (10x) - `test_4_1_connection_cycles()`
- **4.2** Hot Reconfiguration - `test_4_2_hot_reconfiguration()`
- **4.3** Duplicate Connection - `test_4_3_duplicate_connection()`

### Network Configuration

Edit these constants in the test files:

```cpp
// In test_client_main.cpp
static constexpr uint8_t TEST_SERVER_IP[4] = {192, 168, 0, 234};  // PC IP
static constexpr uint8_t PICO_LOCAL_IP[4] = {192, 168, 0, 124};   // Pico IP

// In test_server_main.cpp  
static constexpr uint8_t PICO_LOCAL_IP[4] = {192, 168, 0, 124};   // Pico server IP
```

### Mock Server Modes

```bash
# Standard echo server (default)
python3 mock_server.py --host 0.0.0.0 --port 8000 --mode echo

# Server that immediately disconnects
python3 mock_server.py --port 8000 --mode disconnect

# Dual servers for reconfiguration tests
python3 mock_server.py --mode dual

# Silent client (connects but doesn't send data)
python3 mock_server.py --mode silent_client --target-host 192.168.0.124 --target-port 8000
```

</details>