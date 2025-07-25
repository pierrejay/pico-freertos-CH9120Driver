/* @file CH9120Driver.hpp */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "pico/time.h"
#include <optional>
#include <cstdio>
#include <cstring>
#include "../lib/UartDmaDriver/include/UartDmaDriver.hpp"

#include "CH9120Debug.hpp"
#include "CH9120Errno.hpp"

#ifndef CH9120_DEFAULT_CONNECT_TIMEOUT_MS // Default connect() timeout
    #define CH9120_DEFAULT_CONNECT_TIMEOUT_MS 10000 
#endif
#ifndef CH9120_DEFAULT_ACCEPT_TIMEOUT_MS // Default accept() timeout
    #define CH9120_DEFAULT_ACCEPT_TIMEOUT_MS 5000
#endif

// ===================================================================================
// DRIVER CLASS
// ===================================================================================

class CH9120Driver {
public:

    // ===================================================================================
    // TYPES
    // ===================================================================================

    /* @brief Result codes
     */
    enum Result {
        SUCCESS = 0,
        ERR_BUSY,
        ERR_CONFIG,
        ERR_INIT,
        ERR_TIMEOUT,
        ERR_UART,
        ERR_CONNECTION,
        ERR_STATE
    };
    /* @brief Helper to convert a result code to a string
     */
    static constexpr const char* toString(Result res) {
        switch (res) {
            case SUCCESS: return "Success";
            case ERR_CONFIG: return "Configuration error";
            case ERR_INIT: return "Init error";
            case ERR_TIMEOUT: return "Timeout";
            case ERR_UART: return "UART error";
            case ERR_CONNECTION: return "Connection error";
            case ERR_STATE: return "Invalid driver state";
            default: return "Unknown error";
        }
    }
    /* @brief Helper to cast an error
     */
    static inline Result Error(Result res, const char* msg = nullptr, CH9120Debug::CallCtx ctx = CH9120Debug::CallCtx()) {
        if (msg) {
            CH9120Debug::logf_ctx(ctx, "Error: %s (%s)\n", toString(res), msg);
        } else {
            CH9120Debug::logf_ctx(ctx, "Error: %s\n", toString(res));
        }
        return res;
    }
    /* @brief Helper to return success result
     */
    static inline Result Success() {
        return SUCCESS;
    }

    /* @brief Network mode
     */
    enum class Mode {
        TCP_SERVER = 0,
        TCP_CLIENT = 1,
        UDP_SERVER = 2,
        UDP_CLIENT = 3
    };

    /* @brief State management
     */
    enum State {
        STOPPED,       // Not configured
        CONFIG,        // Configuring  
        RUN_OK,        // Active
        RUN_OVERFLOW   // RX overflow detected
    };

    /* @brief Hardware configuration structure for CH9120 Ethernet module
     */
    struct HardwareConfig {
        uart_inst_t* uart        = uart1;
        uint32_t     baudrate    = 921600;
        uint8_t      txPin       = 20;
        uint8_t      rxPin       = 21;
        uint8_t      cfgPin      = 18;
        uint8_t      resPin      = 19;
        uint8_t      statusPin   = 17;
    };

    /* @brief Network configuration
     */
    struct NetworkConfig {
        uint8_t   localIp[4]      = {192, 168, 1, 200};
        uint8_t   gateway[4]      = {192, 168, 1, 1};
        uint8_t   subnetMask[4]   = {255, 255, 255, 0};
        bool      useDhcp         = true; // Enable DHCP by default
        uint8_t   targetIp[4]     = {192, 168, 1, 100};
        uint16_t  localPort       = 8000;
        uint16_t  targetPort      = 8000;
        Mode      mode            = Mode::TCP_CLIENT;

        /* @brief Validate configuration parameters
         */
        Result validate() const {
            if (localPort == 0 || targetPort == 0) return Error(ERR_CONFIG, "Port cannot be 0");
            if (!useDhcp) {
                bool allZero = true;
                for (auto b: localIp) if (b != 0) { allZero = false; break; }
                if (allZero) return Error(ERR_CONFIG, "Local IP cannot be all zeros when DHCP is disabled");
            }
            return Success();
        }
        
        /* @brief Check equality between 2 different configurations
         */
        bool operator==(const NetworkConfig& other) const {
            return (memcmp(localIp, other.localIp, 4) == 0 &&
                    memcmp(gateway, other.gateway, 4) == 0 &&
                    memcmp(subnetMask, other.subnetMask, 4) == 0 &&
                    memcmp(targetIp, other.targetIp, 4) == 0 &&
                    useDhcp == other.useDhcp &&
                    localPort == other.localPort &&
                    targetPort == other.targetPort &&
                    mode == other.mode);
        }
    };

    /* @brief Link state events (connection status)
     */
    enum LinkEventType {
        LINK_CONNECTED,     // Connected with remote server, or listening for clients (status pin LOW)
        LINK_DISCONNECTED   // Link lost with remote server or no longer listening (status pin HIGH)
    };
    /* @brief Convert a link event type to a string
     */
    static constexpr const char* toString(LinkEventType type) {
        switch (type) {
            case LINK_CONNECTED: return "Link connected";
            case LINK_DISCONNECTED: return "Link disconnected";
            default: return "Unknown link event";
        }
    }
    /* @brief Link event structure
     */
    struct LinkEvent {
        LinkEventType type;
        uint64_t timestamp_us;  // Timestamp in microseconds from boot
    };

    // ===================================================================================
    // SYNC TYPES
    // ===================================================================================

    using Mutex = UartDmaDriver::Mutex;
    using Lock = UartDmaDriver::Lock;
    using BinarySemaphore = UartDmaDriver::BinarySemaphore;

    // ===================================================================================
    // CONSTANTS
    // ===================================================================================

    // CH9120 default settings
    static constexpr uint8_t DEFAULT_UART_TX = 20;    // Waveshare board
    static constexpr uint8_t DEFAULT_UART_RX = 21;    // Waveshare board
    static constexpr uint8_t DEFAULT_CFG_PIN = 18;    // Waveshare board
    static constexpr uint8_t DEFAULT_RES_PIN = 19;    // Waveshare board
    static constexpr uint8_t DEFAULT_STATUS_PIN = 17; // Waveshare board
    static constexpr uint32_t DEFAULT_TRANSPORT_BAUDRATE = 115200;
    static constexpr uint32_t CH9120_MAX_BAUDRATE = 921600;

    // CH9120 configuration mode constants
    static constexpr uint32_t CFG_BAUDRATE = 9600;
    static constexpr uint32_t CFG_PIN_SET_DELAY_MS = 10;    // Delay after pin change
    static constexpr uint32_t CFG_STARTUP_DELAY_MS = 500;   // Delay after boot/reboot
    static constexpr uint32_t CFG_RESPONSE_TIMEOUT_MS = 5000;
    static constexpr uint32_t CFG_CONNECT_CHECK_DELAY = 50; // Delay to check state in connect()

    // CH9120 "hidden settings" (library defaults, not exposed in NetworkConfig)
    static constexpr uint16_t CFG_TX_PACKET_LENGTH = 512; // 1-512 bytes (default: 512 for max throughput)
    static constexpr uint8_t CFG_TX_TIMEOUT = 0;          // 0-200 (*5ms) (default: 0 for hardware timeout)
    static constexpr bool CFG_DISCONNECT_ON_LINK_DOWN = true; // Disconnect when cable is disconnected (default: true)
    static constexpr bool CFG_RANDOM_LOCAL_PORT = false; // Use random local port (default: false, use fixed port)
    static constexpr bool CFG_FLUSH_ON_CONNECT = false; // Flush CH9120 buffer on network disconnect (default: false)

    // Other settings
    static constexpr uint32_t SPINLOCK_TICKS = 5; // Number of ticks to yield for in spinlock (used in close())
    static constexpr uint32_t UART_FLUSH_TIMEOUT_MS = 3000; // Timeout for UART flush operations (clearRxBuffer())
    static constexpr uint32_t DEFAULT_CONNECT_TIMEOUT_MS = CH9120_DEFAULT_CONNECT_TIMEOUT_MS; // Default connect() timeout
    static constexpr uint32_t DEFAULT_ACCEPT_TIMEOUT_MS = CH9120_DEFAULT_ACCEPT_TIMEOUT_MS; // Default accept() timeout
    
    // Link event queue settings
    static constexpr size_t LINK_EVENT_QUEUE_SIZE = 8; // Max link events in queue

    // ===================================================================================
    // PUBLIC MEMBERS
    // ===================================================================================

    CH9120Driver(uart_inst_t* uart = uart1,
             uint32_t baudRate = DEFAULT_TRANSPORT_BAUDRATE,
             uint8_t txPin = DEFAULT_UART_TX,
             uint8_t rxPin = DEFAULT_UART_RX,
             uint8_t cfgPin = DEFAULT_CFG_PIN,
             uint8_t resPin = DEFAULT_RES_PIN,
             uint8_t statusPin = DEFAULT_STATUS_PIN);

    ~CH9120Driver();

    // Initialisation
    Result begin();
    Result setConfig(const NetworkConfig& config);
    Result startRxTx();
    Result stopRxTx();
    
    // State management (public API)
    bool isConnected() const;
    State getDriverState() const;
    bool isOverflowed() const;
    
    // Socket-like API
    int bind(const NetworkConfig& config);  // Configure socket (server/client)
    int connect(const NetworkConfig& config, TickType_t timeout = pdMS_TO_TICKS(DEFAULT_CONNECT_TIMEOUT_MS));  // Client mode only
    int listen(int backlog = 1);            // Start listening (server only)
    int accept(TickType_t timeout = pdMS_TO_TICKS(DEFAULT_ACCEPT_TIMEOUT_MS));  // Server mode: accept incoming connection
    void close();       // Close socket (stops server/client)
    int send(const uint8_t* data, size_t len, TickType_t timeout = portMAX_DELAY);
    int recv(uint8_t* buffer, size_t maxLen, TickType_t timeout = 0);
    
    // Raw access API (protected = safe)
    Result sendRaw(const uint8_t* data, size_t len, TickType_t waitTicks = portMAX_DELAY);
    size_t readRaw(uint8_t* dst, size_t maxLen);
    QueueHandle_t getDataEventQueueHandle() const { return _uartDriver.getEventQueueHandle(); }
    
    // Link event API (hardware connection monitoring)
    QueueHandle_t getLinkEventQueueHandle() const { return _linkEventQueue; }
    

private:

    // ===================================================================================
    // PRIVATE MEMBERS
    // ===================================================================================

    // Configuration & state management
    uart_inst_t* _uart;
    uint8_t _txPin, _rxPin, _cfgPin, _resPin, _statusPin;
    bool _initialized;
    bool _configured;
    uint32_t _trspBaudRate;
    volatile State _stateUnsafe; // Must be accessed under critical section only
    std::optional<NetworkConfig> _currentConfig; // Stores current configuration
    Mutex _drvMutex; // Mutex for thread-safe access

    // Link event queue (hardware connection monitoring)
    QueueHandle_t _linkEventQueue;
    StaticQueue_t _linkEventQueueBuffer;
    uint8_t _linkEventQueueStorage[LINK_EVENT_QUEUE_SIZE * sizeof(LinkEvent)];
    volatile bool _lastLinkState; // Last known link state (from ISR)

    // UART DMA driver integration
    UartDmaDriver _uartDriver; // Replaces custom DMA logic

    // Communication utilities
    void clearRxBuffer();
    
    // Socket API helpers
    static constexpr int resultToErrno(Result result);
    int setupSocket(const NetworkConfig& config, TickType_t timeout); // Common setup for client/server
    

    // ===================================================================================
    // CONFIGURATION METHODS
    // ===================================================================================

    // Configuration methods - WARNING: UNSAFE, MUST BE CALLED UNDER MUTEX
    Result setConfigUnsafe(const NetworkConfig& config); // Version sans mutex pour appels internes
    Result stopRxTxUnsafe(); // Version sans mutex pour appels internes
    Result startRxTxUnsafe(); // Version sans mutex pour appels internes
    Result cfg_sendCommand(const uint8_t* cmd, size_t len);
    Result cfg_waitForResponse(uint32_t timeoutMs = CFG_RESPONSE_TIMEOUT_MS);
    void cfg_enterConfigMode();
    void cfg_exitConfigMode();
    Result cfg_setMode(Mode mode);
    Result cfg_setLocalIP(const uint8_t ip[4]);
    Result cfg_setSubnetMask(const uint8_t mask[4]);
    Result cfg_setGateway(const uint8_t gateway[4]);
    Result cfg_setTargetIP(const uint8_t ip[4]);
    Result cfg_setLocalPort(uint16_t port);
    Result cfg_setTargetPort(uint16_t port);
    Result cfg_setBaudRate(uint32_t baud);
    Result cfg_setDataMode();
    Result cfg_setRandomLocalPort(bool random);
    Result cfg_setTxTimeout(uint8_t timeout);
    Result cfg_setLinkDownMode(bool disconnect);
    Result cfg_setTxPacketLength(uint16_t length);
    Result cfg_setFlushOnConnect(bool flush);
    Result cfg_setDhcpMode(bool enabled);
    Result cfg_finalizeConfig();

    // Operation pins
    inline void pins_setEnable(bool enable); // false = sleep, true = enable
    inline bool pins_getEnable() const;
    inline void pins_setConfig(bool cfg); // false = transport mode, true = config mode
    inline bool pins_getConfig() const;
    inline bool pins_getConnected() const;
    
    // Link state monitoring
    bool initLinkMonitoring();
    void deinitLinkMonitoring();
    static void rawIrqHandler();  // Raw IRQ handler for shared IRQ API
    void updateLinkState(); // Updates _lastLinkState based on current config/mode and generates events
    uint8_t getStatusPin() const { return _statusPin; } // Public access for raw IRQ handler

    // ===================================================================================
    // ATOMIC ACCESS HELPERS FOR STATE (SHARED RESOURCE)
    // ===================================================================================

    /* @brief Get the current state
     * @note Uses critical section for safe concurrent access
     */
    inline State getState() const {
        taskENTER_CRITICAL();
        State state = _stateUnsafe;
        taskEXIT_CRITICAL();
        return state;
    }
    
    /* @brief Set the current state
     * @note Uses critical section for safe concurrent access
     */
    inline void setState(State newState) {
        taskENTER_CRITICAL();
        _stateUnsafe = newState;
        taskEXIT_CRITICAL();
    }
    
    /* @brief Compare and set the current state
     * @note Uses critical section for safe concurrent access
     */
    inline bool compareAndSetState(State expectedState, State newState) {
        taskENTER_CRITICAL();
        bool changed = (_stateUnsafe == expectedState);
        if (changed) {
            _stateUnsafe = newState;
        }
        taskEXIT_CRITICAL();
        return changed;
    }
    
    /* @brief Get the current state from ISR
     * @note Uses critical section for safe concurrent access
     */
    __attribute__((always_inline, hot)) inline State getStateFromISR() const {
        UBaseType_t savedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
        State state = _stateUnsafe;
        taskEXIT_CRITICAL_FROM_ISR(savedInterruptStatus);
        return state;
    }
    
    /* @brief Set the current state from ISR
     * @note Uses critical section for safe concurrent access
     */
    __attribute__((always_inline, hot)) inline void setStateFromISR(State newState) {
        UBaseType_t savedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
        _stateUnsafe = newState;
        taskEXIT_CRITICAL_FROM_ISR(savedInterruptStatus);
    }

    /* @brief Compare and set the current state from ISR
     * @note Uses critical section for safe concurrent access
     */
    __attribute__((always_inline, hot)) inline bool compareAndSetStateFromISR(State expectedState, State newState) {
        UBaseType_t savedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
        bool changed = (_stateUnsafe == expectedState);
        if (changed) {
            _stateUnsafe = newState;
        }
        taskEXIT_CRITICAL_FROM_ISR(savedInterruptStatus);
        return changed;
    }
};