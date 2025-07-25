/* @file CH9120Driver.cpp */

#include "CH9120Driver.hpp"
#include "pico/stdlib.h"
#include "pico/platform.h"
#include "hardware/irq.h"
#include "FreeRTOS.h"
#include "task.h"
#include <cstring>
#include <cstdio>
#include <algorithm>

/* @brief Multi-instance support structure (up to 2 instances: uart0/uart1) */
struct InstanceSlot {
    uart_inst_t* uart;
    CH9120Driver* driver;
};

static constexpr int NUM_UART_INSTANCES = 2; // Maximum number of UART instances on Pico
static InstanceSlot CH9120Driver_instances[NUM_UART_INSTANCES] = {{nullptr, nullptr}, {nullptr, nullptr}};

/* @brief Helper function to get the UART index
 * @param uart The UART instance
 * @return The index of the UART instance
 */
static int getUartIndex(uart_inst_t* uart) {
    if (uart == uart0) return 0;
    if (uart == uart1) return 1;
    return -1; // Invalid UART
}


// ===================================================================================
// CTOR/DTOR
// ===================================================================================

/* @brief Constructor
 * @param uart The UART instance
 * @param baudRate The baud rate
 * @param txPin The TX pin
 * @param rxPin The RX pin
 * @param cfgPin The CFG pin
 * @param resPin The RES pin
 * @param statusPin The STATUS pin
 */
CH9120Driver::CH9120Driver(uart_inst_t* uart, uint32_t baudRate, 
                   uint8_t txPin, uint8_t rxPin, uint8_t cfgPin, 
                   uint8_t resPin, uint8_t statusPin) : 
    _uart(uart), 
    _txPin(txPin), 
    _rxPin(rxPin), 
    _cfgPin(cfgPin), 
    _resPin(resPin), 
    _statusPin(statusPin), 
    _initialized(false), 
    _configured(false), 
    _stateUnsafe(STOPPED),
    _trspBaudRate(baudRate),
    _currentConfig(std::nullopt),
    _linkEventQueue(nullptr),
    _lastLinkState(false),
    _uartDriver(uart, txPin, rxPin, CFG_BAUDRATE) {}

/* @brief Destructor
 */
CH9120Driver::~CH9120Driver() {
    // Cleanup link state monitoring
    deinitLinkMonitoring();
    vTaskDelay(pdMS_TO_TICKS(50)); // Wait for any pending IRQs to finish
    
    // UartDmaDriver handles its own cleanup in its destructor
    
    // Release instance slot (thread-safe)
    int index = getUartIndex(_uart);
    if (index >= 0 && index < NUM_UART_INSTANCES && 
        CH9120Driver_instances[index].driver == this) {
        taskENTER_CRITICAL();
        CH9120Driver_instances[index] = { nullptr, nullptr };
        taskEXIT_CRITICAL();
    }
}


// ===================================================================================
// PUBLIC API
// ===================================================================================

/* @brief Initialize the driver
 * @return Result code
 */
CH9120Driver::Result CH9120Driver::begin() {
    if (_initialized) return Success();

    // Register this instance in the instances array (thread-safe)
    taskENTER_CRITICAL();
    int index = getUartIndex(_uart);
    if (index < 0 || index >= NUM_UART_INSTANCES) {
        taskEXIT_CRITICAL();
        return Error(ERR_INIT, "Invalid UART instance provided");
    }

    if (CH9120Driver_instances[index].driver != nullptr) {
        taskEXIT_CRITICAL();
        return Error(ERR_INIT, "A CH9120Driver instance is already using this UART peripheral");
    }

    CH9120Driver_instances[index] = { _uart, this };
    taskEXIT_CRITICAL();

    // Check transport baud rate compatibility with the chip
    if (_trspBaudRate > CH9120_MAX_BAUDRATE) {
        return Error(ERR_CONFIG, "Baud rate too high - CH9120 max: 921600 bps");
    }
    
    // Configure chips pin
    gpio_init(_cfgPin);
    gpio_init(_resPin);
    gpio_init(_statusPin);
    gpio_set_dir(_cfgPin, GPIO_OUT);
    gpio_set_dir(_resPin, GPIO_OUT);
    gpio_set_dir(_statusPin, GPIO_IN);
    gpio_pull_up(_statusPin);
    
    // Initial state: "run mode", kept asleep until configured
    pins_setEnable(false); // Disable chip by default
    pins_setConfig(false); // Start in transport mode

    // Validate transport baud rate using UartDmaDriver's standard rate check
    if (!UartDmaDriver::isStandardBaudrate(_trspBaudRate)) {
        return Error(ERR_CONFIG, "Non-standard transport baud rate - use 9600, 19200, 38400, 57600, 115200, 230400, 460800, or 921600");
    }

    // Initialize UartDmaDriver (handles UART, GPIO, and DMA setup)
    if (_uartDriver.init() != UartDmaDriver::SUCCESS) {
        return Error(ERR_UART, "Failed to initialize UartDmaDriver");
    }
    
    // Initialize link state monitoring
    if (!initLinkMonitoring()) {
        return Error(ERR_INIT, "Failed to initialize link state monitoring");
    };
    
    _initialized = true;
    return Success();
}

/* @brief Set the configuration
 * @param config The network configuration
 * @return Result code
 */
CH9120Driver::Result CH9120Driver::setConfig(const NetworkConfig& config) {
    if (!_initialized) return Error(ERR_INIT, "Driver not initialized");
    Lock guard(_drvMutex);
    return setConfigUnsafe(config);
}


/* @brief Start RX/TX
 * @return Result code
 */
CH9120Driver::Result CH9120Driver::startRxTx() {
    if (!_initialized) return Error(ERR_INIT, "Driver not initialized");
    if (!_configured) return Error(ERR_CONFIG, "Driver not configured");
    if (getState() != STOPPED) return Error(ERR_STATE, "Driver must be stopped before starting RX");
    
    // Validate hardware state
    if (!pins_getEnable()) return Error(ERR_INIT, "CH9120 chip is disabled");
    if (pins_getConfig()) return Error(ERR_CONFIG, "CH9120 still in config mode");

    Lock guard(_drvMutex, 0); // Try-lock no wait
    if (!guard.isLocked()) return Error(ERR_BUSY);
    return startRxTxUnsafe();
}

/* @brief Stop RX/TX
 * @return Result code
 */
CH9120Driver::Result CH9120Driver::stopRxTx() {
    if (getState() == STOPPED) return Success(); // Already stopped
    if (!_initialized) return Error(ERR_INIT, "Driver not initialized");

    Lock guard(_drvMutex);
    if (!guard.isLocked()) return Error(ERR_BUSY);
    return stopRxTxUnsafe();
}


/* @brief Check if the driver is connected
 * @return True if connected, false otherwise
 */
bool CH9120Driver::isConnected() const {
    return _lastLinkState;
}

/* @brief Get the driver state
 * @return The driver state
 * @note The getState() accessor uses taskENTER_CRITICAL which could be problematic
 * if the user is calling the function repeatedly (e.g. polling loop). The odds
 * of getting an inconsistent state at the moment it changes are quite low, and
 * don't cause any issue in practice (it will cost one extra loop iteration) -
 * unlike inside the driver where we MUST read a perfectly synchronized state to
 * manage operations correctly and avoid race conditions. 
 * However, constantly blocking IRQs by entering critical section repeteadly
 * would be VERY problematic!
 * TLDR: we return the raw ("Unsafe") state to the user, so that IRQs are not
 * affected by the behaviour of the user application.
 */
CH9120Driver::State CH9120Driver::getDriverState() const {
    return _stateUnsafe; // Soft real-time -> no critical section (see comment)
}


/* @brief Check if the driver is overflowed
 * @return True if overflowed, false otherwise
 */
bool CH9120Driver::isOverflowed() const {
    return _uartDriver.isOverflowed();
}


// ===================================================================================
// SOCKET API - PUBLIC METHODS
// ===================================================================================


/* @brief Connect to a server
 * @param config The network configuration
 * @param timeout The timeout
 * @return Result code
 */
int CH9120Driver::connect(const NetworkConfig& config, TickType_t timeout) {
    if (!_initialized) return -SockErrno::ENOTSOCK;
    
    // Validate this is client mode only
    if (config.mode != Mode::TCP_CLIENT && config.mode != Mode::UDP_CLIENT) {
        return -SockErrno::EINVAL; // connect() only for client modes
    }
    
    // Setup socket
    int setupResult = setupSocket(config, timeout);
    if (setupResult != 0) return setupResult;

    // Start communication
    Result startResult = startRxTx();
    if (startResult != SUCCESS) {
        return resultToErrno(startResult);
    }
    CH9120Debug::logf("Started client communication...");
    
    // For client mode: wait for connection establishment
    if (config.mode == Mode::TCP_CLIENT) {
        uint32_t startTime = to_ms_since_boot(get_absolute_time());
        uint32_t timeoutMs = timeout * portTICK_PERIOD_MS;
        while (!isConnected()) {
            uint32_t elapsed = to_ms_since_boot(get_absolute_time()) - startTime;
            if (elapsed >= timeoutMs) {
                return -SockErrno::ETIMEDOUT; // Connection timeout
            }
            vTaskDelay(pdMS_TO_TICKS(CFG_CONNECT_CHECK_DELAY));
        }
        CH9120Debug::logf("TCP client connected successfully");
    } else {
        CH9120Debug::logf("UDP client ready to send");
    }
    
    return 0; // Success
}


/* @brief Bind to a port
 * @param config The network configuration
 * @return Result code
 */
int CH9120Driver::bind(const NetworkConfig& config) {
    if (!_initialized) return -SockErrno::ENOTSOCK;
    
    // Setup socket (configure CH9120)
    int setupResult = setupSocket(config, pdMS_TO_TICKS(10000));
    if (setupResult != 0) return setupResult;
    
    CH9120Debug::logf("Socket bound, ready to listen or connect");
    return 0; // Success
}

/* @brief Listen for connections
 * @param backlog The backlog
 * @return Result code
 */
int CH9120Driver::listen(int backlog) {
    if (!_initialized) return -SockErrno::ENOTSOCK;
    if (!_configured) return -SockErrno::EINVAL;
    
    // Validate we're in server mode
    if (_currentConfig.has_value()) {
        Mode currentMode = _currentConfig.value().mode;
        if (currentMode != Mode::TCP_SERVER && currentMode != Mode::UDP_SERVER) {
            return -SockErrno::EINVAL; // listen() only for server modes
        }
    } else {
        return -SockErrno::ENOTCONN;
    }
    
    // Just start RX (socket already configured by bind())
    if (getState() != STOPPED) return -SockErrno::EINVAL;
    
    Lock guard(_drvMutex);
    if (!guard.isLocked()) return -SockErrno::EBUSY;
    
    Result startResult = startRxTxUnsafe();
    if (startResult != SUCCESS) {
        return resultToErrno(startResult);
    }
    
    CH9120Debug::logf("Server listening (backlog=%d)", backlog);
    return 0; // Success
}


/* @brief Accept a connection
 * @param timeout The timeout
 * @return Result code
 */
int CH9120Driver::accept(TickType_t timeout) {
    if (!_initialized) return -SockErrno::ENOTSOCK;
    if (!_configured) return -SockErrno::ENOTCONN;
    if (getState() != RUN_OK) return -SockErrno::ENOTCONN;
    
    // Validate we're in server mode
    if (_currentConfig.has_value()) {
        Mode currentMode = _currentConfig.value().mode;
        if (currentMode != Mode::TCP_SERVER && currentMode != Mode::UDP_SERVER) {
            CH9120Debug::logf("Accept called on non-server socket");
            return -SockErrno::EINVAL; // Not a server socket
        }
    } else {
        return -SockErrno::ENOTCONN;
    }
    
    // CH9120 handles TCP/IP stack - server already accepts connections after listen()
    // This is essentially a stub for API compatibility
    CH9120Debug::logf("Server ready to accept connections (CH9120 handles TCP/IP)");
    return 0; // Always success - CH9120 manages connections automatically
}


/* @brief Close the socket
 */
void CH9120Driver::close() {
    if (_initialized && getState() != STOPPED) {
        // Wait for current TX operation to finish before stopping RX
        while (_uartDriver.isTxBusy()) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        stopRxTx();
    }
}


/* @brief Send data
 * @param data The data to send
 * @param len The length of the data
 * @param timeout The timeout
 * @return Result code
 */
int CH9120Driver::send(const uint8_t* data, size_t len, TickType_t timeout) {
    if (!_initialized) return -SockErrno::ENOTSOCK;
    if (!_configured) return -SockErrno::ENOTCONN;
    if (getState() != RUN_OK) return -SockErrno::ENOTCONN;
    if (!data) return -SockErrno::EFAULT;
    if (len == 0) return 0; // Nothing to send
    
    // Check message size limits
    if (len > UartDmaDriver::DMA_TX_BUFFER_SIZE) {
        return -SockErrno::EMSGSIZE;
    }
    
    // Use UartDmaDriver for transmission
    UartDmaDriver::Result result = _uartDriver.send(data, len, timeout);
    
    switch (result) {
        case UartDmaDriver::SUCCESS:
            return (int)len; // Return bytes sent
        case UartDmaDriver::ERR_TX_BUSY:
            return (timeout == 0) ? -SockErrno::EWOULDBLOCK : -SockErrno::ETIMEDOUT;
        case UartDmaDriver::ERR_TX_OVERFLOW:
            return -SockErrno::EMSGSIZE;
        case UartDmaDriver::ERR_STATE:
            return -SockErrno::ENOTCONN;
        default:
            return -SockErrno::ENETDOWN;
    }
}


/* @brief Receive data
 * @param buffer The buffer to receive the data
 * @param maxLen The maximum length of the data
 * @param timeout The timeout
 * @return Result code
 */
int CH9120Driver::recv(uint8_t* buffer, size_t maxLen, TickType_t timeout) {
    if (!_initialized) return -SockErrno::ENOTSOCK;
    if (!_configured) return -SockErrno::ENOTCONN;
    if (!buffer) return -SockErrno::EFAULT;
    if (maxLen == 0) return 0; // Nothing to receive
    
    // Check if data is immediately available
    size_t available = _uartDriver.available();
    if (available > 0) {
        size_t toRead = (available > maxLen) ? maxLen : available;
        return (int)_uartDriver.read(buffer, toRead);
    }
    
    // No immediate data - handle timeout
    if (timeout == 0) return 0; // Non-blocking, no data available
    
    // Wait for data with timeout
    UartDmaDriver::Event event;
    if (_uartDriver.popEvent(&event, timeout)) {
        if (event.type == UartDmaDriver::EVT_DATA) {
            // Data available now
            size_t available = _uartDriver.available();
            if (available > 0) {
                size_t toRead = (available > maxLen) ? maxLen : available;
                return (int)_uartDriver.read(buffer, toRead);
            }
        }
        if (event.type == UartDmaDriver::EVT_OVERFLOW) {
            return -SockErrno::ENOBUFS; // Buffer overflow
        }
    }
    
    return -SockErrno::ETIMEDOUT; // Timeout waiting for data
}

// ===================================================================================
// RAW ACCESS API - PUBLIC METHODS
// ===================================================================================


/* @brief Send raw data
 * @param data The data to send
 * @param len The length of the data
 * @param waitTicks The timeout
 * @return Result code
 */
CH9120Driver::Result CH9120Driver::sendRaw(const uint8_t* data, size_t len, TickType_t waitTicks) {
    if (!_initialized) return Error(ERR_INIT, "Driver not initialized");
    if (!_configured) return Error(ERR_CONFIG);
    if (getState() != RUN_OK && getState() != RUN_OVERFLOW) return Error(ERR_STATE, "Driver must be running to send data");

    // Use UartDmaDriver for transmission
    auto result = _uartDriver.send(data, len, waitTicks);
    
    // Convert UartDmaDriver result to CH9120Driver result
    switch (result) {
        case UartDmaDriver::SUCCESS:
            return Success();
        case UartDmaDriver::ERR_TX_BUSY:
            return Error(ERR_BUSY, "TX DMA is busy");
        case UartDmaDriver::ERR_TX_OVERFLOW:
            return Error(ERR_UART, "Data too large for TX buffer");
        default:
            return Error(ERR_UART, "UartDmaDriver send failed");
    }
}


/* @brief Read raw data
 * @param dst The buffer to read the data into
 * @param maxLen The maximum length of the data
 * @return The number of bytes read
 */
size_t CH9120Driver::readRaw(uint8_t* dst, size_t maxLen) {
    if (!_initialized || !_configured) return 0; // Not initialized or configured
    if (maxLen == 0) return 0;
    
    // Use UartDmaDriver to read data
    size_t bytes_read = _uartDriver.read(dst, maxLen);
    
    // Synchronize overflow state with UartDmaDriver
    if (_uartDriver.isOverflowed() && getState() == RUN_OK) {
        setState(RUN_OVERFLOW);
    } else if (getState() == RUN_OVERFLOW) {
        // UartDmaDriver handles auto-recovery, sync our state
        setState(RUN_OK);
    }
    
    return bytes_read;
}


// ===================================================================================
// GENERAL - PRIVATE METHODS
// ===================================================================================


/* @brief Start RX/TX
 * @return Result code
 */
CH9120Driver::Result CH9120Driver::startRxTxUnsafe() {
    // Start UartDmaDriver
    auto result = _uartDriver.start();
    if (result != UartDmaDriver::SUCCESS) {
        setState(STOPPED); // Failed to start - ensure we remain in safe STOPPED state
        return Error(ERR_UART, "Failed to start UartDmaDriver");
    }

    setState(RUN_OK);

    // Update link state after starting RX 
    // (in case we enabled UDP or SERVER mode -> no link state pin)
    updateLinkState();
    
    return Success();
}


/* @brief Stop RX/TX UART operations
 * @return Result code
 * @note Important: this function MUST be called under mutex!
 */
CH9120Driver::Result CH9120Driver::stopRxTxUnsafe() {
    // Stop UartDmaDriver
    _uartDriver.stop();
    
    // Reset connection state when stopping
    _lastLinkState = false;
    
    // Reset state
    setState(STOPPED);

    // Update link state after starting RX 
    // (in case we enabled UDP or SERVER mode -> no link state pin)
    updateLinkState();
    
    return Success();
}


/* @brief Update the link state according to current config & pin state
 */
void CH9120Driver::updateLinkState() {
    // Capture previous state for event generation
    bool previousState = _lastLinkState;
    
    // Update link state based on current configuration
    if (!_initialized || !_currentConfig.has_value() 
        || getState() == STOPPED) { // Driver not initialized or stopped
        _lastLinkState = false;
    } else if (!pins_getEnable() || pins_getConfig()) { // Chip is disabled or in config mode
        _lastLinkState = false;
    } else {
        // In server mode, assume it's connected (we cannot know if a client is live)
        Mode currentMode = _currentConfig.value().mode;
        if (currentMode == Mode::TCP_SERVER 
            || currentMode == Mode::UDP_SERVER 
            || currentMode == Mode::UDP_CLIENT) {
            _lastLinkState = true;
        } else {
            // In TCP client mode, read pin directly
            _lastLinkState = pins_getConnected();
        }
    }
    
    // Generate event if state changed and queue exists
    if (_linkEventQueue && _lastLinkState != previousState) {
        LinkEvent event;
        event.type = _lastLinkState ? LINK_CONNECTED : LINK_DISCONNECTED;
        event.timestamp_us = time_us_64();
        
        // Send to queue (check if called from ISR context)
        if (xPortIsInsideInterrupt()) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(_linkEventQueue, &event, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        } else {
            xQueueSend(_linkEventQueue, &event, 0); // Non-blocking
        }
    }
}


/* @brief Clear the harware RX buffer
 */
void CH9120Driver::clearRxBuffer() {
    uint32_t start = to_ms_since_boot(get_absolute_time());

    while (uart_is_readable(_uart)) {
        uart_getc(_uart);
        if (to_ms_since_boot(get_absolute_time()) - start >= UART_FLUSH_TIMEOUT_MS) return;
        tight_loop_contents();
    }
}


// ===================================================================================
// SOCKET API - PRIVATE METHODS
// ===================================================================================

/* @brief Convert internal Result codes to Linux errno values
 * @param result The result code
 * @return The errno value
 */
constexpr int CH9120Driver::resultToErrno(Result result) {
    switch (result) {
        case SUCCESS: return 0;
        case ERR_INIT: return -SockErrno::ENOTSOCK;
        case ERR_BUSY: return -SockErrno::EBUSY;
        case ERR_CONFIG: return -SockErrno::EINVAL;
        case ERR_TIMEOUT: return -SockErrno::ETIMEDOUT;
        case ERR_UART: return -SockErrno::ENETDOWN;
        case ERR_CONNECTION: return -SockErrno::ECONNREFUSED;
        case ERR_STATE: return -SockErrno::EINVAL;
        default: return -SockErrno::EINVAL;
    }
}


/* @brief Common setup function for both client and server modes
 * @param config The network configuration
 * @param timeout The timeout
 * @return The errno value
 */
int CH9120Driver::setupSocket(const NetworkConfig& config, TickType_t timeout) {
    // Validate config
    if (config.validate() != SUCCESS) return -SockErrno::EINVAL;
    
    // Stop & configure driver under mutex
    Lock guard(_drvMutex, timeout);
    if (!guard.isLocked()) { return -SockErrno::EBUSY; } // Driver is busy
            
    // Check if already running with same config
    if (getState() == RUN_OK && 
        _currentConfig.has_value() && _currentConfig.value() == config) {
        CH9120Debug::logf("Already configured with same config");
        return -SockErrno::EISCONN; // Already running with same config
    }
    
    // Stop if running with different config
    if (getState() != STOPPED) {
        Result stopResult = stopRxTxUnsafe();
        if (stopResult != SUCCESS) return resultToErrno(stopResult);
    }
    
    // Only reconfigure if necessary
    bool needsReconfig = !_currentConfig.has_value() || !(_currentConfig.value() == config);
    if (needsReconfig) {
        CH9120Debug::logf("Configuring CH9120 chip...");
        Result configResult = setConfigUnsafe(config);
        if (configResult != SUCCESS) {
            return resultToErrno(configResult);
        }
    } else {
        CH9120Debug::logf("Using existing configuration...");
    }
    CH9120Debug::logf("CH9120 configured, ready to start communication");
    
    return 0; // Success
}


// ===================================================================================
// LINK EVENT QUEUE IMPLEMENTATION
// ===================================================================================


/* @brief Initialize link monitoring
 * @return True if successful, false otherwise
 */
bool CH9120Driver::initLinkMonitoring() {
    // Create link event queue
    _linkEventQueue = xQueueCreateStatic(
        LINK_EVENT_QUEUE_SIZE,
        sizeof(LinkEvent),
        _linkEventQueueStorage,
        &_linkEventQueueBuffer
    );
    
    if (!_linkEventQueue) {
        return false;
    }
    
    // Initialize link state
    _lastLinkState = pins_getConnected();
    
    // Setup GPIO interrupt for status pin using shared IRQ API (multi-callback safe)
    gpio_add_raw_irq_handler(_statusPin, &rawIrqHandler);
    irq_set_enabled(IO_IRQ_BANK0, true); // Enable GPIO bank IRQ if not already done
    gpio_set_irq_enabled(_statusPin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    return true;
}


/* @brief Deinitialize link monitoring
 */
void CH9120Driver::deinitLinkMonitoring() {
    if (_linkEventQueue) {
        // Remove our specific handler and disable GPIO interrupt
        gpio_remove_raw_irq_handler(_statusPin, &rawIrqHandler);
        gpio_set_irq_enabled(_statusPin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
        
        // Queue will be automatically cleaned up (static allocation)
        _linkEventQueue = nullptr;
    }
}


/* @brief Raw IRQ handler for shared IRQ API (multi-instance support)
 */
void CH9120Driver::rawIrqHandler() {
    // Check ALL instances and handle ALL pending interrupts
    // No early return - we must process every GPIO that triggered
    for (int i = 0; i < NUM_UART_INSTANCES; i++) {
        CH9120Driver* instance = CH9120Driver_instances[i].driver;
        
        if (instance && instance->_initialized) {
            uint status_pin = instance->_statusPin;
            
            // Check if this specific pin triggered an interrupt
            uint32_t events = gpio_get_irq_event_mask(status_pin);
            if (events & (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL)) {
                // Clear the event for this pin only
                gpio_acknowledge_irq(status_pin, events);
                
                // Update link state for this instance
                instance->updateLinkState();
                
                // Continue to check other instances - no break/return!
                // CH9120Debug::logln("[IRQ] Link state changed"); // ONLY uncomment if REALLY needed! Normally not ISR-compliant
            }
        }
    }
}


// ===================================================================================
// CHIP CONFIGURATION METHODS (PRIVATE)
// ===================================================================================


/* @brief Set the configuration
 * @param config The network configuration
 * @return Result code
 */
CH9120Driver::Result CH9120Driver::setConfigUnsafe(const NetworkConfig& config) {
    // State transition: STOPPED -> CONFIG mandatory
    if (getState() != STOPPED) return Error(ERR_STATE, "Driver must be stopped before reconfiguration");
    setState(CONFIG);

    // Validate configuration parameters
    if (config.validate() != SUCCESS) {
        return Error(ERR_CONFIG, "Invalid configuration parameters");
    }

    // Enter in configuration mode
    cfg_enterConfigMode();
    
    // Initialize result storage for deferred error handling
    struct ConfigResult {
        Result code;
        const char* message;
    } res = {SUCCESS, nullptr};

    // Exit this loop at the first error case encountered
    do {
        // Configure all parameters
        if (cfg_setMode(config.mode) != SUCCESS) {
            res = {ERR_CONFIG, "Failed to set transport mode"}; break;
        }
        if (cfg_setLocalIP(config.localIp) != SUCCESS) {
            res = {ERR_CONFIG, "Failed to set local IP"}; break;
        }
        if (cfg_setSubnetMask(config.subnetMask) != SUCCESS) {
            res = {ERR_CONFIG, "Failed to set subnet mask"}; break;
        }
        if (cfg_setGateway(config.gateway) != SUCCESS) {
            res = {ERR_CONFIG, "Failed to set gateway"}; break;
        }
        if (cfg_setTargetIP(config.targetIp) != SUCCESS) {
            res = {ERR_CONFIG, "Failed to set target IP"}; break;
        }
        if (cfg_setLocalPort(config.localPort) != SUCCESS) {
            res = {ERR_CONFIG, "Failed to set local port"}; break;
        }
        if (cfg_setTargetPort(config.targetPort) != SUCCESS) {
            res = {ERR_CONFIG, "Failed to set target port"}; break;
        }
        if (cfg_setBaudRate(_trspBaudRate) != SUCCESS) {
            res = {ERR_CONFIG, "Failed to set baud rate"}; break;
        }
        if (cfg_setDataMode() != SUCCESS) {// Forced to 8N1
            res = {ERR_CONFIG, "Failed to set data mode"}; break;
        }
        if (cfg_setDhcpMode(config.useDhcp) != SUCCESS) {
            res = {ERR_CONFIG, "Failed to set DHCP mode"}; break;
        }

        // Default "hidden" configuration parameters
        if (cfg_setRandomLocalPort(CFG_RANDOM_LOCAL_PORT) != SUCCESS) {
            res = {ERR_CONFIG, "Failed to set Random Local Port mode"}; break;
        }
        if (cfg_setTxPacketLength(CFG_TX_PACKET_LENGTH) != SUCCESS) {
            res = {ERR_CONFIG, "Failed to set TX packet length"}; break;
        }
        if (cfg_setTxTimeout(CFG_TX_TIMEOUT) != SUCCESS) {
            res = {ERR_CONFIG, "Failed to set TX timeout"}; break;
        }
        if (cfg_setLinkDownMode(CFG_DISCONNECT_ON_LINK_DOWN) != SUCCESS) {
            res = {ERR_CONFIG, "Failed to set Link Down mode"}; break;
        }
        if (cfg_setFlushOnConnect(CFG_FLUSH_ON_CONNECT) != SUCCESS) {
            res = {ERR_CONFIG, "Failed to set Flush On Connect mode"}; break;
        }

        if (cfg_finalizeConfig() != SUCCESS) {
            res = {ERR_CONFIG, "Failed to finalize configuration"}; break;
        }

        // res = {SUCCESS, nullptr} if we reach here

    } while(false);

    cfg_exitConfigMode();
    setState(STOPPED); // State transition: CONFIG -> STOPPED

    if (res.code == SUCCESS) {
        _configured = true;
        _currentConfig = config; // Store current configuration
        // updateLinkState(); // Update link state after config change
        return Success();
    } else {
        _configured = false;
        _currentConfig = std::nullopt; // Clear current configuration on error
        return Error(res.code, res.message);
    }
}


/* @brief Send a configuration command
 * @param cmd The command to send
 * @param len The length of the command
 * @return Result code
 */
CH9120Driver::Result CH9120Driver::cfg_sendCommand(const uint8_t* cmd, size_t len) {
    clearRxBuffer(); // Clear RX buffer before sending command
    for (size_t i = 0; i < len; i++) {
        uart_putc(_uart, cmd[i]);
    }
    return cfg_waitForResponse(CFG_RESPONSE_TIMEOUT_MS);
}


/* @brief Wait for a configuration response
 * @param timeoutMs The timeout
 * @return Result code
 */
CH9120Driver::Result CH9120Driver::cfg_waitForResponse(uint32_t timeoutMs) {
    // Enough room to accept 0xAA (official datasheet) or "OK\r\n" (just in case)
    uint8_t buf[4];
    size_t idx = 0;
    uint32_t start = to_ms_since_boot(get_absolute_time());

    while (to_ms_since_boot(get_absolute_time()) - start < timeoutMs) {
        if (uart_is_readable(_uart) && idx < sizeof(buf)) {
            buf[idx] = uart_getc(_uart);
            idx++;
        }

        if (idx && (buf[0] == 0xAA || (idx >= 2 && buf[0] == 'O' && buf[1] == 'K'))) {
            return Success();
        }
        tight_loop_contents();
    }
    return Error(ERR_TIMEOUT);
}


/* @brief Enter configuration mode
 */
void CH9120Driver::cfg_enterConfigMode() {
    // Enable config pin
    pins_setConfig(true);

    // If the chip was down, turn it on
    if (!pins_getEnable()) pins_setEnable(true);
    else vTaskDelay(pdMS_TO_TICKS(CFG_STARTUP_DELAY_MS));

    // Switch to configuration baud rate
    // NB: out of convenience we're bypassing the driver's
    // setBaudrate() method, this is fully OK and tested:
    // - The driver is stopped when we reach this point
    // - This code is only called when holding the mutex
    // - CFG_BAUDRATE is always valid
    // - The driver handles early failure and will revert to
    //   transport baud rate automatically
    uart_set_baudrate(_uart, CFG_BAUDRATE);

    // Clear RX buffer
    clearRxBuffer();
}


/* @brief Set the mode
 * @param mode The mode
 * @return Result code
 */
CH9120Driver::Result CH9120Driver::cfg_setMode(Mode mode) {
    uint8_t cmd[4] = {0x57, 0xAB, 0x10, static_cast<uint8_t>(mode)};
    return cfg_sendCommand(cmd, 4);
}


/* @brief Set the local IP address
 * @param ip The IP address
 * @return Result code
 */
CH9120Driver::Result CH9120Driver::cfg_setLocalIP(const uint8_t ip[4]) {
    uint8_t cmd[7] = {0x57, 0xAB, 0x11, ip[0], ip[1], ip[2], ip[3]};
    return cfg_sendCommand(cmd, 7);
}


/* @brief Set the subnet mask
 * @param mask The subnet mask
 * @return Result code
 */
CH9120Driver::Result CH9120Driver::cfg_setSubnetMask(const uint8_t mask[4]) {
    uint8_t cmd[7] = {0x57, 0xAB, 0x12, mask[0], mask[1], mask[2], mask[3]};
    return cfg_sendCommand(cmd, 7);
}


/* @brief Set the gateway
 * @param gateway The gateway
 * @return Result code
 */
CH9120Driver::Result CH9120Driver::cfg_setGateway(const uint8_t gateway[4]) {
    uint8_t cmd[7] = {0x57, 0xAB, 0x13, gateway[0], gateway[1], gateway[2], gateway[3]};
    return cfg_sendCommand(cmd, 7);
}


/* @brief Set the target IP address
 * @param ip The target IP address
 * @return Result code
 */
CH9120Driver::Result CH9120Driver::cfg_setTargetIP(const uint8_t ip[4]) {
    uint8_t cmd[7] = {0x57, 0xAB, 0x15, ip[0], ip[1], ip[2], ip[3]};
    return cfg_sendCommand(cmd, 7);
}


/* @brief Set the local port
 * @param port The local port
 * @return Result code
 */
CH9120Driver::Result CH9120Driver::cfg_setLocalPort(uint16_t port) {
    uint8_t cmd[5] = {0x57, 0xAB, 0x14, 
                      static_cast<uint8_t>(port & 0xFF),        // LSB
                      static_cast<uint8_t>((port >> 8) & 0xFF)}; // MSB
    return cfg_sendCommand(cmd, 5);
}


/* @brief Set the target port
 * @param port The target port
 * @return Result code
 */
CH9120Driver::Result CH9120Driver::cfg_setTargetPort(uint16_t port) {
    uint8_t cmd[5] = {0x57, 0xAB, 0x16, 
                      static_cast<uint8_t>(port & 0xFF),        // LSB
                      static_cast<uint8_t>((port >> 8) & 0xFF)}; // MSB
    return cfg_sendCommand(cmd, 5);
}


/* @brief Set the baud rate
 * @param baud The baud rate
 * @return Result code
 */
CH9120Driver::Result CH9120Driver::cfg_setBaudRate(uint32_t baud) {
    uint8_t cmd[7] = {0x57, 0xAB, 0x21,
                      static_cast<uint8_t>(baud & 0xFF),        // LSB
                      static_cast<uint8_t>((baud >> 8) & 0xFF),
                      static_cast<uint8_t>((baud >> 16) & 0xFF),
                      static_cast<uint8_t>((baud >> 24) & 0xFF)}; // MSB
    return cfg_sendCommand(cmd, 7);
}


/* @brief Set the data mode
 * @return Result code
 */
CH9120Driver::Result CH9120Driver::cfg_setDataMode() {
    uint8_t cmd[6] = {0x57, 0xAB, 0x22,
                      0x01,  // Stop bit
                      0x04,  // Parity (Even/Odd/Mark/Space/None)
                      0x08}; // Data bits
    return cfg_sendCommand(cmd, 6);
}


/* @brief Set the random local port
 * @param random True if random port, false otherwise
 * @return Result code
 */
CH9120Driver::Result CH9120Driver::cfg_setRandomLocalPort(bool random) {
    uint8_t en = (random) ? 0x01 : 0x00;
    uint8_t cmd[4] = {0x57, 0xAB, 0x17, en};
    return cfg_sendCommand(cmd, 4);
}


/* @brief Set the TX timeout
 * @param timeout The timeout
 * @return Result code
 */
CH9120Driver::Result CH9120Driver::cfg_setTxTimeout(uint8_t timeout) {
    uint8_t cmd[7] = {0x57, 0xAB, 0x23, timeout, 0x00, 0x00, 0x00};
    return cfg_sendCommand(cmd, 7);
}


/* @brief Set the link down mode
 * @param disconnect True if disconnect, false otherwise
 * @return Result code
 */
CH9120Driver::Result CH9120Driver::cfg_setLinkDownMode(bool disconnect) {
    uint8_t disc = (disconnect) ? 0x01 : 0x00;
    uint8_t cmd[4] = {0x57, 0xAB, 0x24, disc};
    return cfg_sendCommand(cmd, 4);
}


/* @brief Set the TX packet length
 * @param length The length
 * @return Result code
 */
CH9120Driver::Result CH9120Driver::cfg_setTxPacketLength(uint16_t length) {
    // Code 0x25 - Serial RX Length (4 bytes little-endian)
    uint8_t cmd[7] = {0x57, 0xAB, 0x25,
                      static_cast<uint8_t>(length & 0xFF),        // LSB first
                      static_cast<uint8_t>((length >> 8) & 0xFF), // MSB second
                      0x00, 0x00};                                // bytes[5..6] = 0
    return cfg_sendCommand(cmd, 7);
}


/* @brief Set the flush on connect
 * @param flush True if flush, false otherwise
 * @return Result code
 */
CH9120Driver::Result CH9120Driver::cfg_setFlushOnConnect(bool flush) {
    uint8_t en = (flush) ? 0x01 : 0x00;
    uint8_t cmd[4] = {0x57, 0xAB, 0x26, en};
    return cfg_sendCommand(cmd, 4);
}


/* @brief Set the DHCP mode
 * @param enabled True if enabled, false otherwise
 * @return Result code
 */
CH9120Driver::Result CH9120Driver::cfg_setDhcpMode(bool enabled) {
    uint8_t en = (enabled) ? 0x01 : 0x00;
    uint8_t cmd[4] = {0x57, 0xAB, 0x33, en};
    return cfg_sendCommand(cmd, 4);
}


/* @brief Finalize the configuration
 * @return Result code
 */
CH9120Driver::Result CH9120Driver::cfg_finalizeConfig() {
    // Save to EEPROM
    uint8_t cmd[3] = {0x57, 0xAB, 0x0D};
    Result result = cfg_sendCommand(cmd, 3);
    if (result != SUCCESS) return Error(result);
    
    // Apply configuration & reboot
    cmd[2] = 0x0E;
    result = cfg_sendCommand(cmd, 3);
    if (result != SUCCESS) return Error(result);
    
    // Wait for startup
    vTaskDelay(pdMS_TO_TICKS(CFG_STARTUP_DELAY_MS));
    
    // No need to send 0x5E cause CH9120 already rebooted
    return Success();
}


/* @brief Exit configuration mode
 */
void CH9120Driver::cfg_exitConfigMode() {
    // Put back CFG pin to run mode
    pins_setConfig(false);
    
    // Switch to transport baud rate
    // NB: out of convenience we're bypassing the driver's
    // setBaudrate() method, this is fully OK and tested:
    // - The driver is stopped when we reach this point
    // - This code is only called when holding the mutex
    // - _trspBaudRate was validated at initialization
    // - This function "cannot fail" and is called whatever
    //   the outcome of the configuration process
    uart_set_baudrate(_uart, _trspBaudRate);
    
    // Empty buffer (just in case)
    clearRxBuffer();
    
    // Note: UartDmaDriver will be restarted by startRxTx() when called by user
}


// ===================================================================================
// PINS MANAGEMENT HELPERS
// ===================================================================================

/* @brief Set the enable pin
 * @param enable True if enable, false otherwise
 */
inline void CH9120Driver::pins_setEnable(bool enable) {
    gpio_put(_resPin, enable ? 1 : 0); // Enabled in HIGH state

    // Wait for stabilization depending on new state
    if (enable) vTaskDelay(pdMS_TO_TICKS(CFG_STARTUP_DELAY_MS));
    else vTaskDelay(pdMS_TO_TICKS(CFG_PIN_SET_DELAY_MS));
}


/* @brief Get the enable pin
 * @return True if enable, false otherwise
 */
inline bool CH9120Driver::pins_getEnable() const { 
    return gpio_get(_resPin) == 1; // Enabled in HIGH state
}


/* @brief Set the config pin
 * @param cfg True if config, false otherwise
 */
inline void CH9120Driver::pins_setConfig(bool cfg) {
    gpio_put(_cfgPin, cfg ? 0 : 1); // Config mode in LOW state
    vTaskDelay(pdMS_TO_TICKS(CFG_PIN_SET_DELAY_MS));
}


/* @brief Get the config pin
 * @return True if config, false otherwise
 */
inline bool CH9120Driver::pins_getConfig() const { 
    return gpio_get(_cfgPin) == 0; // Config mode in LOW state
}


/* @brief Get the connection state pin status
 * @return True if connected, false otherwise
 */
inline bool CH9120Driver::pins_getConnected() const { 
    return gpio_get(_statusPin) == 0; // Connected in LOW state
}