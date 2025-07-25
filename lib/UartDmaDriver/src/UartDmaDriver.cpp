/* @file UartDmaDriver.cpp */

#include "UartDmaDriver.hpp"
#include "pico/stdlib.h"
#include "pico/platform.h"
#include "hardware/irq.h"
#include "FreeRTOS.h"
#include "task.h"
#include <cstring>
#include <cstdio>
#include <algorithm>


// ===================================================================================
// COMPILE-TIME CHECKS
// ===================================================================================


/*@brief Static assertions to ensure configuration correctness
 */
namespace UartDmaDriver_Assert {

// Max bytes that can arrive in one watchdog tick at max baud
constexpr uint32_t maxBytesPerTick = 
    ((UartDmaDriver::RX_MAX_BAUDRATE / 10) * UART_WD_TICK_US + 999'999) / 1'000'000;

// ASSERT 1 - DMA buffer size check
static_assert(UART_DMA_RING_BITS >= 9 && UART_DMA_RING_BITS <= 15,
              "UART_DMA_RING_BITS should be 9-15 (2^9 to 2^15 = 512-32768 bytes RX buffer)");

// ASSERT 2 - Buffer overflow detection capability check
static_assert(
    UartDmaDriver::DMA_OVERFLOW_WATERMARK >= 3 * maxBytesPerTick,
    "CONFIG WARNING: UART_WD_TICK_US too long for max baudrate; "
    "DMA margin (watermark) too small to stop before overwrite."
);

// ASSERT 3 - Queue size check
constexpr size_t maxQueueStorage = UART_RX_MAX_EVT_DATA_SIZE * (UART_EVENT_Q_SIZE - 2);
static_assert(
    maxQueueStorage > UartDmaDriver::DMA_RX_BUFFER_SIZE,
    "CONFIG WARNING: queue can hold less than the DMA buffer size + margin; "
    "increase UART_EVENT_Q_SIZE or UART_RX_MAX_EVT_DATA_SIZE"
);

// ASSERT 4 - Max chunk size check
static_assert(
    UART_RX_MAX_EVT_DATA_SIZE > 3 * maxBytesPerTick,
    "CONFIG WARNING: throughput may be too high for the selected chunk size; "
    "increase UART_RX_MAX_EVT_DATA_SIZE or decrease UART_WD_TICK_US."
);

} // namespace UartDmaDriver_Assert


// ===================================================================================
// STATIC INSTANCES & GLOBAL HANDLERS
// ===================================================================================


/*@brief Static instances for handlers
 *
 * This struct holds the DMA channel and the driver instance for each instance.
 * It is used to support multiple instances of the UartDmaDriver.
 * @note Supports max 2 instances
 */
struct InstanceSlot {
    int dmaChannel;
    UartDmaDriver* driver;
};

static InstanceSlot UartDmaDriver_instances[2] = {{-1, nullptr}, {-1, nullptr}};

// NOUVELLE FONCTION - Installation IRQ safe et unique
#ifndef UART_DMA_DRIVER_NO_INSTALL_IRQ
static void UartDmaDriver_ensureDmaIrqInstalled() {
    static bool s_irq_installed = false;
    
    if (!s_irq_installed) {
        // Mode partagé : compatible avec autres libs DMA
        irq_add_shared_handler(DMA_IRQ_0, UartDmaDriver_dmaIrqCb, 
                               PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
        irq_set_enabled(DMA_IRQ_0, true);
        s_irq_installed = true;
    }
}
#endif


/*@brief Global timer callback wrapper
 *
 * This function is called when the watchdog timer expires.
 * It is used to handle the overflow event.
 */
bool UartDmaDriver_rxWatchdogTimerCb(struct repeating_timer *t) {
    UartDmaDriver* self = static_cast<UartDmaDriver*>(t->user_data);
    if (!self) return false;
    return self->onRxWatchdogTimer();
}


/*@brief Global DMA IRQ0 wrapper
 *
 * This function is called when the DMA IRQ0 is triggered.
 * It is used to handle the RX or TX channel.
 */
void UartDmaDriver_dmaIrqCb() {
    // Check all registered instances to find which RX or TX channel triggered
    for (int i = 0; i < UartDmaDriver::UART_MAX_INSTANCES; i++) {
        if (UartDmaDriver_instances[i].driver && UartDmaDriver_instances[i].dmaChannel >= 0) {
            UartDmaDriver* driver = UartDmaDriver_instances[i].driver;
            
            // Check RX channel (stored in instances array)
            int rx_channel = UartDmaDriver_instances[i].dmaChannel;
            if (dma_channel_get_irq0_status(rx_channel)) {
                driver->onRxDmaCplt();
            }
            
            // Check TX channel
            int tx_channel = driver->getTxDmaChannel();
            if (tx_channel >= 0 && dma_channel_get_irq0_status(tx_channel)) {
                driver->onTxDmaCplt();
            }
        }
    }
}


// ===================================================================================
// CTOR/DTOR
// ===================================================================================


/*@brief Constructor (creates an instance of UartDmaDriver)
 */
UartDmaDriver::UartDmaDriver(uart_inst_t* uart, uint8_t txPin, uint8_t rxPin, 
                   uint32_t baudRate, uint dataBits, uint stopBits, uart_parity_t parity) : 
    _uart(uart), 
    _txPin(txPin), 
    _rxPin(rxPin), 
    _baudRate(baudRate),
    _dataBits(dataBits),
    _stopBits(stopBits),
    _parity(parity),
    _initialized(false),
    _stateUnsafe(STATE_OFF),
    _eventQueue(nullptr),
    _queueDropped(0),
    _dmaRxChannel(-1), 
    _dmaReadPos(0),
    _dmaLastPushedPos(0),
    _lastDmaWritePos(0),
    _dmaTxChannel(-1),
    _watchdogTimerHandle(),
    _silenceTicks(0),
    _watchdogTickUs(UART_WD_TICK_US),              // Initialize with compile-time defaults
    _watchdogSilenceTicks(UART_WD_SILENCE_TICKS)   // Initialize with compile-time defaults
{
    // Constructor validation happens at object creation
    // Validation errors will be caught during init() call
}


/*@brief Destructor (cleans up UartDmaDriver)
 */
UartDmaDriver::~UartDmaDriver() {
    // Complete cleanup: stop hardware then clean up resources
    stopDMA();  // Stops DMA + timer
    
    // Release global instance slot BEFORE unclaiming channel
    if (_dmaRxChannel >= 0) {
        for (int i = 0; i < UART_MAX_INSTANCES; i++) {
            if (UartDmaDriver_instances[i].dmaChannel == _dmaRxChannel) {
                UartDmaDriver_instances[i].dmaChannel = -1;
                UartDmaDriver_instances[i].driver = nullptr;
                break;
            }
        }
    }

    // Release DMA channels if claimed
    if (_dmaRxChannel >= 0) {
        dma_channel_unclaim(_dmaRxChannel);
        _dmaRxChannel = -1;
    }
    if (_dmaTxChannel >= 0) {
        dma_channel_unclaim(_dmaTxChannel);
        _dmaTxChannel = -1;
    }
    
    // TX semaphore is automatically deleted by BinarySemaphore destructor
    // FreeRTOS queue is static -> no call vQueueDelete needed
}


// ===================================================================================
// PUBLIC API METHODS
// ===================================================================================


/*@brief Initialize the driver (validates parameters, inits DMA, registers instance)
 */
UartDmaDriver::Result UartDmaDriver::init() {
    if (_initialized) return Success();

    // Validate UART parameters
    if (_baudRate == 0 || _baudRate > RX_MAX_BAUDRATE) {
        return Error(ERR_CONFIG, "Invalid baud rate");
    }
    if (!isStandardBaudrate(_baudRate)) {
        return Error(ERR_CONFIG, "Non-standard baud rate - use 9600, 19200, 38400, 57600, 115200, 230400, 460800, or 921600");
    }
    if (_dataBits < 5 || _dataBits > 8) {
        return Error(ERR_CONFIG, "Invalid data bits (must be 5-8)");
    }
    if (_stopBits < 1 || _stopBits > 2) {
        return Error(ERR_CONFIG, "Invalid stop bits (must be 1 or 2)");
    }
    if (_parity != UART_PARITY_NONE && _parity != UART_PARITY_EVEN && _parity != UART_PARITY_ODD) {
        return Error(ERR_CONFIG, "Invalid parity setting");
    }

    // Initialize DMA first to get channel for multi-instance support
    if (initDMA() != SUCCESS) return Error(ERR_UART, "Failed to initialize DMA");
    
    // Register this instance in a free slot
    bool slotFound = false;
    for (int i = 0; i < UART_MAX_INSTANCES; i++) {
        if (UartDmaDriver_instances[i].driver == nullptr) {
            UartDmaDriver_instances[i].dmaChannel = _dmaRxChannel;
            UartDmaDriver_instances[i].driver = this;
            slotFound = true;
            break;
        }
    }
    
    if (!slotFound) {
        return Error(ERR_CONFIG, "Too many UartDmaDriver instances (max 2)");
    }
 
    // Configure UART with user parameters
    uart_init(_uart, _baudRate);
    uart_set_format(_uart, _dataBits, _stopBits, _parity);
    gpio_set_function(_txPin, GPIO_FUNC_UART);
    gpio_set_function(_rxPin, GPIO_FUNC_UART);
    
    // Disable FIFO for immediate DREQ and atomic DMA transfers
    uart_set_fifo_enabled(_uart, false);
    
    // Clear any residual data from UART buffers
    clearRxBuffer();
    
    _initialized = true;
    return Success();
}


/*@brief Start or resume transmission (starts DMA, resets RX buffer, resets state)
 */
UartDmaDriver::Result UartDmaDriver::start() {
    if (!_initialized) return Error(ERR_NOT_INITIALIZED);
    if (getRunState() != STATE_OFF) return Error(ERR_STATE, "Driver must be stopped before starting RX");

    Lock guard(_drvMutex);

    // Reset state (flush queue + DMA bookmarks)
    Event dummy;
    while(xQueueReceive(_eventQueue, &dummy, 0) == pdPASS) { }
    resetDMABookmarks();

    // Start DMA and timer
    Result result = startDMA();
    if (result != SUCCESS) {
        setRunState(STATE_OFF); // Failed to start - ensure we remain in safe STATE_OFF state
        return Error(result);
    }

    setRunState(STATE_RUN_OK);
    
    return Success();
}


/*@brief Stop the driver (stops DMA, resets state)
 */
UartDmaDriver::Result UartDmaDriver::stop() {
    if (getRunState() == STATE_OFF) return Success(); // Already stopped
    if (!_initialized) return Error(ERR_NOT_INITIALIZED);

    Lock guard(_drvMutex);

    // Stop DMA and timer
    stopDMA();
    
    // Reset state
    setRunState(STATE_OFF);
    
    return Success();
}

/*@brief Set watchdog timer configuration
* @param tickUs: Watchdog timer tick period in microseconds (100-2000)
* @param silenceTicks: Number of ticks to consider as silence (2-20)
* @return SUCCESS if configuration was set, error otherwise
* @note Driver must be stopped (STATE_OFF) to change timing
*/
UartDmaDriver::Result UartDmaDriver::setWatchdogTiming(uint32_t tickUs, uint8_t silenceTicks) {
    if (getState() != STATE_OFF) {
        return Error(ERR_STATE, "Cannot change timing while driver is running");
    }
    
    // Basic parameter validation
    if (tickUs < MIN_RUNTIME_WD_TICK_US || tickUs > MAX_RUNTIME_WD_TICK_US) {
        return Error(ERR_CONFIG, "tickUs must be 100-10000 microseconds");
    }
    if (MIN_RUNTIME_WD_SILENCE_TICKS < 2 || MAX_RUNTIME_WD_SILENCE_TICKS > 20) {
        return Error(ERR_CONFIG, "silenceTicks must be 2-20 ticks");
    }
    
    // Validate against overflow safety (same logic as static assert)
    uint32_t maxBytesPerTick = ((_baudRate / 10) * tickUs + 999999) / 1000000;
    if (DMA_OVERFLOW_WATERMARK < 3 * maxBytesPerTick) {
        return Error(ERR_CONFIG, "Tick time too long for current baud rate - reduce tickUs");
    }
    if (EVTQ_MAX_EVT_DATA_SIZE < 3 * maxBytesPerTick) {
        return Error(ERR_CONFIG, "Chunk size too small for current baud rate - reduce tickUs");
    }
    
    // Store new timing configuration (applied on next start())
    _watchdogTickUs = tickUs;
    _watchdogSilenceTicks = silenceTicks;
    
    return Success();
}

/*@brief Set UART baud rate at runtime (hot change when safe)
* @param baudRate: New baud rate (must be standard)  
* @return SUCCESS if baud rate was changed, error otherwise
* @note Safe to call during operation - waits for TX completion, protects RX
*/
UartDmaDriver::Result UartDmaDriver::setBaudrate(uint32_t baudRate) {
    // Validate standard baud rate first
    if (!isStandardBaudrate(baudRate)) {
        return Error(ERR_CONFIG, "Non-standard baud rate - use 9600, 19200, 38400, 57600, 115200, 230400, 460800, or 921600");
    }
    
    // Reject change if higher than configured maximum - ensures timing safety contract
    // still applies even if the baud rate is valid
    if (baudRate > RX_MAX_BAUDRATE) {
        return Error(ERR_CONFIG, "Baud rate exceeds maximum");
    }
    
    // Critical section for thread safety
    Lock guard(_drvMutex);
    
    if (!_initialized) {
        return Error(ERR_NOT_INITIALIZED, "Driver not initialized");
    }
    
    // If driver is running, ensure safe state for baud rate change
    State currentState = getRunState();
    if (currentState != STATE_OFF) {
        return Error(ERR_CONFIG, "Driver must be stopped before changing baud rate");
    } else {
        // Driver stopped - just change the hardware setting
        uart_set_baudrate(_uart, baudRate);
        _baudRate = baudRate; // Update stored baud rate
    }
    
    return Success();
}

/*@brief Check if a baud rate is in the list of standard rates
* @param baudRate: Baud rate to validate
* @return true if baud rate is standard, false otherwise
*/
bool UartDmaDriver::isStandardBaudrate(uint32_t baudRate) {
    for (size_t i = 0; i < NUM_STANDARD_BAUDRATES; i++) {
        if (STANDARD_BAUDRATES[i] == baudRate) {
            return true;
        }
    }
    return false;
}


/*@brief Check if the driver is overflowed
* @return `true` if the driver is overflowed, `false` otherwise
 */
bool UartDmaDriver::isOverflowed() const {
    return (getRunState() == STATE_RUN_OVF);
}


/*@brief Get the current state of the driver
* @return The current state of the driver
 */
UartDmaDriver::State UartDmaDriver::getState() const {
    return getRunState();
}


/*@brief Send data to the driver
* @param data The data to send
* @param len The length of the data to send
* @param waitTicks The number of ticks to wait for TX to be available (yields while waiting)
* @return The result of the send operation
* @note The driver must be running to send data
 */
UartDmaDriver::Result UartDmaDriver::send(const uint8_t* data, size_t len, TickType_t waitTicks) {
    if (!_initialized) return Error(ERR_NOT_INITIALIZED);
    if (_dmaTxChannel == -1) return Error(ERR_UART, "TX DMA not initialized");
    
    // Fail if driver is not running
    if (getRunState() == STATE_OFF) {
        return Error(ERR_STATE, "TX disabled when driver is stopped");
    }
    
    if (len == 0) return Success(); // Nothing to send
    if (len > DMA_TX_BUFFER_SIZE) return Error(ERR_TX_OVERFLOW);

    // 1. Check if we can proceed (try-lock no wait)
    Lock guard(_drvMutex, 0);
    if (!guard.isLocked()) {
        return Error(ERR_BUSY, "Driver operation in progress");
    }
    
    // 2. Wait for TX to be available (only if we can proceed)
    if (!_txDmaSemaphore.take(waitTicks)) {
        return Error(ERR_TX_BUSY, "TX busy for too long");
    }
    
    // Copy data to DMA buffer
    memcpy(_dmaTxBuffer, data, len);
    
    // Start DMA transfer (semaphore will be given back by IRQ handler)
    dma_channel_set_trans_count(_dmaTxChannel, len, false);
    dma_channel_set_read_addr(_dmaTxChannel, _dmaTxBuffer, true); // Start transfer
    
    return Success();
}


/*@brief Check if the TX is busy
* @return `true` if TX currently in progress, `false` otherwise
 */
bool UartDmaDriver::isTxBusy() {
    // If semaphore can be taken immediately (non-blocking), TX is not busy
    if (_txDmaSemaphore.tryTake()) {
        _txDmaSemaphore.give(); // Give it back immediately
        return false;
    }
    return true; // Could not take semaphore = TX is busy
}


/*@brief Pop an event from the event queue
* @param outEvent The event to pop
* @param waitTicks The number of ticks to wait for an event (yields while waiting)
* @return `true` if an event was popped, `false` otherwise
 */
bool UartDmaDriver::popEvent(Event* outEvent, TickType_t waitTicks) {
    if (!_initialized || !outEvent) return false;
    
    // Yield until an event is available or timeout occurs
    if (xQueueReceive(_eventQueue, outEvent, waitTicks) == pdPASS) {
        return true;  // Event received
    }
    return false;  // Timeout - no event available
}


/*@brief Read data from the RX buffer
* @param dst The destination buffer to copy the data to
* @param maxLen The maximum number of bytes to read
* @return The number of bytes read
 */
size_t UartDmaDriver::read(uint8_t* dst, size_t maxLen) {
    if (!_initialized) return 0; // Not initialized
    if (maxLen == 0) return 0;
    
    // Calculate available data in ring buffer
    size_t write_pos = getDmaWritePos();
    size_t available_data = (write_pos - _dmaReadPos) & (DMA_RX_BUFFER_SIZE - 1);
    
    // If data is available, process it
    size_t to_read = 0;
    if (available_data > 0) {
        to_read = std::min(maxLen, available_data);
        
        // Handle ring buffer wrap-around
        size_t firstChunk = std::min(to_read, DMA_RX_BUFFER_SIZE - _dmaReadPos);
        memcpy(dst, &_dmaRxBuffer[_dmaReadPos], firstChunk);
        
        if (to_read > firstChunk) {
            memcpy(dst + firstChunk, &_dmaRxBuffer[0], to_read - firstChunk);
        }
        
        // Always advance read bookmark
        _dmaReadPos = (_dmaReadPos + to_read) & (DMA_RX_BUFFER_SIZE - 1);
    }
    
    // AUTO-RECOVERY IN CASE OF STATE_RUN_OVF - always executed even if no data was read
    if (getRunState() == STATE_RUN_OVF) {
        size_t currentWritePos = getDmaWritePos(); // Position is frozen during overflow
        size_t currentUsedSpace = (currentWritePos - _dmaReadPos) & (DMA_RX_BUFFER_SIZE - 1);
        size_t currentFreeSpace = DMA_RX_BUFFER_SIZE - currentUsedSpace;
        
        // Ensure we recover only if we have room for 50% of the buffer (arbitrary threshold)
        constexpr size_t dataRecoveryThreshold = DMA_RX_BUFFER_SIZE / 2;
        bool isBufferPurged = (currentFreeSpace >= dataRecoveryThreshold);
        if (isBufferPurged && compareAndSetRunState(STATE_RUN_OVF, STATE_RUN_OK)) {
            // Attempt to switch from STATE_RUN_OVF to STATE_RUN_OK
            Result result = startDMA();
            
            if (result != SUCCESS) {
                setRunState(STATE_RUN_OVF);  // Rollback if DMA fails to start
            }
        }
    }
    
    return to_read;
}

size_t UartDmaDriver::available() const {
    if (!_initialized) return 0;
    size_t write_pos = getDmaWritePos();
    return (write_pos - _dmaReadPos) & (DMA_RX_BUFFER_SIZE - 1);
}


// ===================================================================================
// PRIVATE METHODS
// ===================================================================================


/*@brief Clear the RX harware buffer
 */
void UartDmaDriver::clearRxBuffer() {
    while (uart_is_readable(_uart)) {
        uart_getc(_uart);
        tight_loop_contents();
    }
}


/*@brief Get the DMA write position
* @return The DMA write position
 */
inline size_t UartDmaDriver::getDmaWritePos() const {
    return (dma_hw->ch[_dmaRxChannel].write_addr - 
            (uintptr_t)_dmaRxBuffer) & (DMA_RX_BUFFER_SIZE - 1);
}


/*@brief Initialize the DMA
* @return The result of the initialization
 */
UartDmaDriver::Result UartDmaDriver::initDMA() {
    // No need to check global instance since we support multiple instances now
    
    // If queue already exists (re-entrance), just flush it
    // Static allocation so no vQueueDelete needed.
    if (!_eventQueue) {
        _eventQueue = xQueueCreateStatic(EVTQ_SIZE, sizeof(Event), _eventQueueStorage, &_eventQueueBuffer);
        if (!_eventQueue) {
            return Error(ERR_CONFIG, "Failed to create receive queue");
        }
    } else {
        Event dummy;
        while (xQueueReceive(_eventQueue, &dummy, 0) == pdPASS) { }
    }
    
    // Initialize all pointers to their clean state
    // This function is called only once per configuration
    resetDMABookmarks();
    
    // Claim DMA channels
    _dmaRxChannel = dma_claim_unused_channel(false);
    if (_dmaRxChannel == -1) {
        vQueueDelete(_eventQueue);
        _eventQueue = nullptr;
        return Error(ERR_UART, "No RX DMA channel available");
    }
    
    _dmaTxChannel = dma_claim_unused_channel(false);
    if (_dmaTxChannel == -1) {
        dma_channel_unclaim(_dmaRxChannel);
        _dmaRxChannel = -1;
        vQueueDelete(_eventQueue);
        _eventQueue = nullptr;
        return Error(ERR_UART, "No TX DMA channel available");
    }
    
    return Success();
}


/*@brief Start the DMA
* @return The result of the start operation
 */
UartDmaDriver::Result UartDmaDriver::startDMA() {
    // Ensure DMA channels are set
    if (_dmaRxChannel == -1 || _dmaTxChannel == -1) return Error(ERR_UART, "DMA channels not initialized");

    // Stop any running DMA & timer
    stopDMA();
    
    // Configure RX DMA channel
    dma_channel_config rx_config = dma_channel_get_default_config(_dmaRxChannel);
    channel_config_set_transfer_data_size(&rx_config, DMA_SIZE_8);
    channel_config_set_read_increment(&rx_config, false);  // UART register fixed
    channel_config_set_write_increment(&rx_config, true);  // Buffer increment
    channel_config_set_ring(&rx_config, true, DMA_RING_BITS);  // 2KB ring buffer
    channel_config_set_dreq(&rx_config, (_uart == uart0) ? DREQ_UART0_RX : DREQ_UART1_RX);

    // Configure TX DMA channel
    dma_channel_config tx_config = dma_channel_get_default_config(_dmaTxChannel);
    channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_8);
    channel_config_set_read_increment(&tx_config, true);   // Buffer increment
    channel_config_set_write_increment(&tx_config, false); // UART register fixed
    channel_config_set_dreq(&tx_config, (_uart == uart0) ? DREQ_UART0_TX : DREQ_UART1_TX);

    // Enable IRQ to auto-restart transfer on completion and handle TX completion
#ifndef UART_DMA_DRIVER_NO_INSTALL_IRQ
    UartDmaDriver_ensureDmaIrqInstalled();
#else
    // Mode manuel : l'utilisateur doit installer l'IRQ lui-même
    // Voir documentation pour appeler UartDmaDriver_dmaIrqCb depuis son handler
#endif
    dma_channel_set_irq0_enabled(_dmaRxChannel, true);
    dma_channel_set_irq0_enabled(_dmaTxChannel, true);
    
    // Configure and start RX DMA
    dma_channel_configure(_dmaRxChannel, &rx_config,
                         _dmaRxBuffer,
                         &uart_get_hw(_uart)->dr,
                         SIZE_MAX,
                         true);               // Start immediately
                         
    // Configure TX DMA (but don't start it yet - will be started by send())
    dma_channel_configure(_dmaTxChannel, &tx_config,
                         &uart_get_hw(_uart)->dr,
                         _dmaTxBuffer,
                         0,            // Length will be set by send()
                         false);              // Don't start yet
    
    // Create a new clean timer with configurable tick period
    bool addTimerRes = add_repeating_timer_us(
        _watchdogTickUs,
        UartDmaDriver_rxWatchdogTimerCb,
        this,
        &_watchdogTimerHandle
    );
    
    if (!addTimerRes) {
        dma_channel_abort(_dmaRxChannel);
        return Error(ERR_CONFIG, "Failed to start watchdog timer");
    }
    
    return Success();
}


/*@brief Stop the DMA
 */
inline void UartDmaDriver::stopDMA() {
    if (_dmaRxChannel == -1) return;

    // Stop RX DMA
    dma_channel_set_irq0_enabled(_dmaRxChannel, false);
    dma_channel_acknowledge_irq0(_dmaRxChannel);
    dma_channel_abort(_dmaRxChannel);
    
    // Stop TX DMA if initialized
    if (_dmaTxChannel >= 0) {
        dma_channel_set_irq0_enabled(_dmaTxChannel, false);
        dma_channel_acknowledge_irq0(_dmaTxChannel);
        dma_channel_abort(_dmaTxChannel);
        _txDmaSemaphore.giveForce();
    }

    // Cancel the watchdog timer
    cancel_repeating_timer(&_watchdogTimerHandle);
    uint32_t wdGraceTimeMs = (_watchdogTickUs * (_watchdogSilenceTicks + 1)) / 1000 + 1;
    
    // Let a small delay to finish any pending WD operations (only if scheduler running)
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        vTaskDelay(pdMS_TO_TICKS(wdGraceTimeMs));
    }
}


/*@brief Reset the DMA bookmarks
 */
inline void UartDmaDriver::resetDMABookmarks() {
    _dmaReadPos = 0;
    _dmaLastPushedPos = 0;
    _lastDmaWritePos = 0;
    _silenceTicks = 0;
    _queueDropped = 0;
}


/*@brief Push an event to the event queue
* @param type The event type
* @param size The size of data chunk (`EVT_DATA` event)
* @param silence The silence flag of the event (`EVT_DATA` event)
* @return `true` if the event was pushed, `false` otherwise
 */
inline bool UartDmaDriver::pushEvent(EventType type, size_t size, bool silence) {
    if (!_initialized || !_eventQueue) return false;
    
    Event event;
    event.type = type;
    event.timestampUs = time_us_64();
    event.size = size;
    event.silenceFlag = silence;
    
    // Send from normal context (non-blocking)
    BaseType_t result = xQueueSend(_eventQueue, &event, 0);

    return (result == pdPASS);
}


/*@brief Push an event to the event queue from ISR
* @param type The event type
* @param size The size of data chunk (`EVT_DATA` event)
* @param silence The silence flag of the event (`EVT_DATA` event)
* @param woken The woken flag to update
* @return `true` if the event was pushed, `false` otherwise
 */
inline bool UartDmaDriver::pushEventFromISR(EventType type, size_t size, bool silence, BaseType_t* woken) {
    if (!_initialized || !_eventQueue) return false;
    
    Event event;
    event.type = type;
    event.timestampUs = time_us_64();
    event.size = size;
    event.silenceFlag = silence;
    
    // Send from ISR (non-blocking) - no yield here!
    BaseType_t local_woken = pdFALSE;
    BaseType_t result = xQueueSendFromISR(_eventQueue, &event, &local_woken);
    
    // Update caller's woken flag
    if (local_woken == pdTRUE) {
        *woken = pdTRUE;
    }
    
    return (result == pdPASS);
}


/*@brief Handle the RX watchdog timer
* @return `true` if the timer should keep running, `false` otherwise
 */
bool UartDmaDriver::onRxWatchdogTimer() {
    // If we're in overflow state, do nothing until app drains the buffer
    if (getRunStateFromISR() == STATE_RUN_OVF) {
        return true; // Keep timer active for recovery detection
    }

    size_t writePos = getDmaWritePos();
    BaseType_t woken = pdFALSE;  // Local aggregated woken flag

    // Check ring buffer space & capacity before proceeding
    size_t usedSpace = (writePos - _dmaReadPos) & (DMA_RX_BUFFER_SIZE - 1);
    size_t freeSpace = DMA_RX_BUFFER_SIZE - usedSpace;

    // 1. OVERFLOW DETECTION (Ring DMA overflow -> stop production)

    bool isRingDMAFull = (freeSpace <= DMA_OVERFLOW_WATERMARK);
    if (isRingDMAFull) {
        // Emergency stop: switch to STATE_RUN_OVF state & stop DMA immediately
        if (compareAndSetRunStateFromISR(STATE_RUN_OK, STATE_RUN_OVF)) {
            dma_channel_abort(_dmaRxChannel); 
        
            // Send overflow event to application (best-effort)
            if (!pushEventFromISR(EVT_OVERFLOW, 0, false, &woken)) {
                _queueDropped = _queueDropped + 1; // Increment dropped events counter
                return true; // Keep timer running
            };
        
            // Hand over control to application - single yield at end
            portYIELD_FROM_ISR(woken);
            return true; // Keep timer running
        }
    }

    // 2. NORMAL OPERATION (process DMA data)
    
    // Check if DMA produced new data since last write position
    if (writePos == _lastDmaWritePos) {
        _silenceTicks++;
    } else {
        _lastDmaWritePos = writePos;
        _silenceTicks = 0;
    }

    // Check & notify data chunks with unified logic
    size_t available = (writePos - _dmaLastPushedPos) & (DMA_RX_BUFFER_SIZE - 1);
    
    // Determine if we should push a chunk based on silence or target size
    bool shouldPushChunk = false;
    size_t chunkSize = 0;
    bool isSilence = false; // Track silence state

    if (_silenceTicks >= _watchdogSilenceTicks) {
        // Silence detected - push any available data (partial chunk for reactivity)
        if (available > 0) {
            shouldPushChunk = true;
            chunkSize = std::min(available, EVTQ_MAX_EVT_DATA_SIZE);
            isSilence = true;
        }
        _silenceTicks = 0; // Reset for next frame
    } else if (available >= EVTQ_MAX_EVT_DATA_SIZE) {
        // No silence - push all available data if we have at least target size
        shouldPushChunk = true;
        chunkSize = std::min(available, EVTQ_MAX_EVT_DATA_SIZE);
    }
    
    // Push exactly one chunk per watchdog call (best-effort)
    if (shouldPushChunk) {
        if (!pushEventFromISR(EVT_DATA, chunkSize, isSilence, &woken)) {
            _queueDropped = _queueDropped + 1; // Increment dropped events counter
        }
        _dmaLastPushedPos = (_dmaLastPushedPos + chunkSize) & (DMA_RX_BUFFER_SIZE - 1);
    }

    // Hand over to application - single yield at end with aggregated woken flag
    portYIELD_FROM_ISR(woken);
    return true; // Keep timer running
}


/*@brief Handle the RX DMA completion
 */
void UartDmaDriver::onRxDmaCplt() {
    if (!dma_channel_get_irq0_status(_dmaRxChannel)) return;
    dma_channel_acknowledge_irq0(_dmaRxChannel);
    dma_channel_set_trans_count(_dmaRxChannel, SIZE_MAX, true);
}


/*@brief Handle the TX DMA completion
 */
void UartDmaDriver::onTxDmaCplt() {
    if (!dma_channel_get_irq0_status(_dmaTxChannel)) return;
    dma_channel_acknowledge_irq0(_dmaTxChannel);
    
    // TX DMA transfer completed - give semaphore back (from ISR)
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    _txDmaSemaphore.giveFromISR(&xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


// ===================================================================================
// ATOMIC ACCESS HELPERS FOR STATE (SHARED RESOURCE)
// ===================================================================================


/*@brief Get the current state of the driver
* @return The current state of the driver
 */
inline UartDmaDriver::State UartDmaDriver::getRunState() const {
    taskENTER_CRITICAL();
    State state = _stateUnsafe;
    taskEXIT_CRITICAL();
    return state;
}


/*@brief Set the state of the driver
* @param newState The new state of the driver
 */
inline void UartDmaDriver::setRunState(State newState) {
    taskENTER_CRITICAL();
    _stateUnsafe = newState;
    taskEXIT_CRITICAL();
}


/*@brief Compare and set the state of the driver
* @param expectedState The expected state of the driver
* @param newState The new state of the driver
* @return `true` if the state was changed, `false` otherwise
 */
inline bool UartDmaDriver::compareAndSetRunState(State expectedState, State newState) {
    taskENTER_CRITICAL();
    bool changed = (_stateUnsafe == expectedState);
    if (changed) {
        _stateUnsafe = newState;
    }
    taskEXIT_CRITICAL();
    return changed;
}


/*@brief Get the current state of the driver from ISR
* @return The current state of the driver
 */
inline UartDmaDriver::State UartDmaDriver::getRunStateFromISR() const {
    UBaseType_t savedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
    State state = _stateUnsafe;
    taskEXIT_CRITICAL_FROM_ISR(savedInterruptStatus);
    return state;
}


/*@brief Set the state of the driver from ISR
* @param newState The new state of the driver
 */
inline void UartDmaDriver::setRunStateFromISR(State newState) {
    UBaseType_t savedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
    _stateUnsafe = newState;
    taskEXIT_CRITICAL_FROM_ISR(savedInterruptStatus);
}


/*@brief Compare and set the state of the driver from ISR
* @param expectedState The expected state of the driver
* @param newState The new state of the driver
* @return `true` if the state was changed, `false` otherwise
 */
inline bool UartDmaDriver::compareAndSetRunStateFromISR(State expectedState, State newState) {
    UBaseType_t savedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
    bool changed = (_stateUnsafe == expectedState);
    if (changed) {
        _stateUnsafe = newState;
    }
    taskEXIT_CRITICAL_FROM_ISR(savedInterruptStatus);
    return changed;
}