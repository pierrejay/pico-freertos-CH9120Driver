/* @file UartDmaDriver.hpp */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <cstdio>
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "../include/FreeRTOS_Types.hpp"
#include "pico/time.h"
#include <optional>

// ====================================================================================
// CONFIGURATION OVERVIEW
//
// The driver will create a queue to store events (data received, overflow detection).
// It will spin up a "watchdog" to monitor the continuous flow of data coming into the DMA buffer.
// - The queue can hold UART_EVENT_Q_SIZE events & the watchdog runs at UART_WD_TICK_US intervals
// - The "data received" events will contain at most UART_RX_MAX_EVT_DATA_SIZE bytes
//
// To be reactive & efficient with predictable data chunk size & timing, the following process is observed:
// - If the watchdog detects a full DMA buffer, it will push an OVERFLOW event to the queue and switch
//   the driver immediately to STATE_RUN_OVF (overflow state) - this will pause the RX DMA until recovery
// - If the watchdog detects no new data for UART_WD_SILENCE_TICKS successive ticks, it will push 
//   a partial chunk to the queue (if there's already some data in the buffer)
// - If data continues to flow in, the timer will wait to accumulate UART_RX_MAX_EVT_DATA_SIZE bytes 
//   before pushing a full chunk to the queue
// - Settings compatibility is ensured by static assertions in the .cpp file (UartDmaDriver_Assert namespace)
//
// Default settings work well with most cases, modify with care!
// - Less than 2% CPU load due to the watchdog in worst-case scenario (in practice closer to <1%)
// - New data detected in less than 2 ms (worst case = partial chunks)
// - The user application can use a fixed-size buffer of size >= UART_RX_MAX_EVT_DATA_SIZE
// - Default timing = perfect for Modbus applications (silence time: 1.8ms, protocol specs: >= 1.75ms)
//
// IMPORTANT! For IRQ cohabitation, check setup procedure in docs
// ====================================================================================

#ifndef UART_RX_MAX_BAUDRATE // Define the max UART baud rate (used in config assertions & enforced in init())
    #define UART_RX_MAX_BAUDRATE 921600
#endif
#ifndef UART_DMA_RING_BITS // Define the DMA ring buffer size in power of two bits
  #define UART_DMA_RING_BITS 11  // <- 2^11 = 2048 (9-15 integer => 512-32768 bytes RX buffer)
#endif
#ifndef UART_RX_MAX_EVT_DATA_SIZE // Define the max size of data chunks advertised in queue
    #define UART_RX_MAX_EVT_DATA_SIZE 256
#endif
#ifndef UART_EVENT_Q_SIZE // Define the depth of event queue (in # events)
    #define UART_EVENT_Q_SIZE 32
#endif
#ifndef UART_WD_TICK_US // Define the tick time for DMA watchdog (ISR invoked every tick)
    #define UART_WD_TICK_US 450
#endif
#ifndef UART_WD_SILENCE_TICKS // Define the number of ticks to consider silence (=> advertise partial chunk)
    #define UART_WD_SILENCE_TICKS 4
#endif
#ifndef UART_TX_BUFFER_SIZE // Define the TX buffer size for DMA transfers
    #define UART_TX_BUFFER_SIZE 256
#endif


class UartDmaDriver {
public:

    // ===================================================================================
    // RESULT TYPE
    // ===================================================================================

    enum Result {
        SUCCESS = 0,
        ERR_NOT_INITIALIZED,
        ERR_BUSY,
        ERR_STATE,
        ERR_UART,
        ERR_CONFIG,
        ERR_TX_BUSY,
        ERR_TX_OVERFLOW,
    };
    static constexpr const char* toString(Result res) {
        switch (res) {
            case SUCCESS: return "Success";
            case ERR_NOT_INITIALIZED: return "Not initialized";
            case ERR_BUSY: return "Driver busy";
            case ERR_STATE: return "Driver state error";
            case ERR_UART: return "UART error";
            case ERR_CONFIG: return "Configuration error";
            case ERR_TX_BUSY: return "TX DMA busy";
            case ERR_TX_OVERFLOW: return "TX data too large";
            default: return "Unknown error";
        }
    }
    static inline Result Error(Result res, const char* msg = nullptr) {
        // Placeholder to manage detailed logging; for now, just return the error
        // printf("[UartDmaDriver] Error: %s (%s)\n", toString(res), msg ? msg : "-");
        return res;
    }
    static inline Result Success() {
        return SUCCESS;
    }

    // ===================================================================================
    // SYNC TYPES
    // ===================================================================================

    using Mutex = UartDmaSync::Mutex;
    using Lock = UartDmaSync::Lock;
    using BinarySemaphore = UartDmaSync::BinarySemaphore;

    // ===================================================================================
    // DATA STRUCTURES
    // ===================================================================================

    enum State {
        STATE_OFF,         // Driver is stopped (RX disabled)
        STATE_RUN_OK,      // Driver is running normally
        STATE_RUN_OVF      // Driver is running but overflow detected
    };

    // Event-driven API for data management
    enum EventType {
        EVT_DATA,      // Data is ready to be read
        EVT_OVERFLOW,  // Flow control activated (stream paused)
    };
    static constexpr const char* toString(EventType type) {
        switch (type) {
            case EVT_DATA: return "Data received";
            case EVT_OVERFLOW: return "Overflow detected";
            default: return "Unknown event";
        }
    }
    struct Event {
        EventType type;
        size_t size;
        bool silenceFlag;
        uint64_t timestampUs;  // Time in microseconds since boot
    };

    // ===================================================================================
    // CONSTANTS
    // ===================================================================================

    // UartDma default settings
    static constexpr uint32_t RX_MAX_BAUDRATE = UART_RX_MAX_BAUDRATE;
    static constexpr int UART_MAX_INSTANCES = 2;
    static constexpr uint32_t MIN_RUNTIME_WD_TICK_US = 100; // Minimum watchdog tick time in microseconds
    static constexpr uint32_t MAX_RUNTIME_WD_TICK_US = 10000; // Maximum watchdog tick time in microseconds
    static constexpr uint8_t MIN_RUNTIME_WD_SILENCE_TICKS = 2; // Minimum silence ticks
    static constexpr uint8_t MAX_RUNTIME_WD_SILENCE_TICKS = 20; // Maximum silence ticks
    
    // Standard baud rates - enforced for reliable timing calculations
    static constexpr uint32_t STANDARD_BAUDRATES[] = {
        9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600
    };
    static constexpr size_t NUM_STANDARD_BAUDRATES = sizeof(STANDARD_BAUDRATES) / sizeof(STANDARD_BAUDRATES[0]);

    // Data chunks queue settings
    static constexpr size_t EVTQ_MAX_EVT_DATA_SIZE = UART_RX_MAX_EVT_DATA_SIZE;
    static constexpr uint EVTQ_SIZE = UART_EVENT_Q_SIZE;

    // DMA transfer settings
    static constexpr uint DMA_RING_BITS = UART_DMA_RING_BITS;
    static constexpr size_t DMA_RX_BUFFER_SIZE = size_t(1) << DMA_RING_BITS;
    static constexpr size_t DMA_OVERFLOW_WATERMARK = // Threshold for overflow detection (min space remaining in buffer)
        (DMA_RX_BUFFER_SIZE / 16u) >= 64u ? (DMA_RX_BUFFER_SIZE / 16u) : 64u; // 64B or 1/16th of buffer size, whichever is larger
    static constexpr int16_t DMA_Q_OVERFLOW_MARKER = -1;
    static constexpr size_t DMA_TX_BUFFER_SIZE = UART_TX_BUFFER_SIZE; // TX buffer size for DMA transfers

    // Watchdog timer settings
    static constexpr int64_t WATCHDOG_TIMER_TICK_US = UART_WD_TICK_US;
    static constexpr uint8_t WATCHDOG_TIMER_SILENCE_TICKS = UART_WD_SILENCE_TICKS;
    static constexpr int64_t WATCHDOG_TIMER_SILENCE_US = 
        WATCHDOG_TIMER_TICK_US * WATCHDOG_TIMER_SILENCE_TICKS; // Duration of silence in microseconds

    // ===================================================================================
    // PUBLIC API
    // ===================================================================================

    UartDmaDriver(uart_inst_t* uart, uint8_t txPin, uint8_t rxPin, uint32_t baudRate, 
                  uint dataBits = 8, uint stopBits = 1, uart_parity_t parity = UART_PARITY_NONE);

    // Disable copy/move constructors and assignment operators
    UartDmaDriver(const UartDmaDriver&) = delete;
    UartDmaDriver& operator=(const UartDmaDriver&) = delete;
    UartDmaDriver(UartDmaDriver&&) = delete;
    UartDmaDriver& operator=(UartDmaDriver&&) = delete;


    ~UartDmaDriver();

    // Lifecycle management
    Result init();
    Result start();
    Result stop();
    
    // State monitoring
    bool isOverflowed() const;
    State getState() const;
    uint32_t getDropped() const { return _queueDropped; }

    // Runtime configuration
    Result setWatchdogTiming(uint32_t tickUs, uint8_t silenceTicks);  // Only when stopped
    Result setBaudrate(uint32_t baudRate);                            // Hot change (when safe)
    static bool isStandardBaudrate(uint32_t baudRate);                // Validation helper
    
    // TX
    Result send(const uint8_t* data, size_t len, TickType_t waitTicks = portMAX_DELAY);
    bool isTxBusy();
    
    // Event-driven RX
    bool popEvent(Event* outEvent, TickType_t waitTicks = 0);
    size_t read(uint8_t* dst, size_t maxLen);
    size_t available() const;
    QueueHandle_t getEventQueueHandle() const { return _eventQueue; }
    
    // IRQ methods - INTERNAL USE ONLY - DO NOT CALL FROM APPLICATION!
    int getTxDmaChannel() const { return _dmaTxChannel; }
    bool onRxWatchdogTimer();  // Watchdog timer callback
    void onRxDmaCplt();        // RX DMA IRQ handler
    void onTxDmaCplt();        // TX DMA IRQ handler

private:

    // ===================================================================================
    // PRIVATE MEMBERS
    // ===================================================================================

    // Configuration & state management
    uart_inst_t* _uart;
    uint8_t _txPin;
    uint8_t _rxPin;
    uint32_t _baudRate;
    uint _dataBits;
    uint _stopBits;
    uart_parity_t _parity;
    bool _initialized;
    volatile State _stateUnsafe; // Must be accessed under critical section only (use getters/setters)
    Mutex _drvMutex;             // Protects against concurrent access to start()/stop() & send()

    // Event queue
    QueueHandle_t _eventQueue;           // Queue of Event (data notifications)
    StaticQueue_t _eventQueueBuffer;     // Static buffer for queue control block
    uint8_t _eventQueueStorage[EVTQ_SIZE * sizeof(Event)]; // Static storage for queue items
    volatile uint32_t _queueDropped;     // Count of dropped events (if queue is full)

    // DMA RX buffer management (ring buffer)
    int _dmaRxChannel;
    uint8_t _dmaRxBuffer[DMA_RX_BUFFER_SIZE] __attribute__((aligned(DMA_RX_BUFFER_SIZE)));
    volatile size_t _dmaReadPos;            // Consumer position (application's bookmark)
    volatile size_t _dmaLastPushedPos;      // Producer position (driver's bookmark)
    volatile size_t _lastDmaWritePos;       // Track RX DMA progress
    void clearRxBuffer();                   // Clear UART RX hardware buffer
    inline size_t getDmaWritePos() const;   // Get current DMA write pointer
    
    // DMA TX buffer management (fixed buffer)
    int _dmaTxChannel;
    uint8_t _dmaTxBuffer[DMA_TX_BUFFER_SIZE] __attribute__((aligned(4)));
    BinarySemaphore _txDmaSemaphore;
    
    // Watchdog timer
    repeating_timer_t _watchdogTimerHandle;  // Hardware timer handle
    volatile uint8_t _silenceTicks;          // Silence counter
    
    // Configurable timing parameters (defaults from compile-time macros)
    uint32_t _watchdogTickUs;                // Watchdog tick period in microseconds
    uint8_t _watchdogSilenceTicks;           // Number of ticks for silence detection

    // DMA & watchdog timer lifecycle
    Result initDMA();      // Initialize state and pointers once
    Result startDMA();     // Start/restart hardware (RX/TX DMA + timer)
    inline void stopDMA(); // Stop hardware (RX/TX DMA + timer)
    inline void resetDMABookmarks(); // Reset RX DMA read/write indices
    
    // Event helpers with automatic timestamping
    inline bool pushEvent(EventType type, size_t size = 0, bool silence = false);
    inline bool pushEventFromISR(EventType type, size_t size, bool silence, BaseType_t* woken);

    // Atomic access helpers to State (concurrency-safe)
    inline State getRunState() const;
    inline void setRunState(State newState);
    inline bool compareAndSetRunState(State expectedState, State newState);
    __attribute__((always_inline, hot)) inline State getRunStateFromISR() const;
    __attribute__((always_inline, hot)) inline void setRunStateFromISR(State newState);
    __attribute__((always_inline, hot)) inline bool compareAndSetRunStateFromISR(State expectedState, State newState);
};

// ===================================================================================
// GLOBAL HANDLERS
// ===================================================================================

// Watchdog & DMA IRQ handlers
bool UartDmaDriver_rxWatchdogTimerCb(struct repeating_timer *t);
void UartDmaDriver_dmaIrqCb();