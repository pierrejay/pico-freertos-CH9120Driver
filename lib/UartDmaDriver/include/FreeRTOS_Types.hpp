#pragma once

#include <FreeRTOS.h>
#include <semphr.h>

namespace UartDmaSync {

/* @brief RAII wrapper for a FreeRTOS mutex
* @note Offers a try-lock mechanism
*/
class Mutex {
public:
    Mutex() { _sem = xSemaphoreCreateMutexStatic(&_semBuf); configASSERT(_sem); }
    ~Mutex() {} // Static Semaphore : no call to vSemaphoreDelete needed
    Mutex(const Mutex&) = delete;
    Mutex& operator=(const Mutex&) = delete;
    inline bool tryLock() { return xSemaphoreTake(_sem, 0) == pdTRUE; }
    inline bool lock(TickType_t wait = portMAX_DELAY) { return xSemaphoreTake(_sem, wait) == pdTRUE; }
    inline void unlock() {  BaseType_t ok = xSemaphoreGive(_sem); configASSERT(ok == pdTRUE); }

private:
    StaticSemaphore_t _semBuf;
    SemaphoreHandle_t _sem;
};

/* @brief RAII wrapper for a FreeRTOS lock (Mutex lock/unlock)
* @param m The Mutex to lock/unlock
* @param wait The time to wait for the lock (in ticks)
*/
class Lock {
public:
    explicit Lock(Mutex& m, TickType_t wait = portMAX_DELAY) : _m(m), _locked(_m.lock(wait)) {}
    ~Lock() { if (_locked) _m.unlock(); }
    bool isLocked() const { return _locked; }

private:
    Mutex& _m;
    volatile bool _locked;
};

/* @brief RAII wrapper for a FreeRTOS binary semaphore
* @note Perfect for DMA transfer synchronization between tasks and ISRs
*/
class BinarySemaphore {
public:
    BinarySemaphore() { 
        _sem = xSemaphoreCreateBinaryStatic(&_semBuf); 
        configASSERT(_sem);
        // Give the semaphore initially so first take() succeeds
        xSemaphoreGive(_sem);
    }
    ~BinarySemaphore() {} // Static Semaphore : no call to vSemaphoreDelete needed
    BinarySemaphore(const BinarySemaphore&) = delete;
    BinarySemaphore& operator=(const BinarySemaphore&) = delete;
    
    inline bool take(TickType_t wait = portMAX_DELAY) { return xSemaphoreTake(_sem, wait) == pdTRUE; }
    inline bool tryTake() { return xSemaphoreTake(_sem, 0) == pdTRUE; }
    inline void give() const { BaseType_t ok = xSemaphoreGive(_sem); configASSERT(ok == pdTRUE); }
    inline void giveForce() const { (void)xSemaphoreGive(_sem); }
    __attribute__((always_inline)) inline void giveFromISR(BaseType_t* pxHigherPriorityTaskWoken) { 
        xSemaphoreGiveFromISR(_sem, pxHigherPriorityTaskWoken); 
    }

private:
    StaticSemaphore_t _semBuf;
    SemaphoreHandle_t _sem;
};

} // namespace UartDmaSync