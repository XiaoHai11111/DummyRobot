#pragma once
#include <cstdint>
#include <cassert>
using osSemaphoreId = void*;
using osThreadId_t = void*;
using osStatus = int;
using osPriority_t = int;
constexpr uint32_t osWaitForever = UINT32_MAX;
constexpr int osPriorityNormal = 24;
constexpr int osPriorityAboveNormal = 32;
struct osThreadAttr_t {
    const char* name;
    uint32_t attr_bits = 0;
    void* cb_mem = nullptr;
    uint32_t cb_size = 0;
    void* stack_mem = nullptr;
    uint32_t stack_size = 0;
    osPriority_t priority = 0;
};
extern osSemaphoreId sem_usb_rx, sem_usb_tx;
int osSemaphoreAcquire(osSemaphoreId, uint32_t);
int osSemaphoreRelease(osSemaphoreId);
osThreadId_t osThreadNew(void (*)(void*), void*, const osThreadAttr_t*);
#define configASSERT(condition) assert(condition)

using osMutexId_t = void*;
constexpr uint32_t osMutexPrioInherit = 2;
struct osMutexAttr_t {
    const char* name;
    uint32_t attr_bits;
    void* cb_mem;
    uint32_t cb_size;
};
osMutexId_t osMutexNew(const osMutexAttr_t*);
int osMutexAcquire(osMutexId_t, uint32_t);
int osMutexRelease(osMutexId_t);
