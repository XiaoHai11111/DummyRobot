#pragma once

#include "FreeRTOS.h"
#include "semphr.h"
#include "cmsis_os.h"

// A packet/DMA semaphore protects the hardware buffer, not a multi-packet
// reply. Keep task writers serialized until all chunks of a record are queued.
// Initialized before the server task starts; no allocation in constructors.
class StreamWriteLock
{
public:
    void Init()
    {
        const osMutexAttr_t attr = {
            .name = "streamTx",
            .attr_bits = osMutexPrioInherit,
            .cb_mem = &storage,
            .cb_size = sizeof(storage),
        };
        mutex = osMutexNew(&attr);
        configASSERT(mutex != nullptr);
    }

    class Guard
    {
    public:
        explicit Guard(StreamWriteLock &lock) : mutex(lock.mutex)
        {
            // Bounded even if the host stops reading. Covers up to three
            // packets for a 159-byte ASCII reply, plus another waiting writer.
            acquired = mutex && osMutexAcquire(mutex, 100) == osOK;
        }
        ~Guard() { if (acquired) osMutexRelease(mutex); }
        explicit operator bool() const { return acquired; }
        Guard(const Guard&) = delete;
        Guard& operator=(const Guard&) = delete;
    private:
        osMutexId_t mutex;
        bool acquired;
    };

private:
    StaticSemaphore_t storage{};
    osMutexId_t mutex = nullptr;
};
