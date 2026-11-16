#include "stm32f4xx.h"
#include <errno.h>
#include <stdint.h>
#include <sys/types.h>

/* Linker-owned SRAM range. FreeRTOS heap/stacks in CCM are independent. */
extern char __HeapBase;
extern char __HeapLimit;
static uintptr_t heap_break;
static uint32_t lock_depth;
static uint32_t saved_primask;

/* malloc is also used by USB enumeration in interrupt context. Scheduler-only
 * locks would not protect it. Preserve PRIMASK and support recursive locking,
 * including calls before the scheduler starts. Never call RTOS/stdio here. */
static void HeapLock(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (lock_depth++ == 0) saved_primask = primask;
}

static void HeapUnlock(void)
{
    if (--lock_depth == 0) __set_PRIMASK(saved_primask);
}

struct _reent;
void __malloc_lock(struct _reent* reent)
{
    (void) reent;
    HeapLock();
}

void __malloc_unlock(struct _reent* reent)
{
    (void) reent;
    HeapUnlock();
}

caddr_t _sbrk(int increment)
{
    HeapLock();
    const uintptr_t base = (uintptr_t)&__HeapBase;
    const uintptr_t limit = (uintptr_t)&__HeapLimit;
    if (heap_break == 0) heap_break = base;
    const uintptr_t previous = heap_break;
    if (increment >= 0)
    {
        const uintptr_t amount = (uintptr_t)increment;
        if (amount > limit - heap_break) goto exhausted;
        heap_break += amount;
    }
    else
    {
        /* Use int64_t so INT_MIN cannot overflow when negated. */
        const uintptr_t amount = (uintptr_t)(-(int64_t)increment);
        if (amount > heap_break - base) goto exhausted;
        heap_break -= amount;
    }
    HeapUnlock();
    return (caddr_t)previous;

exhausted:
    errno = ENOMEM;
    HeapUnlock();
    return (caddr_t)-1;
}
