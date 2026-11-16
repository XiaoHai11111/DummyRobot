#include <assert.h>
#include <errno.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>

caddr_t _sbrk(int);
struct _reent;
void __malloc_lock(struct _reent*);
void __malloc_unlock(struct _reent*);
uint32_t mock_primask;

int main(void)
{
    /* Absolute linker symbols model the board's SRAM heap. No bytes at these
       addresses are dereferenced. A CCM task SP must not limit this range. */
    const uintptr_t base = 0x20008000U, limit = 0x2001F800U;
    const uintptr_t ccm_task_sp = 0x1000FFF0U;
    assert(base + 32 > ccm_task_sp); // The previous _sbrk wrongly rejected this.
    assert((uintptr_t)_sbrk(32) == base);
    assert((uintptr_t)_sbrk(0) == base + 32);
    assert(mock_primask == 0);
    assert((uintptr_t)_sbrk(-32) == base + 32);
    assert((uintptr_t)_sbrk(0) == base);

    errno = 0;
    assert(_sbrk(-1) == (caddr_t)-1 && errno == ENOMEM);
    assert(_sbrk(INT_MIN) == (caddr_t)-1);
    assert(_sbrk(INT_MAX) == (caddr_t)-1);
    assert((uintptr_t)_sbrk(0) == base); // Failed requests do not move the break.
    assert((uintptr_t)_sbrk((int)(limit - base)) == base);
    assert((uintptr_t)_sbrk(0) == limit);
    assert(_sbrk(1) == (caddr_t)-1);
    assert((uintptr_t)_sbrk(-(int)(limit - base)) == limit);

    /* Recursive malloc -> sbrk locks must not enable IRQs prematurely. */
    __malloc_lock(0);
    __malloc_lock(0);
    assert(mock_primask == 1);
    assert((uintptr_t)_sbrk(16) == base && mock_primask == 1);
    __malloc_unlock(0);
    assert(mock_primask == 1);
    __malloc_unlock(0);
    assert(mock_primask == 0);
    mock_primask = 1; // Startup/ISR caller that already disabled interrupts.
    __malloc_lock(0);
    assert((uintptr_t)_sbrk(0) == base + 16);
    __malloc_unlock(0);
    assert(mock_primask == 1);
    puts("newlib SRAM heap bounds, CCM independence and allocator lock tests passed");
}
