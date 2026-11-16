#pragma once

#include <cstdint>

// Conditional integration: keep the load-holding integral, but do not wind it up
// further when P + I + D already demands more than the available current.
// The divisors are powers of two (128/1024). Keep them compile-time constants:
// -Og otherwise emits TWO __aeabi_ldivmod calls on Cortex-M3 every 50 us.
template<int32_t Divisor>
inline void IntegrateLimited(int32_t &integral, int32_t &remainder,
                             int64_t increment,
                             int64_t proportionalDerivative, int32_t currentLimit)
{
    static_assert(Divisor > 0 && (Divisor & (Divisor - 1)) == 0, "power-of-two divisor required");
    const int64_t limit = static_cast<int64_t>(currentLimit) * 1024;
    const int64_t accumulated = remainder + increment;
    int64_t next = integral + accumulated / Divisor;
    if (next > limit) next = limit;
    if (next < -limit) next = -limit;
    const int64_t output = proportionalDerivative + next;
    if ((output > limit && increment > 0) || (output < -limit && increment < 0))
        return;
    integral = static_cast<int32_t>(next);
    remainder = static_cast<int32_t>(accumulated % Divisor);
}
