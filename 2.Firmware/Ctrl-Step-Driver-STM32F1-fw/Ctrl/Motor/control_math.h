#pragma once

#include <cstdint>

// Integer-only stage-2 local compensation. assistCoeffQ16 is
// (viscous - damping) in mA per motor rev/s, scaled to Q16 per encoder step/s.
// Keep this in a helper so the code-generation test can verify that the
// 20 kHz path never emits software float/64-bit division on Cortex-M3.
inline int32_t ApplyLocalCompensationQ16(int32_t current_mA, int32_t velocityStepsPerS,
                                         int32_t assistCoeffQ16, int32_t coulomb_mA,
                                         int32_t deadzoneStepsPerS, int32_t rampInvQ16,
                                         int32_t limit_mA, int32_t currentLimit_mA)
{
    if (limit_mA <= 0) return current_mA;

    const int64_t velocity = velocityStepsPerS;
    const int64_t speed = velocity >= 0 ? velocity : -velocity;
    if (speed <= deadzoneStepsPerS) return current_mA;

    // Ramp the Coulomb term from zero to full over one deadzone width. Without
    // this, a zero/deadzone=0 configuration injects full assist on encoder dither.
    int64_t coulombScaleQ16 = 65536;
    if (rampInvQ16 > 0 && speed < 2 * static_cast<int64_t>(deadzoneStepsPerS))
    {
        coulombScaleQ16 = (speed - deadzoneStepsPerS) * rampInvQ16;
        if (coulombScaleQ16 > 65536) coulombScaleQ16 = 65536;
    }

    int64_t correction_mA = (static_cast<int64_t>(assistCoeffQ16) * velocity) >> 16;
    const int64_t coulomb = (static_cast<int64_t>(coulomb_mA) * coulombScaleQ16) >> 16;
    correction_mA += velocity > 0 ? coulomb : -coulomb;
    if (correction_mA > limit_mA) correction_mA = limit_mA;
    else if (correction_mA < -limit_mA) correction_mA = -limit_mA;

    int64_t total = static_cast<int64_t>(current_mA) + correction_mA;
    if (total > currentLimit_mA) total = currentLimit_mA;
    else if (total < -currentLimit_mA) total = -currentLimit_mA;
    return static_cast<int32_t>(total);
}

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
