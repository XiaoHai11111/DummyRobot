#pragma once
#include <cstdint>

struct ControlTimingStats
{
    volatile uint32_t lastCycles = 0;
    volatile uint32_t maxCycles = 0;
    volatile uint32_t overrunCount = 0;
    uint8_t consecutiveOverruns = 0;

    bool Observe(uint32_t elapsedCycles, uint32_t budgetCycles)
    {
        lastCycles = elapsedCycles;
        if (elapsedCycles > maxCycles) maxCycles = elapsedCycles;
        if (elapsedCycles > budgetCycles)
        {
            ++overrunCount;
            if (consecutiveOverruns < 3) ++consecutiveOverruns;
        }
        else consecutiveOverruns = 0;
        return consecutiveOverruns >= 3;
    }
};

extern ControlTimingStats controlTimingStats;
