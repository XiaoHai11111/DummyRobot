#pragma once

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cctype>

inline bool MotionFinite(float value)
{
    // Also works with the firmware's -Ofast build (which assumes isfinite).
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7F800000U) != 0x7F800000U;
}

struct MotionCommand
{
    char kind{};
    unsigned count{};
    float values[9]{};

    bool Parse(const char* text)
    {
        count = 0;
        kind = *text++;
        if (kind != '>' && kind != '@') return false;
        for (;;)
        {
            if (count == 9) return false;
            char* end;
            const float value = strtof(text, &end);
            if (end == text || !MotionFinite(value)) return false;
            values[count++] = value;
            while (isspace(static_cast<unsigned char>(*end))) ++end;
            if (!*end) break;
            if (*end != ',') return false;
            text = end + 1;
        }
        if (count < 6 || (kind == '@' && count > 7)) return false;
        if (count >= 7 && (values[6] <= 0 || values[6] > 100)) return false;
        if (count >= 8 && (values[7] < 0 || values[7] > 45)) return false;
        if (count == 9 && (values[8] < 0 || values[8] > 1)) return false;
        return true;
    }
};
