#pragma once
#include <cmath>
#include <cstdint>
constexpr float PI = 3.14159265358979323846f;
inline float arm_sin_f32(float x) { return static_cast<float>(std::sin(static_cast<double>(x))); }
inline float arm_cos_f32(float x) { return static_cast<float>(std::cos(static_cast<double>(x))); }
