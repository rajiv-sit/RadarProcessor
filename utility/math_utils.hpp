#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace utility
{

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kRadToDeg = 180.0f / kPi;

template <typename T>
constexpr T squared(T value)
{
    return value * value;
}

template <typename T>
constexpr T degreesToRadians(T degrees)
{
    return degrees * static_cast<T>(kDegToRad);
}

template <typename T>
constexpr T radiansToDegrees(T radians)
{
    return radians * static_cast<T>(kRadToDeg);
}

template <typename T>
constexpr T clamp(T value, T minValue, T maxValue)
{
    return std::min(maxValue, std::max(minValue, value));
}

template <typename T>
inline std::uint64_t secondsToMicroseconds(T seconds)
{
    return static_cast<std::uint64_t>(seconds * static_cast<T>(1e6));
}

template <typename T>
inline T microsecondsToSeconds(std::uint64_t micros)
{
    return static_cast<T>(micros) * static_cast<T>(1e-6);
}

inline float wrapToPi(float angleRad)
{
    angleRad = std::fmod(angleRad + kPi, 2.0f * kPi);
    if (angleRad < 0.0f)
    {
        angleRad += 2.0f * kPi;
    }
    return angleRad - kPi;
}

} // namespace utility
