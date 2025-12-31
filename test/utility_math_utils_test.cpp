#include "utility/math_utils.hpp"

#include <gtest/gtest.h>

TEST(MathUtils, DegreesRadiansConversions)
{
    const float degrees = 180.0f;
    const float radians = utility::degreesToRadians(degrees);
    EXPECT_NEAR(radians, utility::kPi, 1e-6f);
    EXPECT_NEAR(utility::radiansToDegrees(radians), degrees, 1e-6f);
}

TEST(MathUtils, SquaredAndClamp)
{
    EXPECT_EQ(utility::squared(3), 9);
    EXPECT_FLOAT_EQ(utility::squared(2.5f), 6.25f);
    EXPECT_FLOAT_EQ(utility::clamp(5.0f, 0.0f, 4.0f), 4.0f);
    EXPECT_FLOAT_EQ(utility::clamp(-1.0f, 0.0f, 4.0f), 0.0f);
}

TEST(MathUtils, MicrosecondsConversion)
{
    const auto micros = utility::secondsToMicroseconds(0.25);
    EXPECT_EQ(micros, 250000U);
    EXPECT_NEAR(utility::microsecondsToSeconds<double>(micros), 0.25, 1e-9);
}

TEST(MathUtils, WrapsToPiRange)
{
    EXPECT_NEAR(utility::wrapToPi(0.0f), 0.0f, 1e-6f);
    EXPECT_NEAR(utility::wrapToPi(utility::kPi), -utility::kPi, 1e-6f);
    EXPECT_NEAR(utility::wrapToPi(3.0f * utility::kPi), -utility::kPi, 1e-6f);
    EXPECT_NEAR(utility::wrapToPi(-3.0f * utility::kPi), -utility::kPi, 1e-6f);
}
