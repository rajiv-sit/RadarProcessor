#include "utility/math_utils.hpp"
#include "utility/vehicle_config.hpp"

#include "test_helpers.hpp"

#include <gtest/gtest.h>

namespace fs = std::filesystem;

TEST(VehicleConfigTest, LoadsVehicleConfigAndCalibrations)
{
    const fs::path tempDir = test_helpers::makeTempDir("vehicle_config");
    const fs::path iniPath = tempDir / "Vehicle.ini";
    test_helpers::writeFile(iniPath, test_helpers::buildVehicleConfigIni(1.5f, true, false));

    utility::VehicleConfig config;
    ASSERT_TRUE(config.load(iniPath));

    const auto& params = config.parameters();
    EXPECT_NEAR(params.distRearAxleToFrontBumper_m, 1.5f, 1e-3f);
    ASSERT_GE(params.contourIso.size(), 2U);
    EXPECT_FLOAT_EQ(params.contourIso.front().x, 0.0f);
    EXPECT_FLOAT_EQ(params.contourIso.front().y, 0.0f);
    EXPECT_FLOAT_EQ(params.contourIso[1].x, 2.0f);
    EXPECT_FLOAT_EQ(params.contourIso[1].y, 1.0f);

    const auto& cal = params.radarCalibrations[static_cast<std::size_t>(utility::SensorIndex::FrontLeft)];
    EXPECT_NEAR(cal.vcs.longitudinal_m, 1.0f, 1e-3f);
    EXPECT_NEAR(cal.vcs.lateral_m, 0.5f, 1e-3f);
    EXPECT_NEAR(cal.iso.longitudinal_m, 2.5f, 1e-3f);
    EXPECT_NEAR(cal.iso.lateral_m, -0.5f, 1e-3f);
    EXPECT_NEAR(cal.horizontalFov_rad, utility::degreesToRadians(90.0f), 1e-3f);
}

TEST(VehicleConfigTest, FallsBackToVehicleDistRearAxle)
{
    const fs::path tempDir = test_helpers::makeTempDir("vehicle_config_fallback");
    const fs::path iniPath = tempDir / "Vehicle.ini";
    test_helpers::writeFile(iniPath, test_helpers::buildVehicleConfigIni(2.1f, false, true));

    utility::VehicleConfig config;
    ASSERT_TRUE(config.load(iniPath));
    EXPECT_NEAR(config.parameters().distRearAxleToFrontBumper_m, 2.1f, 1e-3f);
}
