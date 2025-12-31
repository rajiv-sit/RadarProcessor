#include "config/VehicleProfile.hpp"

#include "test_helpers.hpp"

#include <gtest/gtest.h>

namespace fs = std::filesystem;

TEST(VehicleProfileTest, ParsesProfileAndMounts)
{
    const fs::path tempDir = test_helpers::makeTempDir("vehicle_profile");
    const fs::path iniPath = tempDir / "VehicleProfile.ini";

    std::ostringstream oss;
    oss << "; comment line\n";
    oss << "[Geometry]\n";
    oss << "distRearAxle = 1.5\n\n";
    oss << "[MRR FRONT]\n";
    oss << "lonPosVCS=2.0\n";
    oss << "latPosVCS=-0.5\n";
    oss << "orientationVCS=15.0\n\n";
    oss << "[Contour]\n";
    oss << "contourPt0=0.0,0.0\n";
    oss << "contourPt1=1.0,2.0\n";

    test_helpers::writeFile(iniPath, oss.str());

    radar::VehicleProfile profile;
    ASSERT_TRUE(profile.load(iniPath));
    EXPECT_NEAR(profile.distRearAxle(), 1.5f, 1e-3f);
    ASSERT_EQ(profile.contourPoints().size(), 2U);
    EXPECT_FLOAT_EQ(profile.contourPoints()[1].x, 2.0f);
    EXPECT_FLOAT_EQ(profile.contourPoints()[1].y, 1.0f);

    const auto* mount = profile.radarMount("MRR FRONT");
    ASSERT_NE(mount, nullptr);
    EXPECT_NEAR(mount->isoPosition.x, 3.5f, 1e-3f);
    EXPECT_NEAR(mount->isoPosition.y, 0.5f, 1e-3f);
}
