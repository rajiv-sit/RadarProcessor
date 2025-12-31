#include "radar_core/odometry_estimator.hpp"

#include "utility/math_utils.hpp"

#include <gtest/gtest.h>

namespace
{
utility::EnhancedDetections makeDetections(const std::vector<std::pair<float, float>>& anglesAndRates)
{
    utility::EnhancedDetections detections;
    detections.header.timestamp_us = 1234U;
    detections.detections.reserve(anglesAndRates.size());
    for (const auto& entry : anglesAndRates)
    {
        utility::EnhancedDetection det;
        det.azimuthRaw_rad = entry.first;
        det.rangeRate_ms = entry.second;
        det.flags = static_cast<std::uint8_t>(utility::DetectionFlag::Valid);
        detections.detections.push_back(det);
    }
    return detections;
}
} // namespace

TEST(RadarOdometryEstimatorTest, RejectsInsufficientSamples)
{
    radar::core::RadarOdometryEstimator estimator;
    utility::RadarCalibration calibration;

    auto detections = makeDetections({{0.0f, -1.0f}});
    EXPECT_FALSE(estimator.processDetections(calibration, detections));

    utility::OdometryEstimate estimate;
    EXPECT_FALSE(estimator.latestEstimate(estimate));
}

TEST(RadarOdometryEstimatorTest, EstimatesVelocityFromDetections)
{
    radar::core::OdometrySettings settings;
    settings.maxIterations = 10;
    settings.minInliers = 2;
    settings.inlierThreshold_mps = 0.2f;
    radar::core::RadarOdometryEstimator estimator(settings);

    utility::RadarCalibration calibration;
    calibration.polarity = 1.0f;
    calibration.iso.orientation_rad = 0.0f;

    const float vLon = 5.0f;
    const float vLat = -2.0f;
    const float rr0 = -vLon;
    const float rr90 = -vLat;
    auto detections = makeDetections({{0.0f, rr0}, {utility::kPi * 0.5f, rr90}});

    ASSERT_TRUE(estimator.processDetections(calibration, detections));

    utility::OdometryEstimate estimate;
    ASSERT_TRUE(estimator.latestEstimate(estimate));
    EXPECT_NEAR(estimate.vLon_mps, vLon, 1e-2f);
    EXPECT_NEAR(std::abs(estimate.vLat_mps), std::abs(vLat), 1e-2f);
    EXPECT_TRUE(estimate.valid);
}
