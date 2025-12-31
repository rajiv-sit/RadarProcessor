#include "radar_core/processing_pipeline.hpp"

#include "utility/math_utils.hpp"

#include <gtest/gtest.h>

namespace
{
utility::VehicleParameters makeVehicleParameters()
{
    utility::VehicleParameters params;
    params.distRearAxleToFrontBumper_m = 1.0f;
    params.cornerHardwareDelay_s = 0.0f;
    params.frontCenterHardwareDelay_s = 0.0f;
    for (auto& calibration : params.radarCalibrations)
    {
        calibration.polarity = 1.0f;
        calibration.rangeRateAccuracy_mps = 1.5f;
        calibration.azimuthAccuracy_rad = utility::degreesToRadians(1.0f);
        calibration.horizontalFov_rad = utility::degreesToRadians(90.0f);
        calibration.vcs.longitudinal_m = 0.0f;
        calibration.vcs.lateral_m = 0.0f;
        calibration.vcs.height_m = 0.0f;
        calibration.vcs.orientation_rad = 0.0f;
        calibration.iso.longitudinal_m = 1.0f;
        calibration.iso.lateral_m = 0.0f;
        calibration.iso.height_m = 0.0f;
        calibration.iso.orientation_rad = 0.0f;
    }
    return params;
}

utility::RawCornerDetections makeCornerDetections()
{
    utility::RawCornerDetections input;
    input.sensor = utility::SensorIndex::FrontLeft;
    input.header.timestamp_us = 1000U;
    input.header.azimuthPolarity = 1.0f;
    input.header.boresightAngle_rad = 0.0f;
    input.range_m[0] = 10.0f;
    input.rangeRate_ms[0] = 0.0f;
    input.azimuthRaw_rad[0] = 0.0f;
    input.azimuth_rad[0] = 0.0f;
    input.longitudinalOffset_m[0] = 1.0f;
    input.lateralOffset_m[0] = 1.0f;
    input.radarValidReturn[0] = 1U;
    return input;
}

utility::RawFrontDetections makeFrontDetections()
{
    utility::RawFrontDetections input;
    input.header.timestamp_us = 2000U;
    input.header.azimuthPolarity = 1.0f;
    input.header.boresightAngle_rad = 0.0f;
    input.range_m[0] = 8.0f;
    input.rangeRate_ms[0] = 0.0f;
    input.azimuthRaw_rad[0] = 0.0f;
    input.azimuth_rad[0] = 0.0f;
    input.longitudinalOffset_m[0] = 2.0f;
    input.lateralOffset_m[0] = 0.5f;
    input.radarValidReturn[0] = 1U;
    input.range_m[64] = 9.0f;
    input.rangeRate_ms[64] = 0.0f;
    input.azimuthRaw_rad[64] = 0.0f;
    input.azimuth_rad[64] = 0.0f;
    input.longitudinalOffset_m[64] = 2.5f;
    input.lateralOffset_m[64] = -0.5f;
    input.radarValidReturn[64] = 1U;
    return input;
}

utility::RawTrackFusion makeTrackFusion()
{
    utility::RawTrackFusion input;
    input.timestamp_us = 900U;
    input.vcsLongitudinalPosition[0] = 1.0f;
    input.vcsLateralPosition[0] = 1.0f;
    input.length[0] = 4.0f;
    input.width[0] = 2.0f;
    input.height[0] = 1.6f;
    input.probabilityOfDetection[0] = 0.8f;
    input.id[0] = 42;
    input.status[0] = static_cast<std::uint8_t>(utility::TrackStatus::Updated);
    input.movingFlag[0] = 1U;
    input.moveableFlag[0] = 1U;
    input.stationaryFlag[0] = 0U;
    input.vehicleFlag[0] = 1U;
    input.objectClassification[0] = static_cast<std::uint16_t>(utility::TrackObjectClass::Car);
    return input;
}
} // namespace

TEST(RadarProcessingPipelineTest, RequiresInitialization)
{
    radar::core::RadarProcessingPipeline pipeline;
    utility::EnhancedDetections output;
    EXPECT_FALSE(pipeline.processCornerDetections(utility::SensorIndex::FrontLeft, 0U,
                                                  utility::RawCornerDetections{}, output));
}

TEST(RadarProcessingPipelineTest, AssociatesTracksWithDetections)
{
    auto params = makeVehicleParameters();
    radar::core::RadarProcessingPipeline pipeline;
    pipeline.initialize(&params);

    utility::VehicleMotionState motion;
    pipeline.updateVehicleState(motion);

    utility::EnhancedTracks tracksOutput;
    pipeline.processTrackFusion(900U, makeTrackFusion(), tracksOutput);
    ASSERT_FALSE(tracksOutput.tracks.empty());

    utility::EnhancedDetections detections;
    EXPECT_FALSE(pipeline.processCornerDetections(utility::SensorIndex::FrontLeft, 1000U,
                                                  makeCornerDetections(), detections));
    ASSERT_FALSE(detections.detections.empty());
    const auto& det = detections.detections[0];
    EXPECT_EQ(det.fusedTrackIndex, 0);
    EXPECT_NE(det.isStationary, 0U);
}

TEST(RadarProcessingPipelineTest, ProcessesFrontDetections)
{
    auto params = makeVehicleParameters();
    radar::core::RadarProcessingPipeline pipeline;
    pipeline.initialize(&params);

    utility::EnhancedDetections outputShort;
    utility::EnhancedDetections outputLong;
    EXPECT_FALSE(pipeline.processFrontDetections(2000U, makeFrontDetections(), outputShort, outputLong));
    ASSERT_EQ(outputShort.detections.size(), utility::kCornerReturnCount);
    ASSERT_EQ(outputLong.detections.size(), utility::kCornerReturnCount);
    EXPECT_NE(outputShort.detections[0].flags, 0U);
    EXPECT_NE(outputLong.detections[0].flags, 0U);
}
