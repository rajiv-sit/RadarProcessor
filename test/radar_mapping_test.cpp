#include "mapping/FusedRadarMapping.hpp"
#include "mapping/RadarVirtualSensorMapping.hpp"

#include <gtest/gtest.h>

TEST(FusedRadarMappingTest, UpdatesAndResetsOccupiedCells)
{
    radar::FusedRadarMapping::Settings settings;
    settings.cellSize = 1.0f;
    settings.mapRadius = 3.0f;
    settings.occupiedThreshold = 0.05f;
    settings.enableFreespace = false;
    settings.maxAdditiveProbability = 0.8f;
    settings.enablePlausibilityScaling = false;
    settings.minPlausibility = 0.0f;
    radar::FusedRadarMapping mapping(settings);

    radar::BaseRadarSensor::PointCloud points;
    radar::RadarPoint point{};
    point.x = 1.0f;
    point.y = 1.0f;
    point.range_m = 1.5f;
    point.azimuthRaw_rad = 0.1f;
    point.azimuth_rad = 0.1f;
    point.amplitude_dBsm = 50.0f;
    point.radarValid = 1U;
    point.isStationary = 1U;
    point.sensorIndex = 0;
    points.push_back(point);

    mapping.update(points);
    const auto occupied = mapping.occupiedCells();
    EXPECT_FALSE(occupied.empty());

    mapping.reset();
    EXPECT_TRUE(mapping.occupiedCells().empty());
}

TEST(FusedRadarMappingTest, AppliesSettingsAndHitModel)
{
    radar::FusedRadarMapping::Settings settings;
    settings.cellSize = 0.5f;
    settings.mapRadius = 2.0f;
    settings.radarModel = radar::FusedRadarMapping::RadarModel::Hits;
    settings.enablePlausibilityScaling = false;
    settings.minPlausibility = 0.0f;
    settings.occupiedThreshold = 0.0f;
    radar::FusedRadarMapping mapping(settings);

    radar::RadarPoint point{};
    point.x = 0.5f;
    point.y = 0.5f;
    point.range_m = 0.8f;
    point.radarValid = 1U;
    point.sensorIndex = 4;
    point.amplitude_dBsm = 50.0f;
    point.isStationary = 1U;
    mapping.update({point});
    EXPECT_FALSE(mapping.occupiedCells().empty());

    settings.mapRadius = 4.0f;
    mapping.applySettings(settings);
    EXPECT_EQ(mapping.settings().mapRadius, 4.0f);
}

TEST(RadarVirtualSensorMappingTest, SegmentCountClamps)
{
    radar::RadarVirtualSensorMapping mapping;
    mapping.setSegmentCount(1);
    EXPECT_EQ(mapping.segmentCount(), 3U);
}

TEST(RadarVirtualSensorMappingTest, UpdatesRingFromDetections)
{
    radar::RadarVirtualSensorMapping mapping;
    mapping.setSegmentCount(8);

    std::vector<glm::vec2> contour = {
        {-1.0f, -1.0f},
        {1.0f, -1.0f},
        {1.0f, 1.0f},
        {-1.0f, 1.0f},
    };
    mapping.setVehicleContour(contour);

    std::vector<glm::vec2> detections = {glm::vec2(5.0f, 0.0f)};
    mapping.update(detections, {});

    const auto ring = mapping.ring(10.0f);
    ASSERT_EQ(ring.size(), 8U);
    const float length = glm::length(ring.front());
    EXPECT_NEAR(length, 5.0f, 0.1f);
}
