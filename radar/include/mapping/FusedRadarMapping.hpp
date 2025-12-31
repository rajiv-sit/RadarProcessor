#pragma once

#include "sensors/BaseRadarSensor.hpp"

#include <glm/glm.hpp>

#include <cstdint>
#include <vector>

namespace radar
{

class FusedRadarMapping
{
public:
    enum class RadarModel : uint8_t
    {
        Gaussian = 0,
        Hits
    };

    enum class PlausibilityCombinationMethod : uint8_t
    {
        Average = 0,
        Product,
        Minimum,
        Custom
    };

    struct Settings
    {
        float cellSize = 0.5F;
        float hitIncrement = 0.5F;
        float missDecrement = 0.1F;
        float maxLogOdds = 5.0F;
        float minLogOdds = -5.0F;
        float occupiedThreshold = 0.2F;
        float mapRadius = 60.0F;
        RadarModel radarModel = RadarModel::Gaussian;
        bool enableOccupied = true;
        bool enableFreespace = true;
        bool alwaysMapDynamicDetections = false;
        bool enablePlausibilityScaling = true;
        float maxAdditiveProbability = 0.275F;
        float maxFreeSpaceRange_m = 100.0F;
        float minRange_m = 1e-6F;
        float minPlausibility = 0.01F;
        float freespaceAngleAccuracy_rad = 0.0174532925F;
        float freespaceRangeSigmaFactor = 4.0F;
        float srrRangeAccuracy_m = 0.01F;
        float srrAngleAccuracy_deg = 0.1F;
        float mrrRangeAccuracy_m = 0.25F;
        float mrrAngleAccuracy_deg = 0.5F;
        PlausibilityCombinationMethod plausibilityMethod = PlausibilityCombinationMethod::Custom;
        float customCombinationRangeThreshold = 10.0F;
        float plausibilityRangeMidpoint = 7.0F;
        float plausibilityRangeBandwidth = 10.5F;
        float plausibilityAzimuthMidpoint = 65.0F;
        float plausibilityAzimuthBandwidth = 14.65F;
        float plausibilityAmplitudeMidpoint = -22.0F;
        float plausibilityAmplitudeBandwidth = 8.79F;
    };

    explicit FusedRadarMapping(Settings settings = Settings());

    void update(const BaseRadarSensor::PointCloud& points);
    void reset();
    std::vector<glm::vec3> occupiedCells() const;
    void applySettings(const Settings& settings);
    const Settings& settings() const noexcept;

private:
    bool worldToCell(const glm::vec2& position, int& ix, int& iy) const;
    void updatePlausibilityCache();
    float computePlausibility(float range_m, float azimuth_rad, float amplitude_dBsm) const;
    void computeSensorAccuracies(const RadarPoint& point,
                                 float& rangeAccuracy_m,
                                 float& angleAccuracy_rad) const;
    void addGaussian(const glm::vec2& detectionPosition,
                     const glm::vec2& relativeVector,
                     float range_m,
                     float azimuth_rad,
                     float rangeAccuracy_m,
                     float angleAccuracy_rad,
                     float plausibility);
    void addHit(const glm::vec2& detectionPosition, float plausibility);
    void addFreespaceCone(const glm::vec2& sensorPosition,
                          float azimuth_rad,
                          float range_m,
                          float rangeAccuracy_m,
                          float amplitude_dBsm);
    void updateCell(int ix, int iy, float delta);
    glm::vec3 cellCenter(int ix, int iy) const;
    void initializeGrid();

    Settings m_settings;
    int m_gridSize = 0;
    float m_gridCenter = 0.0F;
    std::vector<float> m_logOdds;
    float m_rangeGrowthRate = 0.0F;
    float m_azimuthGrowthRate = 0.0F;
    float m_amplitudeGrowthRate = 0.0F;
};

} // namespace radar
