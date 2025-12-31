#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace radar
{

class VehicleProfile;

struct RadarPoint
{
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    float intensity = 0.0F;
    float range_m = 0.0F;
    float rangeRate_ms = 0.0F;
    float rangeRateRaw_ms = 0.0F;
    float azimuthRaw_rad = 0.0F;
    float azimuth_rad = 0.0F;
    float amplitude_dBsm = 0.0F;
    float longitudinalOffset_m = 0.0F;
    float lateralOffset_m = 0.0F;
    int8_t motionStatus = -1;
    uint8_t radarValid = 0;
    uint8_t superResolution = 0;
    uint8_t nearTarget = 0;
    uint8_t hostVehicleClutter = 0;
    uint8_t multibounce = 0;
    int sensorIndex = -1;
    float horizontalFov_rad = 0.0F;
    float maximumRange_m = 0.0F;
    float azimuthPolarity = 0.0F;
    float boresightAngle_rad = 0.0F;
    float sensorLongitudinal_m = 0.0F;
    float sensorLateral_m = 0.0F;
    float elevationRaw_rad = 0.0F;
    int8_t fusedTrackIndex = -1;
    uint8_t isStationary = 0;
    uint8_t isMoveable = 0;
    uint8_t isStatic = 0;
    float stationaryProbability = 0.0F;
};

class BaseRadarSensor
{
public:
    using PointCloud = std::vector<RadarPoint>;

    virtual ~BaseRadarSensor() = default;

    virtual const std::string& identifier() const noexcept = 0;
    virtual void configure(float maxRangeMeters) = 0;
    virtual bool readNextScan(PointCloud& destination, uint64_t& timestampUs) = 0;
    virtual const VehicleProfile* vehicleProfile() const noexcept
    {
        return nullptr;
    }
};

} // namespace radar
