#pragma once

#include "processing/RadarTrack.hpp"
#include "sensors/BaseRadarSensor.hpp"

#include <glm/glm.hpp>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace utility
{
struct VehicleParameters;
}

namespace radar
{

struct RadarFrame
{
    BaseRadarSensor::PointCloud detections;
    std::vector<RadarTrack> tracks;
    uint64_t timestampUs = 0U;
    std::vector<std::string> sources;
    bool hasDetections = false;
    bool hasTracks = false;
};

class RadarPlayback
{
public:
    struct Settings
    {
        std::filesystem::path dataRoot;
        std::vector<std::string> inputFiles;
        std::filesystem::path vehicleConfigPath;
    };

    explicit RadarPlayback(Settings settings);
    ~RadarPlayback();
    RadarPlayback(const RadarPlayback&) = delete;
    RadarPlayback& operator=(const RadarPlayback&) = delete;
    RadarPlayback(RadarPlayback&&) noexcept;
    RadarPlayback& operator=(RadarPlayback&&) noexcept;

    bool initialize();
    bool readNextFrame(RadarFrame& frame);

    const std::vector<glm::vec2>& vehicleContour() const noexcept;
    const utility::VehicleParameters* vehicleParameters() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace radar
