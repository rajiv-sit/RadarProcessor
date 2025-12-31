#pragma once

#include "sensors/BaseRadarSensor.hpp"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace radar
{

class OfflineRadarDataReader
{
public:
    explicit OfflineRadarDataReader(std::filesystem::path dataDirectory,
                                     std::vector<std::string> dataFiles);

    bool configure(float maxRangeMeters);
    bool readNextScan(BaseRadarSensor::PointCloud& destination, uint64_t& timestampUs);
    const std::vector<std::string>& lastFrameSources() const noexcept;

private:
    struct SourceFrame
    {
        BaseRadarSensor::PointCloud points;
        uint64_t timestampUs = 0;
        bool ready = false;
        bool finished = false;
    };

    bool prepareFrames();
    std::filesystem::path findRadarFile(const std::string& filename) const;
    std::filesystem::path m_dataDirectory;
    std::vector<std::unique_ptr<BaseRadarSensor>> m_sensors;
    std::vector<SourceFrame> m_frames;
    std::vector<std::string> m_files;
    std::vector<std::string> m_lastFrameSources;
};

} // namespace radar
