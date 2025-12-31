#pragma once

#include "sensors/BaseRadarSensor.hpp"
#include "sensors/OfflineRadarDataReader.hpp"

#include <filesystem>
#include <string>

namespace radar
{

class OfflineRadarSensor final : public BaseRadarSensor
{
public:
    explicit OfflineRadarSensor(std::filesystem::path dataDirectory = std::filesystem::current_path() / "data");

    const std::string& identifier() const noexcept override;
    void configure(float maxRangeMeters) override;
    bool readNextScan(PointCloud& destination, uint64_t& timestampUs) override;
    const std::vector<std::string>& lastFrameSources() const noexcept;

private:
    OfflineRadarDataReader m_reader;
    std::string m_identifier = "offline-radar";
};

} // namespace radar
