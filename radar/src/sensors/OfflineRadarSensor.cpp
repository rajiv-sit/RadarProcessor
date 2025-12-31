#include "sensors/OfflineRadarSensor.hpp"

namespace radar
{

namespace
{
const std::vector<std::string> kDefaultRadarFiles = {
    "fourCornersfusedRadarDetections.txt",
    "fusedFrontRadarsDetections.txt",
    "fusedRadarTracks.txt",
};
} // namespace

OfflineRadarSensor::OfflineRadarSensor(std::filesystem::path dataDirectory)
    : m_reader(std::move(dataDirectory), kDefaultRadarFiles)
{
}

const std::string& OfflineRadarSensor::identifier() const noexcept
{
    return m_identifier;
}

void OfflineRadarSensor::configure(float maxRangeMeters)
{
    m_reader.configure(maxRangeMeters);
}

bool OfflineRadarSensor::readNextScan(PointCloud& destination, uint64_t& timestampUs)
{
    return m_reader.readNextScan(destination, timestampUs);
}

const std::vector<std::string>& OfflineRadarSensor::lastFrameSources() const noexcept
{
    return m_reader.lastFrameSources();
}

} // namespace radar
