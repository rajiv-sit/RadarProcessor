#include "sensors/MultiRadarSensor.hpp"

#include <algorithm>
#include <sstream>

namespace radar
{

MultiRadarSensor::MultiRadarSensor(std::vector<std::unique_ptr<BaseRadarSensor>> sensors)
    : m_sensors(std::move(sensors))
{
    std::ostringstream identifier;
    for (size_t index = 0; index < m_sensors.size(); ++index)
    {
        if (index != 0)
        {
            identifier << '+';
        }
        identifier << m_sensors[index]->identifier();
        if (!m_profile)
        {
            m_profile = m_sensors[index]->vehicleProfile();
        }
    }
    m_identifier = identifier.str();
    if (m_identifier.empty())
    {
        m_identifier = "multi-radar";
    }
}

const std::string& MultiRadarSensor::identifier() const noexcept
{
    return m_identifier;
}

void MultiRadarSensor::configure(float maxRangeMeters)
{
    for (const auto& sensor : m_sensors)
    {
        sensor->configure(maxRangeMeters);
    }
}

bool MultiRadarSensor::readNextScan(PointCloud& destination, uint64_t& timestampUs)
{
    destination.clear();
    bool any = false;
    uint64_t lastTimestamp = timestampUs;
    for (const auto& sensor : m_sensors)
    {
        m_tempBuffer.clear();
        uint64_t sensorTimestamp = 0;
        if (sensor->readNextScan(m_tempBuffer, sensorTimestamp))
        {
            destination.insert(destination.end(), m_tempBuffer.begin(), m_tempBuffer.end());
            lastTimestamp = sensorTimestamp;
            any = true;
        }
    }

    if (any)
    {
        timestampUs = lastTimestamp;
    }

    return any;
}

const VehicleProfile* MultiRadarSensor::vehicleProfile() const noexcept
{
    return m_profile;
}

} // namespace radar
