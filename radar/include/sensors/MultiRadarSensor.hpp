#pragma once

#include "sensors/BaseRadarSensor.hpp"

#include <memory>
#include <string>
#include <vector>

namespace radar
{

class MultiRadarSensor final : public BaseRadarSensor
{
public:
    explicit MultiRadarSensor(std::vector<std::unique_ptr<BaseRadarSensor>> sensors);

    const std::string& identifier() const noexcept override;
    void configure(float maxRangeMeters) override;
    bool readNextScan(PointCloud& destination, uint64_t& timestampUs) override;
    const VehicleProfile* vehicleProfile() const noexcept override;

private:
    std::vector<std::unique_ptr<BaseRadarSensor>> m_sensors;
    std::string m_identifier;
    const VehicleProfile* m_profile = nullptr;
    PointCloud m_tempBuffer;
};

} // namespace radar
