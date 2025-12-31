#pragma once

#include "sensors/BaseRadarSensor.hpp"

#include <memory>
#include <string>
#include <vector>

namespace radar
{

class RadarFactory
{
public:
    static std::unique_ptr<BaseRadarSensor> createSensor(const std::vector<std::string>& filenames);
};

} // namespace radar
