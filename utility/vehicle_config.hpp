#pragma once

#include <filesystem>

#include "utility/radar_types.hpp"

namespace utility
{

class VehicleConfig
{
public:
    bool load(const std::filesystem::path& path);
    const VehicleParameters& parameters() const noexcept;

private:
    VehicleParameters m_parameters;
};

} // namespace utility
