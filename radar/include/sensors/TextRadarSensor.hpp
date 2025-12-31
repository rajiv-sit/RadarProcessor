#pragma once

#include "config/VehicleProfile.hpp"
#include "sensors/BaseRadarSensor.hpp"

#include <filesystem>
#include <fstream>
#include <glm/glm.hpp>
#include <string>

namespace radar
{

class TextRadarSensor final : public BaseRadarSensor
{
public:
    explicit TextRadarSensor(std::filesystem::path path);
    ~TextRadarSensor() override = default;

    const std::string& identifier() const noexcept override;
    void configure(float maxRangeMeters) override;
    bool readNextScan(PointCloud& destination, uint64_t& timestampUs) override;
    const VehicleProfile* vehicleProfile() const noexcept override;

private:
    bool parseRadarReturnLine(const std::string& line, PointCloud& destination, uint64_t& timestampUs);
    bool parseLegacyLine(const std::string& line, PointCloud& destination, uint64_t& timestampUs);
    void loadVehicleProfile();
    glm::vec2 transformToIso(float x, float y) const;
    std::string m_identifier;
    std::ifstream m_file;
    float m_maxRange = 120.0F;
    std::filesystem::path m_path;
    VehicleProfile m_vehicleProfile;
    const VehicleProfile::RadarMount* m_radarMount = nullptr;
    bool m_profileLoaded = false;
};

} // namespace radar
