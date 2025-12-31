#pragma once

#include <glm/glm.hpp>

#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

namespace radar
{

class VehicleProfile
{
public:
    struct RadarMount
    {
        glm::vec2 isoPosition{0.0F};
        float isoOrientationRad = 0.0F;
    };

    bool load(const std::filesystem::path& iniPath);
    const RadarMount* radarMount(const std::string& name) const noexcept;
    const std::vector<glm::vec2>& contourPoints() const noexcept;
    float distRearAxle() const noexcept;

private:
    struct RadarPoseVcs
    {
        float lon = 0.0F;
        float lat = 0.0F;
        float orientationDeg = 0.0F;
        bool hasValues = false;
    };

    static std::string trim(const std::string& input);
    static bool parseFloat(const std::string& text, float& value);
    static bool parseContourPoint(const std::string& text, glm::vec2& point);
    RadarMount convertToIso(const RadarPoseVcs& pose) const;

    float m_distRearAxle = 0.0F;
    std::unordered_map<std::string, RadarMount> m_radars;
    std::vector<glm::vec2> m_contourPoints;
};

} // namespace radar
