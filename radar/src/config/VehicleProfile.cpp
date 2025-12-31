#include "config/VehicleProfile.hpp"

#include <charconv>
#include <cmath>
#include <fstream>
#include <map>
#include <system_error>

namespace radar
{

bool VehicleProfile::load(const std::filesystem::path& iniPath)
{
    m_contourPoints.clear();
    m_radars.clear();
    m_distRearAxle = 0.0F;

    std::ifstream file(iniPath);
    if (!file)
    {
        return false;
    }

    std::string line;
    std::string currentSection;
    std::string currentRadarName;
    RadarPoseVcs currentRadar;
    std::map<int, glm::vec2> contourPoints;

    auto commitRadar = [&]() {
        if (currentRadar.hasValues && !currentRadarName.empty())
        {
            m_radars[currentRadarName] = convertToIso(currentRadar);
        }
        currentRadar = {};
    };

    while (std::getline(file, line))
    {
        const auto commentPos = line.find(';');
        const std::string trimmed = trim(commentPos == std::string::npos ? line : line.substr(0, commentPos));
        if (trimmed.empty())
        {
            continue;
        }

        if (trimmed.front() == '[' && trimmed.back() == ']')
        {
            commitRadar();
            currentSection = trim(trimmed.substr(1, trimmed.size() - 2));
            currentRadarName = currentSection;
            continue;
        }

        const auto equalPos = trimmed.find('=');
        if (equalPos == std::string::npos)
        {
            continue;
        }

        const std::string key = trim(trimmed.substr(0, equalPos));
        const std::string value = trim(trimmed.substr(equalPos + 1));

        if (key.rfind("contourPt", 0) == 0)
        {
            glm::vec2 contourPoint;
            if (parseContourPoint(value, contourPoint))
            {
                int index = 0;
                const std::string indexText = key.substr(std::string("contourPt").size());
                const char* start = indexText.data();
                const char* end = start + indexText.size();
                const auto result = std::from_chars(start, end, index);
                if (result.ec == std::errc())
                {
                    contourPoints[index] = contourPoint;
                }
            }
            continue;
        }

        if (currentSection == "Geometry" && key == "distRearAxle")
        {
            parseFloat(value, m_distRearAxle);
            continue;
        }

        if (key == "lonPosVCS")
        {
            float parsed = 0.0F;
            if (parseFloat(value, parsed))
            {
                currentRadar.lon = parsed;
                currentRadar.hasValues = true;
            }
        }
        else if (key == "latPosVCS")
        {
            float parsed = 0.0F;
            if (parseFloat(value, parsed))
            {
                currentRadar.lat = parsed;
                currentRadar.hasValues = true;
            }
        }
        else if (key == "orientationVCS")
        {
            float parsed = 0.0F;
            if (parseFloat(value, parsed))
            {
                currentRadar.orientationDeg = parsed;
                currentRadar.hasValues = true;
            }
        }
    }

    commitRadar();
    m_contourPoints.reserve(contourPoints.size());
    for (const auto& entry : contourPoints)
    {
        m_contourPoints.push_back(entry.second);
    }
    return true;
}

const VehicleProfile::RadarMount* VehicleProfile::radarMount(const std::string& name) const noexcept
{
    const auto iter = m_radars.find(name);
    return iter != m_radars.end() ? &iter->second : nullptr;
}

const std::vector<glm::vec2>& VehicleProfile::contourPoints() const noexcept
{
    return m_contourPoints;
}

float VehicleProfile::distRearAxle() const noexcept
{
    return m_distRearAxle;
}

std::string VehicleProfile::trim(const std::string& input)
{
    const auto first = input.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
    {
        return {};
    }

    const auto last = input.find_last_not_of(" \t\r\n");
    return input.substr(first, last - first + 1);
}

bool VehicleProfile::parseFloat(const std::string& text, float& value)
{
    const auto* start = text.data();
    const auto* end = start + text.size();
    const auto result = std::from_chars(start, end, value);
    return result.ec == std::errc();
}

bool VehicleProfile::parseContourPoint(const std::string& text, glm::vec2& point)
{
    const auto commaPos = text.find(',');
    if (commaPos == std::string::npos)
    {
        return false;
    }

    const std::string first = trim(text.substr(0, commaPos));
    const std::string second = trim(text.substr(commaPos + 1));
    float longitudinal = 0.0F;
    float lateral = 0.0F;
    if (!parseFloat(first, longitudinal) || !parseFloat(second, lateral))
    {
        return false;
    }

    point.x = lateral;
    point.y = longitudinal;
    return true;
}

VehicleProfile::RadarMount VehicleProfile::convertToIso(const RadarPoseVcs& pose) const
{
    RadarMount mount;
    mount.isoPosition.x = pose.lon + m_distRearAxle;
    mount.isoPosition.y = -pose.lat;
    mount.isoOrientationRad = -glm::radians(pose.orientationDeg);
    return mount;
}

} // namespace radar
