#include "sensors/TextRadarSensor.hpp"

#include <charconv>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>

#include "logging/Logger.hpp"

#include <glm/glm.hpp>

namespace radar
{

namespace
{
constexpr size_t kMetadataFields = 9;
constexpr float kMinIntensity = 0.001F;
constexpr float kMaxPosition = 250.0F;
constexpr size_t kRadarHeaderFields = 9;
constexpr size_t kRadarReturnFields = 14;
constexpr size_t kRadarTailFields = 3;
constexpr size_t kRadarFieldsPerReturn = 15;
}

TextRadarSensor::TextRadarSensor(std::filesystem::path path)
    : m_path(std::move(path))
{
    m_identifier = m_path.filename().string();
    Logger::log(Logger::Level::Info, "TextRadarSensor opening file: " + m_path.string());
    m_file.open(m_path, std::ios::in);
    if (!m_file)
    {
        Logger::log(Logger::Level::Error, "Failed to open radar data file: " + m_path.string());
    }
    loadVehicleProfile();
}

const std::string& TextRadarSensor::identifier() const noexcept
{
    return m_identifier;
}

void TextRadarSensor::configure(float maxRangeMeters)
{
    m_maxRange = maxRangeMeters;
}

bool TextRadarSensor::readNextScan(PointCloud& destination, uint64_t& timestampUs)
{
    std::string line;
    while (std::getline(m_file, line))
    {
        if (line.empty())
        {
            continue;
        }
        destination.clear();
        if (parseRadarReturnLine(line, destination, timestampUs))
        {
            return true;
        }
        if (parseLegacyLine(line, destination, timestampUs))
        {
            return true;
        }
    }

    return false;
}

const VehicleProfile* TextRadarSensor::vehicleProfile() const noexcept
{
    return m_profileLoaded ? &m_vehicleProfile : nullptr;
}

void TextRadarSensor::loadVehicleProfile()
{
    if (m_profileLoaded)
    {
        return;
    }

    m_profileLoaded = true;
    const std::filesystem::path profilePath = m_path.parent_path() / "VehicleProfile.ini";
    Logger::log(Logger::Level::Info, "Loading vehicle profile from: " + profilePath.string());
    if (!std::filesystem::exists(profilePath))
    {
        std::cerr << "Vehicle profile not found: " << profilePath << '\n';
        Logger::log(Logger::Level::Error, "Vehicle profile not found: " + profilePath.string());
        return;
    }

    if (!m_vehicleProfile.load(profilePath))
    {
        std::cerr << "Failed to load vehicle profile: " << profilePath << '\n';
        Logger::log(Logger::Level::Error, "Failed to load vehicle profile: " + profilePath.string());
        return;
    }

    m_radarMount = m_vehicleProfile.radarMount("MRR FRONT");
    if (!m_radarMount)
    {
        std::cerr << "Radar profile missing MRR FRONT mount\n";
    }
}

bool TextRadarSensor::parseRadarReturnLine(const std::string& line,
                                           PointCloud& destination,
                                           uint64_t& timestampUs)
{
    std::istringstream tokenizer(line);
    std::vector<double> tokens;
    tokens.reserve(1024);
    double value = 0.0;
    while (tokenizer >> value)
    {
        tokens.push_back(value);
    }

    if (tokens.size() < kRadarHeaderFields + kRadarTailFields)
    {
        return false;
    }

    const size_t remaining = tokens.size() - (kRadarHeaderFields + kRadarTailFields);
    if (remaining % kRadarFieldsPerReturn != 0)
    {
        return false;
    }

    const size_t returnCount = remaining / kRadarFieldsPerReturn;
    if (returnCount == 0)
    {
        return false;
    }

    const int sensorIndex = static_cast<int>(tokens[0]);
    timestampUs = static_cast<uint64_t>(tokens[1]);
    const float horizontalFov_rad = static_cast<float>(tokens[3]);
    const float maximumRange_m = static_cast<float>(tokens[4]);
    const float azimuthPolarity = static_cast<float>(tokens[5]);
    const float boresightAngle_rad = static_cast<float>(tokens[6]);
    const float sensorLongitudinal_m = static_cast<float>(tokens[7]);
    const float sensorLateral_m = static_cast<float>(tokens[8]);

    const size_t returnsOffset = kRadarHeaderFields;
    const size_t elevationOffset = returnsOffset + returnCount * kRadarReturnFields + kRadarTailFields;

    destination.clear();
    destination.reserve(returnCount);
    for (size_t index = 0; index < returnCount; ++index)
    {
        const size_t base = returnsOffset + index * kRadarReturnFields;
        const float range_m = static_cast<float>(tokens[base + 0]);
        const float rangeRate_ms = static_cast<float>(tokens[base + 1]);
        const float rangeRateRaw_ms = static_cast<float>(tokens[base + 2]);
        const float azimuthRaw_rad = static_cast<float>(tokens[base + 3]);
        const float azimuth_rad = static_cast<float>(tokens[base + 4]);
        const float amplitude_dBsm = static_cast<float>(tokens[base + 5]);
        const float longitudinalOffset_m = static_cast<float>(tokens[base + 6]);
        const float lateralOffset_m = static_cast<float>(tokens[base + 7]);
        const int8_t motionStatus = static_cast<int8_t>(tokens[base + 8]);
        const uint8_t radarValid = static_cast<uint8_t>(tokens[base + 9]);
        const uint8_t superResolution = static_cast<uint8_t>(tokens[base + 10]);
        const uint8_t nearTarget = static_cast<uint8_t>(tokens[base + 11]);
        const uint8_t hostVehicleClutter = static_cast<uint8_t>(tokens[base + 12]);
        const uint8_t multibounce = static_cast<uint8_t>(tokens[base + 13]);

        if (range_m <= 0.0F &&
            longitudinalOffset_m == 0.0F &&
            lateralOffset_m == 0.0F &&
            radarValid == 0 &&
            superResolution == 0 &&
            nearTarget == 0 &&
            hostVehicleClutter == 0 &&
            multibounce == 0)
        {
            continue;
        }

        float elevationRaw_rad = 0.0F;
        if (elevationOffset + index < tokens.size())
        {
            elevationRaw_rad = static_cast<float>(tokens[elevationOffset + index]);
        }

        float x = lateralOffset_m;
        float y = longitudinalOffset_m;
        if (x == 0.0F && y == 0.0F && range_m > 0.0F)
        {
            x = range_m * std::sin(azimuth_rad);
            y = range_m * std::cos(azimuth_rad);
        }

        if (!std::isfinite(x) || !std::isfinite(y))
        {
            continue;
        }

        if (std::abs(x) > kMaxPosition || std::abs(y) > kMaxPosition)
        {
            continue;
        }

        if (m_maxRange > 0.0F)
        {
            const float distance = std::sqrt(x * x + y * y);
            if (distance > m_maxRange)
            {
                continue;
            }
        }

        float z = 0.0F;
        if (std::isfinite(elevationRaw_rad))
        {
            z = range_m * std::sin(elevationRaw_rad);
        }

        RadarPoint point{};
        point.x = x;
        point.y = y;
        point.z = z;
        point.intensity = 1.0F;
        point.range_m = range_m;
        point.rangeRate_ms = rangeRate_ms;
        point.rangeRateRaw_ms = rangeRateRaw_ms;
        point.azimuthRaw_rad = azimuthRaw_rad;
        point.azimuth_rad = azimuth_rad;
        point.amplitude_dBsm = amplitude_dBsm;
        point.longitudinalOffset_m = longitudinalOffset_m;
        point.lateralOffset_m = lateralOffset_m;
        point.motionStatus = motionStatus;
        point.radarValid = radarValid;
        point.superResolution = superResolution;
        point.nearTarget = nearTarget;
        point.hostVehicleClutter = hostVehicleClutter;
        point.multibounce = multibounce;
        point.sensorIndex = sensorIndex;
        point.horizontalFov_rad = horizontalFov_rad;
        point.maximumRange_m = maximumRange_m;
        point.azimuthPolarity = azimuthPolarity;
        point.boresightAngle_rad = boresightAngle_rad;
        point.sensorLongitudinal_m = sensorLongitudinal_m;
        point.sensorLateral_m = sensorLateral_m;
        point.elevationRaw_rad = elevationRaw_rad;
        destination.push_back(point);
    }

    return !destination.empty();
}

bool TextRadarSensor::parseLegacyLine(const std::string& line,
                                      PointCloud& destination,
                                      uint64_t& timestampUs)
{
    std::istringstream tokenizer(line);
    std::vector<float> tokens;
    tokens.reserve(256);
    float value = 0.0F;
    while (tokenizer >> value)
    {
        tokens.push_back(value);
    }

    if (tokens.size() <= kMetadataFields)
    {
        return false;
    }

    timestampUs = static_cast<uint64_t>(tokens[1]);
    destination.clear();

    for (size_t index = kMetadataFields; index + 2 < tokens.size(); index += 3)
    {
        const float x = tokens[index];
        const float y = tokens[index + 1];
        const float intensity = std::fabs(tokens[index + 2]);

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(intensity))
        {
            continue;
        }

        if (std::abs(x) > kMaxPosition || std::abs(y) > kMaxPosition)
        {
            continue;
        }

        if (intensity < kMinIntensity)
        {
            continue;
        }

        const glm::vec2 iso = transformToIso(x, y);
        RadarPoint point{};
        point.x = iso.x;
        point.y = iso.y;
        point.z = 0.0F;
        point.intensity = intensity;
        destination.push_back(point);
    }

    if (destination.empty())
    {
        return false;
    }

    return true;
}

glm::vec2 TextRadarSensor::transformToIso(float x, float y) const
{
    if (!m_radarMount)
    {
        return glm::vec2(x, y);
    }

    const float cosTheta = std::cos(m_radarMount->isoOrientationRad);
    const float sinTheta = std::sin(m_radarMount->isoOrientationRad);
    const float rotatedX = x * cosTheta - y * sinTheta + m_radarMount->isoPosition.x;
    const float rotatedY = x * sinTheta + y * cosTheta + m_radarMount->isoPosition.y;
    return glm::vec2(rotatedX, rotatedY);
}

} // namespace radar
