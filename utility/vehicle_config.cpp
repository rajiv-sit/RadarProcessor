#include "utility/vehicle_config.hpp"

#include <cmath>
#include <iostream>
#include <string>

#include "utility/math_utils.hpp"

#include "IniFileParser.h"

namespace utility
{
namespace
{
constexpr int kMaxContourPoints = 64;

void updateIsoPose(float distRearAxleToFrontBumper_m, RadarCalibration& calibration)
{
    calibration.iso.lateral_m = -calibration.vcs.lateral_m;
    calibration.iso.longitudinal_m = calibration.vcs.longitudinal_m + distRearAxleToFrontBumper_m;
    calibration.iso.height_m = calibration.vcs.height_m;
    calibration.iso.orientation_rad = -calibration.vcs.orientation_rad;
}

void readRadarSection(IniFileParser& parser,
                      const std::string& section,
                      float distRearAxleToFrontBumper_m,
                      RadarCalibration& calibration)
{
    parser.readScalar(section, "polarityVCS", calibration.polarity);
    parser.readScalar(section, "rangeRateAccuracy", calibration.rangeRateAccuracy_mps);

    float azimuthAccuracy_deg = utility::radiansToDegrees(calibration.azimuthAccuracy_rad);
    parser.readScalar(section, "azimuthAccuracy", azimuthAccuracy_deg);
    calibration.azimuthAccuracy_rad = utility::degreesToRadians(azimuthAccuracy_deg);

    float orientation_deg = utility::radiansToDegrees(calibration.vcs.orientation_rad);
    parser.readScalar(section, "orientationVCS", orientation_deg);
    calibration.vcs.orientation_rad = utility::degreesToRadians(orientation_deg);

    parser.readScalar(section, "lonPosVCS", calibration.vcs.longitudinal_m);
    parser.readScalar(section, "latPosVCS", calibration.vcs.lateral_m);
    parser.readScalar(section, "heightAboveGround", calibration.vcs.height_m);

    float horizontalFov_deg = utility::radiansToDegrees(calibration.horizontalFov_rad);
    parser.readScalar(section, "horizontalFieldOfView", horizontalFov_deg);
    calibration.horizontalFov_rad = utility::degreesToRadians(horizontalFov_deg);

    updateIsoPose(distRearAxleToFrontBumper_m, calibration);
}

float readDistRearAxle(IniFileParser& parser, float currentValue)
{
    parser.readScalar("Geometry", "distRearAxle", currentValue);
    if (currentValue <= 0.0f)
    {
        parser.readScalar("Vehicle", "distRearAxle", currentValue);
    }
    return currentValue;
}

void readContour(IniFileParser& parser, std::vector<glm::vec2>& contour)
{
    contour.clear();
    for (int i = 0; i < kMaxContourPoints; ++i)
    {
        const std::string key = "contourPt" + std::to_string(i);
        glm::vec2 value(0.0f);
        if (parser.getVector("Contour", key, value))
        {
            if (std::isfinite(value.x) && std::isfinite(value.y))
            {
                contour.emplace_back(value.y, value.x);
            }
        }
    }
}
} // namespace

bool VehicleConfig::load(const std::filesystem::path& path)
{
    m_parameters = VehicleParameters{};

    IniFileParser parser(path.string());
    if (parser.parseError() != 0)
    {
        std::cerr << "VehicleConfig failed to parse: " << path.string() << "\n";
        return false;
    }

    m_parameters.distRearAxleToFrontBumper_m = readDistRearAxle(parser, m_parameters.distRearAxleToFrontBumper_m);
    parser.readScalar("Radar Common", "cornerHardwareTimeDelay", m_parameters.cornerHardwareDelay_s);
    parser.readScalar("Radar Common", "frontCenterHardwareTimeDelay", m_parameters.frontCenterHardwareDelay_s);

    readContour(parser, m_parameters.contourIso);

    auto& calibrations = m_parameters.radarCalibrations;
    readRadarSection(parser, "SRR FWD LEFT", m_parameters.distRearAxleToFrontBumper_m,
                     calibrations[static_cast<std::size_t>(SensorIndex::FrontLeft)]);
    readRadarSection(parser, "SRR FWD RIGHT", m_parameters.distRearAxleToFrontBumper_m,
                     calibrations[static_cast<std::size_t>(SensorIndex::FrontRight)]);
    readRadarSection(parser, "SRR REAR LEFT", m_parameters.distRearAxleToFrontBumper_m,
                     calibrations[static_cast<std::size_t>(SensorIndex::RearLeft)]);
    readRadarSection(parser, "SRR REAR RIGHT", m_parameters.distRearAxleToFrontBumper_m,
                     calibrations[static_cast<std::size_t>(SensorIndex::RearRight)]);

    RadarCalibration mrrCalibration = calibrations[static_cast<std::size_t>(SensorIndex::FrontShort)];
    readRadarSection(parser, "MRR FRONT", m_parameters.distRearAxleToFrontBumper_m, mrrCalibration);
    calibrations[static_cast<std::size_t>(SensorIndex::FrontShort)] = mrrCalibration;
    calibrations[static_cast<std::size_t>(SensorIndex::FrontLong)] = mrrCalibration;

    return true;
}

const VehicleParameters& VehicleConfig::parameters() const noexcept
{
    return m_parameters;
}

} // namespace utility
