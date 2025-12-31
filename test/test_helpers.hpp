#pragma once

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace test_helpers
{
namespace fs = std::filesystem;

inline fs::path makeTempDir(const std::string& prefix)
{
    const auto stamp = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    fs::path dir = fs::temp_directory_path() / (prefix + "_" + std::to_string(stamp));
    fs::create_directories(dir);
    return dir;
}

inline void writeFile(const fs::path& path, const std::string& content)
{
    fs::create_directories(path.parent_path());
    std::ofstream file(path, std::ios::out | std::ios::trunc);
    file << content;
}

inline std::string buildVehicleConfigIni(float distRearAxle,
                                         bool geometryDist,
                                         bool vehicleDist)
{
    std::ostringstream oss;
    if (geometryDist)
    {
        oss << "[Geometry]\n";
        oss << "distRearAxle=" << distRearAxle << "\n\n";
    }
    if (vehicleDist)
    {
        oss << "[Vehicle]\n";
        oss << "distRearAxle=" << distRearAxle << "\n\n";
    }

    oss << "[Radar Common]\n";
    oss << "cornerHardwareTimeDelay=0.01\n";
    oss << "frontCenterHardwareTimeDelay=0.02\n\n";

    oss << "[Contour]\n";
    oss << "contourPt0=0.0,0.0\n";
    oss << "contourPt1=1.0,2.0\n";
    oss << "contourPt2=2.0,2.0\n\n";

    const std::vector<std::string> sections = {
        "SRR FWD LEFT",
        "SRR FWD RIGHT",
        "SRR REAR LEFT",
        "SRR REAR RIGHT",
        "MRR FRONT",
    };

    for (const auto& section : sections)
    {
        oss << '[' << section << "]\n";
        oss << "polarityVCS=1.0\n";
        oss << "rangeRateAccuracy=0.4\n";
        oss << "azimuthAccuracy=1.5\n";
        oss << "orientationVCS=5.0\n";
        oss << "lonPosVCS=1.0\n";
        oss << "latPosVCS=0.5\n";
        oss << "heightAboveGround=0.3\n";
        oss << "horizontalFieldOfView=90.0\n\n";
    }

    return oss.str();
}

inline std::string buildVehicleProfileIni(float distRearAxle)
{
    std::ostringstream oss;
    oss << "[Geometry]\n";
    oss << "distRearAxle=" << distRearAxle << "\n\n";
    oss << "[MRR FRONT]\n";
    oss << "lonPosVCS=1.2\n";
    oss << "latPosVCS=-0.4\n";
    oss << "orientationVCS=10.0\n\n";
    oss << "[Contour]\n";
    oss << "contourPt0=0.0,0.0\n";
    oss << "contourPt1=1.0,2.0\n";
    return oss.str();
}

inline std::string buildCornerDetectionsLine(uint64_t timestampOut,
                                             uint64_t timestampIn,
                                             int radarIndex)
{
    std::ostringstream oss;
    oss << radarIndex << ' '
        << timestampOut << ' '
        << timestampIn << ' '
        << 1.0 << ' '
        << 120.0 << ' '
        << 1.0 << ' '
        << 0.0 << ' '
        << 0.0 << ' '
        << 0.0;

    const std::size_t returnCount = 64;
    for (std::size_t i = 0; i < returnCount; ++i)
    {
        const bool valid = (i == 0U);
        const float range = valid ? 10.0F : 0.0F;
        const float rangeRate = 0.0F;
        const float rangeRateRaw = 0.0F;
        const float azimuthRaw = 0.1F;
        const float azimuth = 0.1F;
        const float amplitude = -5.0F;
        const float lonOffset = valid ? 1.0F : 0.0F;
        const float latOffset = valid ? 1.0F : 0.0F;
        const int motionStatus = 0;
        const int radarValid = valid ? 1 : 0;
        const int superRes = 0;
        const int nearTarget = 0;
        const int hostClutter = 0;
        const int multibounce = 0;

        oss << ' ' << range
            << ' ' << rangeRate
            << ' ' << rangeRateRaw
            << ' ' << azimuthRaw
            << ' ' << azimuth
            << ' ' << amplitude
            << ' ' << lonOffset
            << ' ' << latOffset
            << ' ' << motionStatus
            << ' ' << radarValid
            << ' ' << superRes
            << ' ' << nearTarget
            << ' ' << hostClutter
            << ' ' << multibounce;
    }

    oss << " 0 0 0";

    for (std::size_t i = 0; i < returnCount; ++i)
    {
        const float elevation = (i == 0U) ? 0.05F : 0.0F;
        oss << ' ' << elevation;
    }

    return oss.str();
}

inline std::string buildFrontDetectionsLine(uint64_t timestampOut, uint64_t timestampIn)
{
    std::ostringstream oss;
    oss << 0 << ' '
        << timestampOut << ' '
        << timestampIn << ' '
        << 1.0 << ' '
        << 120.0 << ' '
        << 1.0 << ' '
        << 0.0 << ' '
        << 0.0 << ' '
        << 0.0;

    const std::size_t returnCount = 128;
    for (std::size_t i = 0; i < returnCount; ++i)
    {
        const bool valid = (i == 0U || i == 64U);
        const float range = valid ? 8.0F : 0.0F;
        const float rangeRate = 0.0F;
        const float rangeRateRaw = 0.0F;
        const float azimuthRaw = 0.05F;
        const float azimuth = 0.05F;
        const float amplitude = -10.0F;
        const float lonOffset = valid ? 1.5F : 0.0F;
        const float latOffset = valid ? 0.5F : 0.0F;
        const int motionStatus = 0;
        const int radarValid = valid ? 1 : 0;
        const int superRes = 0;
        const int nearTarget = 0;
        const int hostClutter = 0;
        const int multibounce = 0;

        oss << ' ' << range
            << ' ' << rangeRate
            << ' ' << rangeRateRaw
            << ' ' << azimuthRaw
            << ' ' << azimuth
            << ' ' << amplitude
            << ' ' << lonOffset
            << ' ' << latOffset
            << ' ' << motionStatus
            << ' ' << radarValid
            << ' ' << superRes
            << ' ' << nearTarget
            << ' ' << hostClutter
            << ' ' << multibounce;
    }

    oss << " 0 0 0";
    for (std::size_t i = 0; i < returnCount; ++i)
    {
        const float elevation = (i == 0U || i == 64U) ? 0.03F : 0.0F;
        oss << ' ' << elevation;
    }

    return oss.str();
}

inline std::string buildTrackLine(uint64_t timestamp)
{
    std::ostringstream oss;
    oss << timestamp << ' '
        << timestamp << ' '
        << timestamp << ' '
        << 1 << ' '
        << 1;

    const std::size_t trackCount = 96;
    for (std::size_t i = 0; i < trackCount; ++i)
    {
        const bool valid = (i == 0U);
        const float lon = valid ? 1.0F : 0.0F;
        const float lat = valid ? 1.0F : 0.0F;
        const float length = valid ? 4.0F : 0.0F;
        const float width = valid ? 2.0F : 0.0F;
        const float height = valid ? 1.6F : 0.0F;
        const float prob = valid ? 0.9F : 0.0F;
        const int id = valid ? 7 : 0;
        const int movingFlag = valid ? 1 : 0;
        const int stationaryFlag = 0;
        const int moveableFlag = valid ? 1 : 0;
        const int vehicleFlag = valid ? 1 : 0;
        const int status = valid ? 5 : 0;
        const int objectClass = valid ? 1 : 0;
        const int classConfidence = valid ? 80 : 0;
        const float vLat = 0.0F;
        const float vLon = 0.0F;
        const float aLat = 0.0F;
        const float aLon = 0.0F;
        const float heading = 0.0F;
        const float headingRate = 0.0F;

        oss << ' ' << lon
            << ' ' << lat
            << ' ' << 0.0
            << ' ' << 0.0
            << ' ' << length
            << ' ' << width
            << ' ' << height
            << ' ' << prob
            << ' ' << id;

        for (int skip = 0; skip < 8; ++skip)
        {
            oss << ' ' << 0.0;
        }

        oss << ' ' << movingFlag
            << ' ' << stationaryFlag
            << ' ' << moveableFlag;

        for (int skip = 0; skip < 5; ++skip)
        {
            oss << ' ' << 0.0;
        }

        oss << ' ' << vehicleFlag
            << ' ' << status
            << ' ' << objectClass
            << ' ' << classConfidence
            << ' ' << vLat
            << ' ' << vLon
            << ' ' << aLat
            << ' ' << aLon
            << ' ' << heading
            << ' ' << headingRate;
    }

    return oss.str();
}

} // namespace test_helpers
