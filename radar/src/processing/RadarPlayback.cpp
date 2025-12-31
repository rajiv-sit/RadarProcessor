#include "processing/RadarPlayback.hpp"

#include "logging/Logger.hpp"

#include "radar_core/processing_pipeline.hpp"
#include "utility/radar_types.hpp"
#include "utility/vehicle_config.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>

namespace fs = std::filesystem;

namespace radar
{
namespace
{
constexpr size_t kCornerReturnCount = utility::kCornerReturnCount;
constexpr size_t kFrontReturnCount = utility::kFrontReturnCount;
constexpr size_t kTrackCount = utility::kTrackCount;
constexpr float kMinTrackExtent = 0.25F;

enum class StreamType
{
    CornerDetections,
    FrontDetections,
    Tracks
};

struct StreamState
{
    StreamType type;
    std::string label;
    fs::path path;
    std::ifstream file;
    bool hasPending = false;
    bool exhausted = false;
    uint64_t timestampUs = 0U;
    uint64_t lastTimestampUs = 0U;
    utility::SensorIndex radarIndex = utility::SensorIndex::FrontLeft;
    utility::RawCornerDetections cornerData{};
    utility::RawFrontDetections frontData{};
    utility::RawTrackFusion trackData{};
    std::vector<float> elevationRad;
};

std::string toLower(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c)
                   {
                       return static_cast<char>(std::tolower(c));
                   });
    return value;
}

std::string radarIndexLabel(utility::SensorIndex index)
{
    switch (index)
    {
        case utility::SensorIndex::FrontLeft:
            return "front_left";
        case utility::SensorIndex::FrontRight:
            return "front_right";
        case utility::SensorIndex::RearLeft:
            return "rear_left";
        case utility::SensorIndex::RearRight:
            return "rear_right";
        case utility::SensorIndex::FrontShort:
            return "front_short";
        case utility::SensorIndex::FrontLong:
            return "front_long";
        default:
            return "unknown";
    }
}

const utility::RadarCalibration& calibrationForSensor(const utility::VehicleParameters& params,
                                                      utility::SensorIndex index)
{
    const std::size_t idx = static_cast<std::size_t>(index);
    if (idx < params.radarCalibrations.size())
    {
        return params.radarCalibrations[idx];
    }
    return params.radarCalibrations.front();
}

bool readNextNonEmptyLine(std::ifstream& file, std::string& line)
{
    while (std::getline(file, line))
    {
        if (!line.empty())
        {
            return true;
        }
    }
    return false;
}

bool parseCornerLine(const std::string& line,
                     utility::SensorIndex& radarIndex,
                     uint64_t& timestampOut,
                     utility::RawCornerDetections& base,
                     std::vector<float>& elevationRad)
{
    std::istringstream ss(line);
    double radarIndexRaw = 0.0;
    double timestampOutRaw = 0.0;
    double timestampInRaw = 0.0;
    double horizontalFov = 0.0;
    double maximumRange = 0.0;
    double azimuthPolarity = 0.0;
    double boresight = 0.0;
    double longitudinalPos = 0.0;
    double lateralPos = 0.0;

    if (!(ss >> radarIndexRaw >> timestampOutRaw >> timestampInRaw >> horizontalFov >> maximumRange >>
          azimuthPolarity >> boresight >> longitudinalPos >> lateralPos))
    {
        return false;
    }

    radarIndex = static_cast<utility::SensorIndex>(static_cast<int>(radarIndexRaw));
    timestampOut = static_cast<uint64_t>(timestampOutRaw);
    base.sensor = radarIndex;
    base.header.timestamp_us = static_cast<uint64_t>(timestampInRaw);
    base.header.horizontalFov_rad = static_cast<float>(horizontalFov);
    base.header.maximumRange_m = static_cast<float>(maximumRange);
    base.header.azimuthPolarity = static_cast<float>(azimuthPolarity);
    base.header.boresightAngle_rad = static_cast<float>(boresight);
    base.header.sensorLongitudinal_m = static_cast<float>(longitudinalPos);
    base.header.sensorLateral_m = static_cast<float>(lateralPos);

    for (size_t i = 0; i < kCornerReturnCount; ++i)
    {
        double value = 0.0;
        if (!(ss >> value))
        {
            return false;
        }
        base.range_m[i] = static_cast<float>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.rangeRate_ms[i] = static_cast<float>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.rangeRateRaw_ms[i] = static_cast<float>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.azimuthRaw_rad[i] = static_cast<float>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.azimuth_rad[i] = static_cast<float>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.amplitude_dBsm[i] = static_cast<float>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.longitudinalOffset_m[i] = static_cast<float>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.lateralOffset_m[i] = static_cast<float>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.motionStatus[i] = static_cast<int8_t>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.radarValidReturn[i] = static_cast<uint8_t>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.superResolutionDetection[i] = static_cast<uint8_t>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.nearTargetDetection[i] = static_cast<uint8_t>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.hostVehicleClutter[i] = static_cast<uint8_t>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.multibounceDetection[i] = static_cast<uint8_t>(value);
    }

    double lookType = 0.0;
    double scanType = 0.0;
    double lookIndex = 0.0;
    ss >> lookType >> scanType >> lookIndex;
    static_cast<void>(lookType);
    static_cast<void>(scanType);
    static_cast<void>(lookIndex);

    elevationRad.assign(kCornerReturnCount, 0.0F);
    for (size_t i = 0; i < kCornerReturnCount; ++i)
    {
        double value = 0.0;
        if (!(ss >> value))
        {
            break;
        }
        elevationRad[i] = static_cast<float>(value);
    }

    return true;
}

bool parseFrontLine(const std::string& line,
                    uint64_t& timestampOut,
                    utility::RawFrontDetections& base,
                    std::vector<float>& elevationRad)
{
    std::istringstream ss(line);
    double radarIndexRaw = 0.0;
    double timestampOutRaw = 0.0;
    double timestampInRaw = 0.0;
    double horizontalFov = 0.0;
    double maximumRange = 0.0;
    double azimuthPolarity = 0.0;
    double boresight = 0.0;
    double longitudinalPos = 0.0;
    double lateralPos = 0.0;

    if (!(ss >> radarIndexRaw >> timestampOutRaw >> timestampInRaw >> horizontalFov >> maximumRange >>
          azimuthPolarity >> boresight >> longitudinalPos >> lateralPos))
    {
        return false;
    }

    timestampOut = static_cast<uint64_t>(timestampOutRaw);
    base.header.timestamp_us = static_cast<uint64_t>(timestampInRaw);
    static_cast<void>(radarIndexRaw);
    base.header.horizontalFov_rad = static_cast<float>(horizontalFov);
    base.header.maximumRange_m = static_cast<float>(maximumRange);
    base.header.azimuthPolarity = static_cast<float>(azimuthPolarity);
    base.header.boresightAngle_rad = static_cast<float>(boresight);
    base.header.sensorLongitudinal_m = static_cast<float>(longitudinalPos);
    base.header.sensorLateral_m = static_cast<float>(lateralPos);

    for (size_t i = 0; i < kFrontReturnCount; ++i)
    {
        double value = 0.0;
        if (!(ss >> value))
        {
            return false;
        }
        base.range_m[i] = static_cast<float>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.rangeRate_ms[i] = static_cast<float>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.rangeRateRaw_ms[i] = static_cast<float>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.azimuthRaw_rad[i] = static_cast<float>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.azimuth_rad[i] = static_cast<float>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.amplitude_dBsm[i] = static_cast<float>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.longitudinalOffset_m[i] = static_cast<float>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.lateralOffset_m[i] = static_cast<float>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.motionStatus[i] = static_cast<int8_t>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.radarValidReturn[i] = static_cast<uint8_t>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.superResolutionDetection[i] = static_cast<uint8_t>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.nearTargetDetection[i] = static_cast<uint8_t>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.hostVehicleClutter[i] = static_cast<uint8_t>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.multibounceDetection[i] = static_cast<uint8_t>(value);
    }

    double lookType = 0.0;
    double scanType = 0.0;
    double lookIndex = 0.0;
    ss >> lookType >> scanType >> lookIndex;
    static_cast<void>(lookType);
    static_cast<void>(scanType);
    static_cast<void>(lookIndex);

    elevationRad.assign(kFrontReturnCount, 0.0F);
    for (size_t i = 0; i < kFrontReturnCount; ++i)
    {
        double value = 0.0;
        if (!(ss >> value))
        {
            break;
        }
        elevationRad[i] = static_cast<float>(value);
    }

    return true;
}

bool parseTrackLine(const std::string& line,
                    uint64_t& timestampOut,
                    utility::RawTrackFusion& base)
{
    std::istringstream ss(line);
    double currentTime = 0.0;
    double visionTimestamp = 0.0;
    double fusionTimestamp = 0.0;
    double fusionIndex = 0.0;
    double imageFrameIndex = 0.0;

    if (!(ss >> currentTime >> visionTimestamp >> fusionTimestamp >> fusionIndex >> imageFrameIndex))
    {
        return false;
    }

    timestampOut = static_cast<uint64_t>(currentTime);
    base.timestamp_us = timestampOut;
    base.visionTimestamp = static_cast<uint64_t>(visionTimestamp);
    base.fusionTimestamp = static_cast<uint64_t>(fusionTimestamp);
    base.fusionIndex = static_cast<uint32_t>(fusionIndex);
    base.imageFrameIndex = static_cast<uint32_t>(imageFrameIndex);

    for (size_t i = 0; i < kTrackCount; ++i)
    {
        double value = 0.0;
        if (!(ss >> value))
        {
            return false;
        }
        base.vcsLongitudinalPosition[i] = static_cast<float>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.vcsLateralPosition[i] = static_cast<float>(value);

        for (int skip = 0; skip < 2; ++skip)
        {
            if (!(ss >> value))
            {
                return false;
            }
        }

        if (!(ss >> value))
        {
            return false;
        }
        base.length[i] = static_cast<float>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.width[i] = static_cast<float>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.height[i] = static_cast<float>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.probabilityOfDetection[i] = static_cast<float>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.id[i] = static_cast<int32_t>(value);

        for (int skip = 0; skip < 8; ++skip)
        {
            if (!(ss >> value))
            {
                return false;
            }
        }

        if (!(ss >> value))
        {
            return false;
        }
        base.movingFlag[i] = static_cast<std::uint8_t>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.stationaryFlag[i] = static_cast<std::uint8_t>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.moveableFlag[i] = static_cast<std::uint8_t>(value);

        for (int skip = 0; skip < 5; ++skip)
        {
            if (!(ss >> value))
            {
                return false;
            }
        }

        if (!(ss >> value))
        {
            return false;
        }
        base.vehicleFlag[i] = static_cast<std::uint8_t>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.status[i] = static_cast<std::uint8_t>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.objectClassification[i] = static_cast<std::uint16_t>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.objectClassificationConfidence[i] = static_cast<std::uint8_t>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.vcsLateralVelocity[i] = static_cast<float>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.vcsLongitudinalVelocity[i] = static_cast<float>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.vcsLateralAcceleration[i] = static_cast<float>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.vcsLongitudinalAcceleration[i] = static_cast<float>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.vcsHeading[i] = static_cast<float>(value);
        if (!(ss >> value))
        {
            return false;
        }
        base.vcsHeadingRate[i] = static_cast<float>(value);
    }

    return true;
}

void appendEnhancedDetections(const utility::EnhancedDetections& data,
                              const utility::RadarCalibration& radarCal,
                              int sensorIndex,
                              const std::vector<float>& elevationRad,
                              radar::BaseRadarSensor::PointCloud& outPoints)
{
    for (size_t i = 0; i < data.detections.size(); ++i)
    {
        const auto& det = data.detections[i];
        if (det.flags == 0U &&
            det.range_m <= 0.0F &&
            det.longitudinalOffset_m == 0.0F &&
            det.lateralOffset_m == 0.0F)
        {
            continue;
        }

        float detAngle = det.azimuth_rad;
        if (detAngle == 0.0F && det.azimuthRaw_rad != 0.0F)
        {
            const float azimuthPolarity = data.header.azimuthPolarity == 0.0F ? 1.0F : data.header.azimuthPolarity;
            detAngle = data.header.boresightAngle_rad + azimuthPolarity * det.azimuthRaw_rad;
        }

        float lateral = det.lateralOffset_m;
        float longitudinal = det.longitudinalOffset_m;
        if (lateral == 0.0F && longitudinal == 0.0F && det.range_m > 0.0F)
        {
            lateral = det.range_m * std::sin(detAngle);
            longitudinal = det.range_m * std::cos(detAngle);
        }

        if (!std::isfinite(lateral) || !std::isfinite(longitudinal))
        {
            continue;
        }

        float elevationRadValue = 0.0F;
        if (i < elevationRad.size())
        {
            elevationRadValue = elevationRad[i];
        }

        float z = det.elevationRaw_m;
        if (z == 0.0F && elevationRadValue != 0.0F)
        {
            z = radarCal.vcs.height_m + det.range_m * std::sin(elevationRadValue);
        }

        RadarPoint point{};
        point.x = lateral;
        point.y = longitudinal;
        point.z = z;
        point.intensity = 1.0F;
        point.range_m = det.range_m;
        point.rangeRate_ms = det.rangeRate_ms;
        point.rangeRateRaw_ms = det.rangeRateRaw_ms;
        point.azimuthRaw_rad = det.azimuthRaw_rad;
        point.azimuth_rad = det.azimuth_rad;
        point.amplitude_dBsm = det.amplitude_dBsm;
        point.longitudinalOffset_m = det.longitudinalOffset_m;
        point.lateralOffset_m = det.lateralOffset_m;
        point.motionStatus = det.motionStatus;
        point.fusedTrackIndex = det.fusedTrackIndex;
        point.isStationary = det.isStationary;
        point.isMoveable = det.isMoveable;
        point.isStatic = det.isStatic;
        point.stationaryProbability = det.stationaryProbability;
        point.sensorIndex = sensorIndex;
        point.horizontalFov_rad = data.header.horizontalFov_rad;
        point.maximumRange_m = data.header.maximumRange_m;
        point.azimuthPolarity = data.header.azimuthPolarity;
        point.boresightAngle_rad = data.header.boresightAngle_rad;
        point.sensorLongitudinal_m = data.header.sensorLongitudinal_m;
        point.sensorLateral_m = data.header.sensorLateral_m;
        point.elevationRaw_rad = elevationRadValue;

        const std::uint8_t typeMask = det.flags;
        point.radarValid = static_cast<std::uint8_t>(
            (typeMask & static_cast<std::uint8_t>(utility::DetectionFlag::Valid)) != 0U);
        point.superResolution = static_cast<std::uint8_t>(
            (typeMask & static_cast<std::uint8_t>(utility::DetectionFlag::SuperResolution)) != 0U);
        point.nearTarget = static_cast<std::uint8_t>(
            (typeMask & static_cast<std::uint8_t>(utility::DetectionFlag::NearTarget)) != 0U);
        point.hostVehicleClutter = static_cast<std::uint8_t>(
            (typeMask & static_cast<std::uint8_t>(utility::DetectionFlag::HostVehicleClutter)) != 0U);
        point.multibounce = static_cast<std::uint8_t>(
            (typeMask & static_cast<std::uint8_t>(utility::DetectionFlag::MultiBounce)) != 0U);

        outPoints.push_back(point);
    }
}

void appendTracks(const utility::EnhancedTracks& data,
                  std::vector<RadarTrack>& tracks)
{
    for (const auto& track : data.tracks)
    {
        if (track.status == utility::TrackStatus::Invalid)
        {
            continue;
        }

        RadarTrack output{};
        output.isoPosition = glm::vec2(
            track.vcsLongitudinalPosition,
            track.vcsLateralPosition);
        output.isoVelocity = glm::vec2(
            track.vcsLongitudinalVelocity,
            track.vcsLateralVelocity);
        output.length = std::max(track.length, kMinTrackExtent);
        output.width = std::max(track.width, kMinTrackExtent);
        output.height = track.height;
        output.headingRad = track.vcsHeading;
        output.headingRate = track.vcsHeadingRate;
        output.probabilityOfDetection = track.probabilityOfDetection;
        output.id = track.id;
        output.objectClassification = track.objectClassification;
        output.objectClassificationConfidence = track.objectClassificationConfidence;
        output.isMoving = track.isMoving;
        output.isStationary = track.isStationary;
        output.isMoveable = track.isMoveable;
        output.isVehicle = track.isVehicle;

        if (output.height == 0.0F)
        {
            const uint16_t classId = track.objectClassification;
            if (classId == static_cast<uint16_t>(utility::TrackObjectClass::Car) ||
                classId == static_cast<uint16_t>(utility::TrackObjectClass::Motorcycle) ||
                classId == static_cast<uint16_t>(utility::TrackObjectClass::Bicycle))
            {
                output.height = 1.8F;
            }
            else if (classId == static_cast<uint16_t>(utility::TrackObjectClass::Truck))
            {
                output.height = 3.8F;
            }
            else
            {
                output.height = 0.05F;
            }
        }

        tracks.push_back(output);
    }
}

} // namespace

struct RadarPlayback::Impl
{
    explicit Impl(Settings settings)
        : settings(std::move(settings))
    {
    }

    Settings settings;
    fs::path dataRoot;
    fs::path vehicleConfigPath;
    utility::VehicleConfig vehicleConfig;
    const utility::VehicleParameters* vehicleParameters = nullptr;
    std::vector<glm::vec2> contour;
    radar::core::RadarProcessingPipeline pipeline;
    std::vector<StreamState> streams;
    bool initialized = false;
};

RadarPlayback::RadarPlayback(Settings settings)
    : m_impl(std::make_unique<Impl>(std::move(settings)))
{
}

RadarPlayback::~RadarPlayback() = default;

RadarPlayback::RadarPlayback(RadarPlayback&&) noexcept = default;

RadarPlayback& RadarPlayback::operator=(RadarPlayback&&) noexcept = default;

bool RadarPlayback::initialize()
{
    if (!m_impl || m_impl->initialized)
    {
        return m_impl && m_impl->initialized;
    }

    m_impl->dataRoot = m_impl->settings.dataRoot;
    if (m_impl->dataRoot.empty())
    {
        m_impl->dataRoot = fs::current_path() / "data";
    }

    m_impl->vehicleConfigPath = m_impl->settings.vehicleConfigPath;
    if (m_impl->vehicleConfigPath.empty())
    {
        m_impl->vehicleConfigPath = m_impl->dataRoot / "Vehicle.ini";
    }
    if (!fs::exists(m_impl->vehicleConfigPath))
    {
        const fs::path fallback = fs::current_path() / "Vehicle.ini";
        if (fs::exists(fallback))
        {
            m_impl->vehicleConfigPath = fallback;
        }
    }

    Logger::initialize(fs::current_path() / "radar_reader.log");
    Logger::log(Logger::Level::Info, "RadarPlayback using data root: " + m_impl->dataRoot.string());
    Logger::log(Logger::Level::Info, "RadarPlayback using vehicle config: " + m_impl->vehicleConfigPath.string());

    if (!m_impl->vehicleConfig.load(m_impl->vehicleConfigPath))
    {
        Logger::log(Logger::Level::Error, "Failed to read vehicle configuration");
        return false;
    }

    m_impl->vehicleParameters = &m_impl->vehicleConfig.parameters();
    m_impl->contour = m_impl->vehicleParameters->contourIso;
    m_impl->pipeline.initialize(m_impl->vehicleParameters);

    for (const auto& file : m_impl->settings.inputFiles)
    {
        const std::string lowerName = toLower(file);
        StreamType type = StreamType::CornerDetections;
        std::string label = "corner";
        if (lowerName.find("track") != std::string::npos)
        {
            type = StreamType::Tracks;
            label = "tracks";
        }
        else if (lowerName.find("front") != std::string::npos)
        {
            type = StreamType::FrontDetections;
            label = "front";
        }

        fs::path path(file);
        if (!path.is_absolute())
        {
            path = m_impl->dataRoot / file;
        }

        StreamState stream{};
        stream.type = type;
        stream.label = label;
        stream.path = path;
        stream.file.open(path, std::ios::in);
        if (!stream.file)
        {
            Logger::log(Logger::Level::Error,
                        "Failed to open radar input file: " + path.string());
            continue;
        }
        m_impl->streams.push_back(std::move(stream));
    }

    m_impl->initialized = !m_impl->streams.empty();
    if (!m_impl->initialized)
    {
        Logger::log(Logger::Level::Error, "RadarPlayback has no valid input files.");
    }
    return m_impl->initialized;
}

bool RadarPlayback::readNextFrame(RadarFrame& frame)
{
    if (!m_impl || !m_impl->initialized)
    {
        return false;
    }

    frame = RadarFrame{};

    for (auto& stream : m_impl->streams)
    {
        if (stream.exhausted || stream.hasPending)
        {
            continue;
        }

        std::string line;
        while (readNextNonEmptyLine(stream.file, line))
        {
            bool parsed = false;
            if (stream.type == StreamType::CornerDetections)
            {
                stream.elevationRad.clear();
                parsed = parseCornerLine(line,
                                         stream.radarIndex,
                                         stream.timestampUs,
                                         stream.cornerData,
                                         stream.elevationRad);
            }
            else if (stream.type == StreamType::FrontDetections)
            {
                stream.elevationRad.clear();
                parsed = parseFrontLine(line, stream.timestampUs, stream.frontData, stream.elevationRad);
            }
            else
            {
                parsed = parseTrackLine(line, stream.timestampUs, stream.trackData);
            }

            if (parsed)
            {
                if (stream.lastTimestampUs > 0U && stream.timestampUs < stream.lastTimestampUs)
                {
                    Logger::log(Logger::Level::Warning,
                                "Non-monotonic timestamp in " + stream.path.string());
                }
                stream.lastTimestampUs = stream.timestampUs;
                stream.hasPending = true;
                break;
            }
        }

        if (!stream.hasPending && stream.file.eof())
        {
            stream.exhausted = true;
        }
    }

    uint64_t earliestTimestamp = std::numeric_limits<uint64_t>::max();
    for (const auto& stream : m_impl->streams)
    {
        if (stream.hasPending)
        {
            earliestTimestamp = std::min(earliestTimestamp, stream.timestampUs);
        }
    }

    if (earliestTimestamp == std::numeric_limits<uint64_t>::max())
    {
        return false;
    }

    frame.timestampUs = earliestTimestamp;

    for (auto& stream : m_impl->streams)
    {
        if (!stream.hasPending || stream.timestampUs != earliestTimestamp)
        {
            continue;
        }

        if (stream.type == StreamType::CornerDetections)
        {
            utility::EnhancedDetections output;
            m_impl->pipeline.processCornerDetections(stream.radarIndex,
                                                     stream.timestampUs,
                                                     stream.cornerData,
                                                     output);
            const auto& radarCal = calibrationForSensor(*m_impl->vehicleParameters, stream.radarIndex);
            const size_t before = frame.detections.size();
            appendEnhancedDetections(output,
                                     radarCal,
                                     static_cast<int>(stream.radarIndex),
                                     stream.elevationRad,
                                     frame.detections);
            if (frame.detections.size() > before)
            {
                frame.sources.push_back("corner:" + radarIndexLabel(stream.radarIndex));
                frame.hasDetections = true;
            }
        }
        else if (stream.type == StreamType::FrontDetections)
        {
            utility::EnhancedDetections outputShort;
            utility::EnhancedDetections outputLong;
            m_impl->pipeline.processFrontDetections(stream.timestampUs,
                                                    stream.frontData,
                                                    outputShort,
                                                    outputLong);
            const auto& radarCalShort = calibrationForSensor(*m_impl->vehicleParameters,
                                                             utility::SensorIndex::FrontShort);
            const auto& radarCalLong = calibrationForSensor(*m_impl->vehicleParameters,
                                                            utility::SensorIndex::FrontLong);
            std::vector<float> shortElev;
            std::vector<float> longElev;
            if (!stream.elevationRad.empty())
            {
                const size_t midpoint = std::min(kCornerReturnCount, stream.elevationRad.size());
                shortElev.assign(stream.elevationRad.begin(),
                                 stream.elevationRad.begin() + midpoint);
                if (stream.elevationRad.size() > midpoint)
                {
                    longElev.assign(stream.elevationRad.begin() + midpoint, stream.elevationRad.end());
                }
            }
            const size_t beforeShort = frame.detections.size();
            appendEnhancedDetections(outputShort,
                                     radarCalShort,
                                     static_cast<int>(utility::SensorIndex::FrontShort),
                                     shortElev,
                                     frame.detections);
            const bool addedShort = frame.detections.size() > beforeShort;
            const size_t beforeLong = frame.detections.size();
            appendEnhancedDetections(outputLong,
                                     radarCalLong,
                                     static_cast<int>(utility::SensorIndex::FrontLong),
                                     longElev,
                                     frame.detections);
            const bool addedLong = frame.detections.size() > beforeLong;
            if (addedShort)
            {
                frame.sources.push_back("front:" + radarIndexLabel(utility::SensorIndex::FrontShort));
            }
            if (addedLong)
            {
                frame.sources.push_back("front:" + radarIndexLabel(utility::SensorIndex::FrontLong));
            }
            if (addedShort || addedLong)
            {
                frame.hasDetections = true;
            }
        }
        else
        {
            utility::EnhancedTracks output;
            m_impl->pipeline.processTrackFusion(stream.timestampUs,
                                                stream.trackData,
                                                output);
            appendTracks(output, frame.tracks);
            frame.sources.push_back("tracks");
            frame.hasTracks = !frame.tracks.empty();
        }

        stream.hasPending = false;
    }

    frame.hasTracks = frame.hasTracks || !frame.tracks.empty();
    frame.hasDetections = frame.hasDetections || !frame.detections.empty();
    return true;
}

const std::vector<glm::vec2>& RadarPlayback::vehicleContour() const noexcept
{
    static const std::vector<glm::vec2> kEmpty;
    return m_impl ? m_impl->contour : kEmpty;
}

const utility::VehicleParameters* RadarPlayback::vehicleParameters() const noexcept
{
    return m_impl ? m_impl->vehicleParameters : nullptr;
}

} // namespace radar
