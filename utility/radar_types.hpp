#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

namespace utility
{

enum class SensorIndex : int
{
    FrontLeft = 0,
    FrontRight = 1,
    RearLeft = 2,
    RearRight = 3,
    FrontShort = 4,
    FrontLong = 5,
    Count = 6
};

constexpr std::size_t kCornerReturnCount = 64U;
constexpr std::size_t kFrontReturnCount = 128U;
constexpr std::size_t kTrackCount = 96U;

enum class TrackStatus : std::uint8_t
{
    Invalid = 0,
    Merged = 1,
    New = 2,
    NewCoasted = 3,
    NewUpdated = 4,
    Updated = 5,
    Coasted = 6
};

enum class TrackObjectClass : std::uint16_t
{
    Unknown = 0U,
    Car = 1U,
    Motorcycle = 2U,
    Truck = 3U,
    Bicycle = 9U,
    Pedestrian = 10U,
    Animal = 12U
};

enum class DetectionFlag : std::uint8_t
{
    Valid = 1U << 0U,
    SuperResolution = 1U << 1U,
    NearTarget = 1U << 2U,
    HostVehicleClutter = 1U << 3U,
    MultiBounce = 1U << 4U
};

inline std::uint8_t packDetectionFlags(std::uint8_t radarValid,
                                      std::uint8_t superResolution,
                                      std::uint8_t nearTarget,
                                      std::uint8_t hostVehicleClutter,
                                      std::uint8_t multibounce)
{
    return static_cast<std::uint8_t>(radarValid +
                                     (superResolution << 1U) +
                                     (nearTarget << 2U) +
                                     (hostVehicleClutter << 3U) +
                                     (multibounce << 4U));
}

struct RadarPose
{
    float longitudinal_m = 0.0f;
    float lateral_m = 0.0f;
    float height_m = 0.0f;
    float orientation_rad = 0.0f;
};

struct RadarCalibration
{
    RadarPose vcs;
    RadarPose iso;
    float polarity = 1.0f;
    float rangeRateAccuracy_mps = 0.0f;
    float azimuthAccuracy_rad = 0.0f;
    float horizontalFov_rad = 0.0f;
};

struct VehicleParameters
{
    float distRearAxleToFrontBumper_m = 0.0f;
    float cornerHardwareDelay_s = 0.0f;
    float frontCenterHardwareDelay_s = 0.0f;
    std::array<RadarCalibration, static_cast<std::size_t>(SensorIndex::Count)> radarCalibrations{};
    std::vector<glm::vec2> contourIso;
};

struct RawDetectionsHeader
{
    std::uint64_t timestamp_us = 0U;
    float horizontalFov_rad = 0.0f;
    float maximumRange_m = 0.0f;
    float azimuthPolarity = 0.0f;
    float boresightAngle_rad = 0.0f;
    float sensorLongitudinal_m = 0.0f;
    float sensorLateral_m = 0.0f;
};

struct RawCornerDetections
{
    SensorIndex sensor = SensorIndex::FrontLeft;
    RawDetectionsHeader header;
    std::array<float, kCornerReturnCount> range_m{};
    std::array<float, kCornerReturnCount> rangeRate_ms{};
    std::array<float, kCornerReturnCount> rangeRateRaw_ms{};
    std::array<float, kCornerReturnCount> azimuthRaw_rad{};
    std::array<float, kCornerReturnCount> azimuth_rad{};
    std::array<float, kCornerReturnCount> amplitude_dBsm{};
    std::array<float, kCornerReturnCount> longitudinalOffset_m{};
    std::array<float, kCornerReturnCount> lateralOffset_m{};
    std::array<std::int8_t, kCornerReturnCount> motionStatus{};
    std::array<std::uint8_t, kCornerReturnCount> radarValidReturn{};
    std::array<std::uint8_t, kCornerReturnCount> superResolutionDetection{};
    std::array<std::uint8_t, kCornerReturnCount> nearTargetDetection{};
    std::array<std::uint8_t, kCornerReturnCount> hostVehicleClutter{};
    std::array<std::uint8_t, kCornerReturnCount> multibounceDetection{};
};

struct RawFrontDetections
{
    RawDetectionsHeader header;
    std::array<float, kFrontReturnCount> range_m{};
    std::array<float, kFrontReturnCount> rangeRate_ms{};
    std::array<float, kFrontReturnCount> rangeRateRaw_ms{};
    std::array<float, kFrontReturnCount> azimuthRaw_rad{};
    std::array<float, kFrontReturnCount> azimuth_rad{};
    std::array<float, kFrontReturnCount> amplitude_dBsm{};
    std::array<float, kFrontReturnCount> longitudinalOffset_m{};
    std::array<float, kFrontReturnCount> lateralOffset_m{};
    std::array<std::int8_t, kFrontReturnCount> motionStatus{};
    std::array<std::uint8_t, kFrontReturnCount> radarValidReturn{};
    std::array<std::uint8_t, kFrontReturnCount> superResolutionDetection{};
    std::array<std::uint8_t, kFrontReturnCount> nearTargetDetection{};
    std::array<std::uint8_t, kFrontReturnCount> hostVehicleClutter{};
    std::array<std::uint8_t, kFrontReturnCount> multibounceDetection{};
};

struct RawTrackFusion
{
    std::uint64_t timestamp_us = 0U;
    std::uint64_t visionTimestamp = 0U;
    std::uint64_t fusionTimestamp = 0U;
    std::uint32_t fusionIndex = 0U;
    std::uint32_t imageFrameIndex = 0U;
    std::array<float, kTrackCount> vcsLongitudinalPosition{};
    std::array<float, kTrackCount> vcsLateralPosition{};
    std::array<float, kTrackCount> length{};
    std::array<float, kTrackCount> width{};
    std::array<float, kTrackCount> height{};
    std::array<float, kTrackCount> probabilityOfDetection{};
    std::array<std::int32_t, kTrackCount> id{};
    std::array<std::uint16_t, kTrackCount> objectClassification{};
    std::array<std::uint8_t, kTrackCount> objectClassificationConfidence{};
    std::array<std::uint8_t, kTrackCount> status{};
    std::array<float, kTrackCount> vcsLateralVelocity{};
    std::array<float, kTrackCount> vcsLongitudinalVelocity{};
    std::array<float, kTrackCount> vcsLateralAcceleration{};
    std::array<float, kTrackCount> vcsLongitudinalAcceleration{};
    std::array<float, kTrackCount> vcsHeading{};
    std::array<float, kTrackCount> vcsHeadingRate{};
    std::array<std::uint8_t, kTrackCount> movingFlag{};
    std::array<std::uint8_t, kTrackCount> stationaryFlag{};
    std::array<std::uint8_t, kTrackCount> moveableFlag{};
    std::array<std::uint8_t, kTrackCount> vehicleFlag{};
};

struct EnhancedDetection
{
    float range_m = 0.0f;
    float rangeRate_ms = 0.0f;
    float rangeRateRaw_ms = 0.0f;
    float azimuthRaw_rad = 0.0f;
    float azimuth_rad = 0.0f;
    float amplitude_dBsm = 0.0f;
    float longitudinalOffset_m = 0.0f;
    float lateralOffset_m = 0.0f;
    std::int8_t motionStatus = -1;
    std::uint8_t flags = 0U;
    std::int8_t fusedTrackIndex = -1;
    std::uint8_t isStationary = 0U;
    std::uint8_t isMoveable = 0U;
    std::uint8_t isStatic = 0U;
    float stationaryProbability = 0.0f;
    float elevationRaw_m = 0.0f;
};

struct EnhancedDetections
{
    RawDetectionsHeader header;
    std::vector<EnhancedDetection> detections;
};

struct EnhancedTrack
{
    float vcsLongitudinalPosition = 0.0f;
    float vcsLateralPosition = 0.0f;
    float vcsLateralVelocity = 0.0f;
    float vcsLongitudinalVelocity = 0.0f;
    float vcsLateralAcceleration = 0.0f;
    float vcsLongitudinalAcceleration = 0.0f;
    float vcsHeading = 0.0f;
    float vcsHeadingRate = 0.0f;
    float length = 0.0f;
    float width = 0.0f;
    float height = 0.0f;
    float probabilityOfDetection = 0.0f;
    std::int32_t id = -1;
    std::uint16_t objectClassification = 0U;
    std::uint8_t objectClassificationConfidence = 0U;
    bool isMoving = false;
    bool isStationary = false;
    bool isMoveable = false;
    bool isVehicle = false;
    TrackStatus status = TrackStatus::Invalid;
};

struct EnhancedTracks
{
    std::uint64_t timestamp_us = 0U;
    std::vector<EnhancedTrack> tracks;
};

struct VehicleMotionState
{
    float vLon_mps = 0.0f;
    float vLat_mps = 0.0f;
    float yawRate_rps = 0.0f;
    float vLonVariance = 0.1f;
    float vLatVariance = 0.1f;
    float yawRateVariance = 0.1f;
};

struct OdometryEstimate
{
    std::uint64_t timestamp_us = 0U;
    float vLon_mps = 0.0f;
    float vLat_mps = 0.0f;
    float yawRate_rps = 0.0f;
    float covariance[9] = {0.0f};
    std::uint32_t inlierCount = 0U;
    bool valid = false;
};

} // namespace utility
