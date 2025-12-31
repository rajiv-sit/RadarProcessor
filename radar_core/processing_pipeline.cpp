#include "radar_core/processing_pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "utility/math_utils.hpp"

namespace radar::core
{
namespace
{
struct OrientedBox
{
    glm::vec2 center{0.0f};
    float halfLength = 0.0f;
    float halfWidth = 0.0f;
    float heading = 0.0f;

    bool contains(const glm::vec2& point) const
    {
        const glm::vec2 delta = point - center;
        const float cosH = std::cos(-heading);
        const float sinH = std::sin(-heading);
        const float localX = delta.x * cosH - delta.y * sinH;
        const float localY = delta.x * sinH + delta.y * cosH;
        return std::abs(localX) <= halfLength && std::abs(localY) <= halfWidth;
    }
};

float stationaryProbabilityFromDistance(float mDist)
{
    return 1.0f - std::erf(mDist / std::sqrt(2.0f));
}

float detectionAngleRad(const utility::EnhancedDetection& det, const utility::RadarCalibration& calibration)
{
    return (-det.azimuthRaw_rad * calibration.polarity) + calibration.iso.orientation_rad;
}

float yawCompensation(const utility::VehicleMotionState& state,
                      const utility::RadarCalibration& calibration,
                      float detAngle)
{
    return state.yawRate_rps *
           ((calibration.iso.longitudinal_m * std::sin(detAngle)) -
            (calibration.iso.lateral_m * std::cos(detAngle)));
}
} // namespace

RadarProcessingPipeline::RadarProcessingPipeline(ProcessingSettings settings)
    : m_settings(settings)
    , m_odometry(settings.odometry)
{
}

void RadarProcessingPipeline::initialize(const utility::VehicleParameters* parameters)
{
    m_parameters = parameters;
}

void RadarProcessingPipeline::updateVehicleState(const utility::VehicleMotionState& state)
{
    m_motionState = state;
    m_hasExternalMotionState = true;
}

bool RadarProcessingPipeline::processCornerDetections(utility::SensorIndex sensor,
                                                      std::uint64_t timestamp_us,
                                                      const utility::RawCornerDetections& input,
                                                      utility::EnhancedDetections& output)
{
    if (!m_parameters)
    {
        return false;
    }

    const bool updateValid = updateSensorStatus(sensor, input.header.timestamp_us);
    mapCornerDetections(input, output);

    const std::uint64_t delayUs = utility::secondsToMicroseconds(m_parameters->cornerHardwareDelay_s);
    const std::uint64_t observationTime = timestamp_us > delayUs ? timestamp_us - delayUs : 0U;

    classifyDetections(sensor, observationTime, output);
    associateDetections(sensor, observationTime, output);

    const auto& calibration = m_parameters->radarCalibrations[static_cast<std::size_t>(sensor)];
    if (!m_hasExternalMotionState)
    {
        if (m_odometry.processDetections(calibration, output))
        {
            m_odometry.latestEstimate(m_lastOdometry);
            m_motionState.vLon_mps = m_lastOdometry.vLon_mps;
            m_motionState.vLat_mps = m_lastOdometry.vLat_mps;
            m_motionState.yawRate_rps = m_lastOdometry.yawRate_rps;
        }
    }

    return updateValid ? m_lastOdometry.valid : false;
}

bool RadarProcessingPipeline::processFrontDetections(std::uint64_t timestamp_us,
                                                     const utility::RawFrontDetections& input,
                                                     utility::EnhancedDetections& outputShort,
                                                     utility::EnhancedDetections& outputLong)
{
    if (!m_parameters)
    {
        return false;
    }

    const bool updateShort = updateSensorStatus(utility::SensorIndex::FrontShort, input.header.timestamp_us);
    const bool updateLong = updateSensorStatus(utility::SensorIndex::FrontLong, input.header.timestamp_us);

    mapFrontDetections(input, outputShort, outputLong);

    const std::uint64_t delayUs = utility::secondsToMicroseconds(m_parameters->frontCenterHardwareDelay_s);
    const std::uint64_t observationTime = timestamp_us > delayUs ? timestamp_us - delayUs : 0U;

    classifyDetections(utility::SensorIndex::FrontShort, observationTime, outputShort);
    associateDetections(utility::SensorIndex::FrontShort, observationTime, outputShort);
    classifyDetections(utility::SensorIndex::FrontLong, observationTime, outputLong);
    associateDetections(utility::SensorIndex::FrontLong, observationTime, outputLong);

    if (!m_hasExternalMotionState)
    {
        const auto& calibration = m_parameters->radarCalibrations[static_cast<std::size_t>(utility::SensorIndex::FrontShort)];
        if (m_odometry.processDetections(calibration, outputShort))
        {
            m_odometry.latestEstimate(m_lastOdometry);
            m_motionState.vLon_mps = m_lastOdometry.vLon_mps;
            m_motionState.vLat_mps = m_lastOdometry.vLat_mps;
            m_motionState.yawRate_rps = m_lastOdometry.yawRate_rps;
        }
    }

    return (updateShort && updateLong) ? m_lastOdometry.valid : false;
}

void RadarProcessingPipeline::processTrackFusion(std::uint64_t timestamp_us,
                                                 const utility::RawTrackFusion& input,
                                                 utility::EnhancedTracks& output)
{
    output.timestamp_us = timestamp_us;
    output.tracks.clear();
    m_tracks.clear();

    for (std::size_t i = 0; i < utility::kTrackCount; ++i)
    {
        const auto status = static_cast<utility::TrackStatus>(input.status[i]);
        if (status == utility::TrackStatus::Invalid)
        {
            continue;
        }

        utility::EnhancedTrack track;
        track.vcsLongitudinalPosition = input.vcsLongitudinalPosition[i];
        track.vcsLateralPosition = input.vcsLateralPosition[i];
        track.vcsLateralVelocity = input.vcsLateralVelocity[i];
        track.vcsLongitudinalVelocity = input.vcsLongitudinalVelocity[i];
        track.vcsLateralAcceleration = input.vcsLateralAcceleration[i];
        track.vcsLongitudinalAcceleration = input.vcsLongitudinalAcceleration[i];
        track.vcsHeading = input.vcsHeading[i];
        track.vcsHeadingRate = input.vcsHeadingRate[i];
        track.length = input.length[i];
        track.width = input.width[i];
        track.height = input.height[i];
        track.probabilityOfDetection = input.probabilityOfDetection[i];
        track.id = input.id[i];
        track.objectClassification = input.objectClassification[i];
        track.objectClassificationConfidence = input.objectClassificationConfidence[i];
        track.isMoving = input.movingFlag[i] != 0U;
        track.isStationary = input.stationaryFlag[i] != 0U;
        track.isMoveable = input.moveableFlag[i] != 0U;
        track.isVehicle = input.vehicleFlag[i] != 0U;
        track.status = status;

        output.tracks.push_back(track);

        TrackState state;
        state.position = glm::vec2(track.vcsLongitudinalPosition, track.vcsLateralPosition);
        state.velocity = glm::vec2(track.vcsLongitudinalVelocity, track.vcsLateralVelocity);
        state.acceleration = glm::vec2(track.vcsLongitudinalAcceleration, track.vcsLateralAcceleration);
        state.length = track.length;
        state.width = track.width;
        state.height = track.height;
        state.heading = track.vcsHeading;
        state.headingRate = track.vcsHeadingRate;
        state.isStationary = track.isStationary;
        state.isMoveable = track.isMoveable;
        state.movingVotes = 0.0f;
        m_tracks.push_back(state);
    }

    m_tracksTimestamp_us = timestamp_us;
}

bool RadarProcessingPipeline::latestOdometry(utility::OdometryEstimate& out) const noexcept
{
    out = m_lastOdometry;
    return m_lastOdometry.valid;
}

bool RadarProcessingPipeline::updateSensorStatus(utility::SensorIndex sensor, std::uint64_t timestamp_us)
{
    auto& state = m_sensorStates[static_cast<std::size_t>(sensor)];
    if (!state.initialized)
    {
        state.initialized = true;
        state.timestamp_us = timestamp_us;
        state.numConsecutiveInvalid = 0U;
        return true;
    }

    if (timestamp_us > state.timestamp_us)
    {
        state.timestamp_us = timestamp_us;
        state.numConsecutiveInvalid = 0U;
        return true;
    }

    state.numConsecutiveInvalid += 1U;
    return false;
}

void RadarProcessingPipeline::mapCornerDetections(const utility::RawCornerDetections& input,
                                                  utility::EnhancedDetections& output) const
{
    output.header = input.header;
    output.detections.resize(utility::kCornerReturnCount);

    for (std::size_t i = 0; i < utility::kCornerReturnCount; ++i)
    {
        auto& det = output.detections[i];
        det.range_m = input.range_m[i];
        det.rangeRate_ms = input.rangeRate_ms[i];
        det.rangeRateRaw_ms = input.rangeRateRaw_ms[i];
        det.azimuthRaw_rad = input.azimuthRaw_rad[i];
        det.azimuth_rad = input.azimuth_rad[i];
        det.amplitude_dBsm = input.amplitude_dBsm[i];
        det.longitudinalOffset_m = input.longitudinalOffset_m[i];
        det.lateralOffset_m = input.lateralOffset_m[i];
        det.motionStatus = input.motionStatus[i];
        det.flags = utility::packDetectionFlags(input.radarValidReturn[i],
                                                input.superResolutionDetection[i],
                                                input.nearTargetDetection[i],
                                                input.hostVehicleClutter[i],
                                                input.multibounceDetection[i]);
    }
}

void RadarProcessingPipeline::mapFrontDetections(const utility::RawFrontDetections& input,
                                                 utility::EnhancedDetections& outputShort,
                                                 utility::EnhancedDetections& outputLong) const
{
    outputShort.header = input.header;
    outputLong.header = input.header;
    outputShort.detections.resize(utility::kCornerReturnCount);
    outputLong.detections.resize(utility::kCornerReturnCount);

    for (std::size_t i = 0; i < utility::kFrontReturnCount; ++i)
    {
        auto& det = (i < utility::kCornerReturnCount)
                        ? outputShort.detections[i]
                        : outputLong.detections[i - utility::kCornerReturnCount];
        det.range_m = input.range_m[i];
        det.rangeRate_ms = input.rangeRate_ms[i];
        det.rangeRateRaw_ms = input.rangeRateRaw_ms[i];
        det.azimuthRaw_rad = input.azimuthRaw_rad[i];
        det.azimuth_rad = input.azimuth_rad[i];
        det.amplitude_dBsm = input.amplitude_dBsm[i];
        det.longitudinalOffset_m = input.longitudinalOffset_m[i];
        det.lateralOffset_m = input.lateralOffset_m[i];
        det.motionStatus = input.motionStatus[i];
        det.flags = utility::packDetectionFlags(input.radarValidReturn[i],
                                                input.superResolutionDetection[i],
                                                input.nearTargetDetection[i],
                                                input.hostVehicleClutter[i],
                                                input.multibounceDetection[i]);
    }
}

void RadarProcessingPipeline::classifyDetections(utility::SensorIndex sensor,
                                                 std::uint64_t /*timestamp_us*/,
                                                 utility::EnhancedDetections& detections)
{
    const auto& calibration = m_parameters->radarCalibrations[static_cast<std::size_t>(sensor)];
    const float sigmaRangeRate = calibration.rangeRateAccuracy_mps / 3.0f;
    const float rangeRateVar = utility::squared(std::max(0.01f, sigmaRangeRate));

    for (auto& det : detections.detections)
    {
        det.fusedTrackIndex = -1;
        det.isMoveable = 0U;

        const float detAngle = detectionAngleRad(det, calibration);
        const float yawTerm = yawCompensation(m_motionState, calibration, detAngle);
        const float compensatedRangeRate = det.rangeRate_ms + yawTerm;

        const float predictedRangeRate =
            -(m_motionState.vLon_mps * std::cos(detAngle) + m_motionState.vLat_mps * std::sin(detAngle));

        const float mDist = std::abs(compensatedRangeRate - predictedRangeRate) /
                            std::sqrt(std::max(rangeRateVar, 1e-4f));

        det.isStationary = static_cast<std::uint8_t>(mDist <= m_settings.stationary.nSigma);
        det.stationaryProbability = std::clamp(stationaryProbabilityFromDistance(mDist), 0.0f, 1.0f);
        det.isStatic = det.isStationary;
    }
}

void RadarProcessingPipeline::associateDetections(utility::SensorIndex sensor,
                                                  std::uint64_t timestamp_us,
                                                  utility::EnhancedDetections& detections)
{
    if (m_tracks.empty())
    {
        return;
    }

    const auto& calibration = m_parameters->radarCalibrations[static_cast<std::size_t>(sensor)];
    const float sigmaRangeRate = calibration.rangeRateAccuracy_mps / 3.0f;
    const float rangeRateVar = utility::squared(std::max(0.01f, sigmaRangeRate));

    const float dt_s = utility::microsecondsToSeconds<float>(timestamp_us > m_tracksTimestamp_us
                                                                ? timestamp_us - m_tracksTimestamp_us
                                                                : 0U);

    std::vector<OrientedBox> boxes;
    boxes.reserve(m_tracks.size());
    std::vector<glm::vec2> centers;
    centers.reserve(m_tracks.size());

    for (const auto& track : m_tracks)
    {
        const glm::vec2 position = track.position + (track.velocity * dt_s) + (track.acceleration * (0.5f * dt_s * dt_s));
        const float heading = track.heading + track.headingRate * dt_s;
        const float halfLength = std::max(track.length, 0.1f) * 0.5f * m_settings.association.boundingBoxScale;
        const float halfWidth = std::max(track.width, 0.1f) * 0.5f * m_settings.association.boundingBoxScale;
        boxes.push_back({position, halfLength, halfWidth, heading});
        centers.push_back(position);
    }

    const std::uint8_t validMask = static_cast<std::uint8_t>(utility::DetectionFlag::Valid) |
                                   static_cast<std::uint8_t>(utility::DetectionFlag::SuperResolution);

    for (auto& det : detections.detections)
    {
        if ((det.flags & validMask) == 0U)
        {
            continue;
        }

        const glm::vec2 detPos = detectionPositionVcs(det, calibration);
        const float detAngle = detectionAngleRad(det, calibration);
        const float rangeRateModelX = -std::cos(detAngle);
        const float rangeRateModelY = -std::sin(detAngle);

        float bestDistance = std::numeric_limits<float>::max();
        std::size_t bestIndex = boxes.size();

        for (std::size_t i = 0; i < boxes.size(); ++i)
        {
            if (!boxes[i].contains(detPos))
            {
                continue;
            }

            const auto& track = m_tracks[i];
            const glm::vec2 relativeVelocity =
                glm::vec2(m_motionState.vLon_mps, m_motionState.vLat_mps) - track.velocity;

            const float predictedRangeRate = relativeVelocity.x * rangeRateModelX +
                                             relativeVelocity.y * rangeRateModelY;

            const float mDist = std::abs(det.rangeRate_ms - predictedRangeRate) /
                                std::sqrt(std::max(rangeRateVar, 1e-4f));

            if (mDist <= m_settings.association.rangeRateSigma && mDist < bestDistance)
            {
                bestDistance = mDist;
                bestIndex = i;
            }
        }

        if (bestIndex < boxes.size())
        {
            auto& track = m_tracks[bestIndex];
            std::uint8_t moveable = track.isMoveable ? 1U : 0U;
            if (!track.isMoveable)
            {
                const float vote = det.isStationary ? -det.stationaryProbability : (1.0f - det.stationaryProbability);
                track.movingVotes = utility::clamp(track.movingVotes + vote, -100.0f, 100.0f);
                moveable = track.movingVotes > 0.0f ? 1U : 0U;
            }

            det.isMoveable = moveable;
            det.isStatic = static_cast<std::uint8_t>((det.isStationary != 0U) && (det.isMoveable == 0U));
            det.fusedTrackIndex = static_cast<std::int8_t>(bestIndex);
        }
    }
}

glm::vec2 RadarProcessingPipeline::detectionPositionVcs(const utility::EnhancedDetection& det,
                                                        const utility::RadarCalibration& calibration) const
{
    float lon = det.longitudinalOffset_m;
    float lat = det.lateralOffset_m;
    if (lon == 0.0f && lat == 0.0f && det.range_m > 0.0f)
    {
        lon = det.range_m * std::cos(det.azimuth_rad);
        lat = det.range_m * std::sin(det.azimuth_rad);
    }

    const float detAngle = (-det.azimuthRaw_rad * calibration.polarity) + calibration.vcs.orientation_rad;
    if (lon == 0.0f && lat == 0.0f && det.range_m > 0.0f)
    {
        lon = det.range_m * std::cos(detAngle);
        lat = det.range_m * std::sin(detAngle);
    }

    return glm::vec2(lon + calibration.vcs.longitudinal_m, lat + calibration.vcs.lateral_m);
}

} // namespace radar::core
