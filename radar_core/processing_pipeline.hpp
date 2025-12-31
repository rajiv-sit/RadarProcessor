#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "radar_core/odometry_estimator.hpp"
#include "radar_core/processing_common.hpp"
#include "utility/radar_types.hpp"

namespace radar::core
{

class RadarProcessingPipeline
{
public:
    explicit RadarProcessingPipeline(ProcessingSettings settings = {});

    void initialize(const utility::VehicleParameters* parameters);
    void updateVehicleState(const utility::VehicleMotionState& state);

    bool processCornerDetections(utility::SensorIndex sensor,
                                 std::uint64_t timestamp_us,
                                 const utility::RawCornerDetections& input,
                                 utility::EnhancedDetections& output);

    bool processFrontDetections(std::uint64_t timestamp_us,
                                const utility::RawFrontDetections& input,
                                utility::EnhancedDetections& outputShort,
                                utility::EnhancedDetections& outputLong);

    void processTrackFusion(std::uint64_t timestamp_us,
                            const utility::RawTrackFusion& input,
                            utility::EnhancedTracks& output);

    bool latestOdometry(utility::OdometryEstimate& out) const noexcept;

private:
    struct SensorUpdateState
    {
        bool initialized = false;
        std::uint64_t timestamp_us = 0U;
        std::uint32_t numConsecutiveInvalid = 0U;
    };

    struct TrackState
    {
        glm::vec2 position{0.0f};
        glm::vec2 velocity{0.0f};
        glm::vec2 acceleration{0.0f};
        float length = 0.0f;
        float width = 0.0f;
        float height = 0.0f;
        float heading = 0.0f;
        float headingRate = 0.0f;
        bool isStationary = false;
        bool isMoveable = false;
        float movingVotes = 0.0f;
    };

    bool updateSensorStatus(utility::SensorIndex sensor, std::uint64_t timestamp_us);

    void mapCornerDetections(const utility::RawCornerDetections& input,
                             utility::EnhancedDetections& output) const;
    void mapFrontDetections(const utility::RawFrontDetections& input,
                            utility::EnhancedDetections& outputShort,
                            utility::EnhancedDetections& outputLong) const;

    void classifyDetections(utility::SensorIndex sensor,
                            std::uint64_t timestamp_us,
                            utility::EnhancedDetections& detections);
    void associateDetections(utility::SensorIndex sensor,
                             std::uint64_t timestamp_us,
                             utility::EnhancedDetections& detections);

    glm::vec2 detectionPositionVcs(const utility::EnhancedDetection& det,
                                   const utility::RadarCalibration& calibration) const;

    ProcessingSettings m_settings;
    const utility::VehicleParameters* m_parameters = nullptr;

    std::array<SensorUpdateState, static_cast<std::size_t>(utility::SensorIndex::Count)> m_sensorStates{};
    std::vector<TrackState> m_tracks;
    std::uint64_t m_tracksTimestamp_us = 0U;

    utility::VehicleMotionState m_motionState{};
    bool m_hasExternalMotionState = false;

    RadarOdometryEstimator m_odometry;
    utility::OdometryEstimate m_lastOdometry{};
};

} // namespace radar::core
