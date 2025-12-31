#pragma once

#include <cstdint>

#include "radar_core/processing_common.hpp"
#include "utility/radar_types.hpp"

namespace radar::core
{

class RadarOdometryEstimator
{
public:
    explicit RadarOdometryEstimator(OdometrySettings settings = {});

    void reset();
    void updateSettings(const OdometrySettings& settings);

    bool processDetections(const utility::RadarCalibration& calibration,
                           const utility::EnhancedDetections& detections);

    bool latestEstimate(utility::OdometryEstimate& out) const noexcept;

private:
    OdometrySettings m_settings;
    utility::OdometryEstimate m_lastEstimate;
};

} // namespace radar::core
