#pragma once

#include <cstdint>

namespace radar::core
{

struct DetectionAssociationSettings
{
    float boundingBoxScale = 1.1f;
    float rangeRateSigma = 3.0f;
    float velocityVariance = 0.05f;
    float headingRateVariance = 0.05f;
};

struct StationaryClassificationSettings
{
    float nSigma = 3.0f;
};

struct OdometrySettings
{
    int maxIterations = 120;
    float inlierThreshold_mps = 0.35f;
    int minInliers = 6;
};

struct ProcessingSettings
{
    DetectionAssociationSettings association;
    StationaryClassificationSettings stationary;
    OdometrySettings odometry;
};

} // namespace radar::core
