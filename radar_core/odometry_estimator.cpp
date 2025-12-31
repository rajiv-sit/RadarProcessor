#include "radar_core/odometry_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include <Eigen/Dense>

namespace radar::core
{
namespace
{
struct Sample
{
    float cosAngle = 0.0f;
    float sinAngle = 0.0f;
    float rangeRate = 0.0f;
};

float predictedRangeRate(const Sample& sample, float vLon, float vLat)
{
    return -(vLon * sample.cosAngle + vLat * sample.sinAngle);
}

bool solvePair(const Sample& a, const Sample& b, float& vLon, float& vLat)
{
    const float a11 = -a.cosAngle;
    const float a12 = -a.sinAngle;
    const float a21 = -b.cosAngle;
    const float a22 = -b.sinAngle;
    const float det = a11 * a22 - a12 * a21;
    if (std::abs(det) < 1e-4f)
    {
        return false;
    }
    vLon = (a.rangeRate * a22 - a12 * b.rangeRate) / det;
    vLat = (a11 * b.rangeRate - a.rangeRate * a21) / det;
    return true;
}
} // namespace

RadarOdometryEstimator::RadarOdometryEstimator(OdometrySettings settings)
    : m_settings(settings)
{
    reset();
}

void RadarOdometryEstimator::reset()
{
    m_lastEstimate = utility::OdometryEstimate{};
}

void RadarOdometryEstimator::updateSettings(const OdometrySettings& settings)
{
    m_settings = settings;
}

bool RadarOdometryEstimator::processDetections(const utility::RadarCalibration& calibration,
                                               const utility::EnhancedDetections& detections)
{
    std::vector<Sample> samples;
    samples.reserve(detections.detections.size());

    const std::uint8_t validMask = static_cast<std::uint8_t>(utility::DetectionFlag::Valid) |
                                   static_cast<std::uint8_t>(utility::DetectionFlag::SuperResolution);

    for (const auto& det : detections.detections)
    {
        if ((det.flags & validMask) == 0U)
        {
            continue;
        }
        if (!std::isfinite(det.rangeRate_ms))
        {
            continue;
        }

        const float angle = (-det.azimuthRaw_rad * calibration.polarity) + calibration.iso.orientation_rad;
        samples.push_back({std::cos(angle), std::sin(angle), det.rangeRate_ms});
    }

    if (samples.size() < 2U)
    {
        return false;
    }

    std::mt19937 rng(42);
    std::uniform_int_distribution<std::size_t> dist(0, samples.size() - 1U);

    float bestVLon = 0.0f;
    float bestVLat = 0.0f;
    std::uint32_t bestInliers = 0U;
    const float threshold = std::max(0.05f, m_settings.inlierThreshold_mps);

    const int iterations = std::max(1, m_settings.maxIterations);
    for (int iter = 0; iter < iterations; ++iter)
    {
        const std::size_t i = dist(rng);
        std::size_t j = dist(rng);
        if (samples.size() > 1U)
        {
            while (j == i)
            {
                j = dist(rng);
            }
        }

        float vLon = 0.0f;
        float vLat = 0.0f;
        if (!solvePair(samples[i], samples[j], vLon, vLat))
        {
            continue;
        }

        std::uint32_t inliers = 0U;
        for (const auto& sample : samples)
        {
            const float residual = std::abs(predictedRangeRate(sample, vLon, vLat) - sample.rangeRate);
            if (residual <= threshold)
            {
                ++inliers;
            }
        }

        if (inliers > bestInliers)
        {
            bestInliers = inliers;
            bestVLon = vLon;
            bestVLat = vLat;
        }
    }

    std::vector<Sample> inlierSamples;
    const bool useInliers = bestInliers >= static_cast<std::uint32_t>(m_settings.minInliers);
    if (useInliers)
    {
        inlierSamples.reserve(bestInliers);
        for (const auto& sample : samples)
        {
            const float residual = std::abs(predictedRangeRate(sample, bestVLon, bestVLat) - sample.rangeRate);
            if (residual <= threshold)
            {
                inlierSamples.push_back(sample);
            }
        }
    }

    const auto& fitSamples = useInliers ? inlierSamples : samples;
    if (fitSamples.size() < 2U)
    {
        return false;
    }

    Eigen::MatrixXf A(static_cast<int>(fitSamples.size()), 2);
    Eigen::VectorXf b(static_cast<int>(fitSamples.size()));
    for (int idx = 0; idx < static_cast<int>(fitSamples.size()); ++idx)
    {
        A(idx, 0) = -fitSamples[idx].cosAngle;
        A(idx, 1) = -fitSamples[idx].sinAngle;
        b(idx) = fitSamples[idx].rangeRate;
    }

    const Eigen::Vector2f solution = A.colPivHouseholderQr().solve(b);

    m_lastEstimate.timestamp_us = detections.header.timestamp_us;
    m_lastEstimate.vLon_mps = solution(0);
    m_lastEstimate.vLat_mps = solution(1);
    m_lastEstimate.yawRate_rps = 0.0f;
    m_lastEstimate.inlierCount = useInliers ? static_cast<std::uint32_t>(fitSamples.size()) : bestInliers;
    m_lastEstimate.valid = useInliers;

    for (float& value : m_lastEstimate.covariance)
    {
        value = 0.0f;
    }
    m_lastEstimate.covariance[0] = useInliers ? 1.0f / static_cast<float>(fitSamples.size()) : 1.0f;
    m_lastEstimate.covariance[4] = m_lastEstimate.covariance[0];
    m_lastEstimate.covariance[8] = 1.0f;

    return m_lastEstimate.valid;
}

bool RadarOdometryEstimator::latestEstimate(utility::OdometryEstimate& out) const noexcept
{
    out = m_lastEstimate;
    return m_lastEstimate.valid;
}

} // namespace radar::core
