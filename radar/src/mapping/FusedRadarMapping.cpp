#include "mapping/FusedRadarMapping.hpp"

#include <algorithm>
#include <cmath>

namespace
{
constexpr float kDegToRad = 0.0174532925F;
constexpr float kRadToDeg = 57.2957795F;
constexpr float kMinProbability = 1e-3F;
constexpr float kMaxProbability = 1.0F - kMinProbability;

float wrapTo180(float degrees)
{
    float wrapped = std::fmod(degrees + 180.0F, 360.0F);
    if (wrapped < 0.0F)
    {
        wrapped += 360.0F;
    }
    return wrapped - 180.0F;
}

float computeGrowthRate(float bandwidth)
{
    if (bandwidth <= 0.0F)
    {
        return 0.0F;
    }
    return 4.39444915F / bandwidth;
}

float computeIndividualPlausibility(float value, float growthRate, float midpoint)
{
    return 1.0F / (1.0F + std::exp(-growthRate * (value - midpoint)));
}

bool isMrrSensorIndex(int sensorIndex)
{
    return sensorIndex == 4 || sensorIndex == 5;
}
} // namespace

namespace radar
{

FusedRadarMapping::FusedRadarMapping(Settings settings)
    : m_settings(std::move(settings))
{
    updatePlausibilityCache();
    initializeGrid();
}

void FusedRadarMapping::update(const BaseRadarSensor::PointCloud& points)
{
    for (const auto& point : points)
    {
        const bool detectionTypeValid = (point.radarValid != 0U) || (point.superResolution != 0U);
        if (!detectionTypeValid)
        {
            continue;
        }

        const glm::vec2 detectionPosition(point.x, point.y);
        const glm::vec2 sensorPosition(point.sensorLateral_m, point.sensorLongitudinal_m);
        const glm::vec2 relativeVector = detectionPosition - sensorPosition;
        const float relativeNorm = glm::length(relativeVector);
        const float range_m = point.range_m > 0.0F ? point.range_m : relativeNorm;
        if (range_m <= m_settings.minRange_m)
        {
            continue;
        }

        float azimuth_rad = 0.0F;
        if (relativeNorm > 1e-3F)
        {
            azimuth_rad = std::atan2(relativeVector.x, relativeVector.y);
        }
        else
        {
            const float polarity = point.azimuthPolarity == 0.0F ? 1.0F : point.azimuthPolarity;
            azimuth_rad = -point.azimuthRaw_rad * polarity + point.boresightAngle_rad;
        }

        float rangeAccuracy_m = 0.0F;
        float angleAccuracy_rad = 0.0F;
        computeSensorAccuracies(point, rangeAccuracy_m, angleAccuracy_rad);

        const float plausibility = computePlausibility(range_m, azimuth_rad, point.amplitude_dBsm);
        const bool isStationary =
            (point.isStationary != 0U) || (point.isStatic != 0U) || (point.motionStatus == 0);

        if (m_settings.enableOccupied && plausibility >= m_settings.minPlausibility &&
            (isStationary || m_settings.alwaysMapDynamicDetections))
        {
            if (m_settings.radarModel == RadarModel::Gaussian)
            {
                addGaussian(detectionPosition,
                            relativeVector,
                            range_m,
                            azimuth_rad,
                            rangeAccuracy_m,
                            angleAccuracy_rad,
                            plausibility);
            }
            else
            {
                addHit(detectionPosition, plausibility);
            }
        }

        if (m_settings.enableFreespace)
        {
            addFreespaceCone(sensorPosition, azimuth_rad, range_m, rangeAccuracy_m, point.amplitude_dBsm);
        }
    }
}

void FusedRadarMapping::reset()
{
    std::fill(m_logOdds.begin(), m_logOdds.end(), 0.0F);
}

void FusedRadarMapping::applySettings(const Settings& settings)
{
    m_settings = settings;
    updatePlausibilityCache();
    initializeGrid();
}

const FusedRadarMapping::Settings& FusedRadarMapping::settings() const noexcept
{
    return m_settings;
}

std::vector<glm::vec3> FusedRadarMapping::occupiedCells() const
{
    std::vector<glm::vec3> cells;
    cells.reserve(m_gridSize * m_gridSize / 16);
    for (int iy = 0; iy < m_gridSize; ++iy)
    {
        for (int ix = 0; ix < m_gridSize; ++ix)
        {
            const float value = m_logOdds[iy * m_gridSize + ix];
            if (value >= m_settings.occupiedThreshold)
            {
                cells.push_back(cellCenter(ix, iy));
            }
        }
    }
    return cells;
}

bool FusedRadarMapping::worldToCell(const glm::vec2& position, int& ix, int& iy) const
{
    const float scaledX = position.x / m_settings.cellSize + m_gridCenter;
    const float scaledY = position.y / m_settings.cellSize + m_gridCenter;
    ix = static_cast<int>(std::floor(scaledX));
    iy = static_cast<int>(std::floor(scaledY));
    return ix >= 0 && ix < m_gridSize && iy >= 0 && iy < m_gridSize;
}

void FusedRadarMapping::updatePlausibilityCache()
{
    m_rangeGrowthRate = -computeGrowthRate(m_settings.plausibilityRangeBandwidth);
    m_azimuthGrowthRate = -computeGrowthRate(m_settings.plausibilityAzimuthBandwidth);
    m_amplitudeGrowthRate = computeGrowthRate(m_settings.plausibilityAmplitudeBandwidth);
}

float FusedRadarMapping::computePlausibility(float range_m, float azimuth_rad, float amplitude_dBsm) const
{
    if (!m_settings.enablePlausibilityScaling)
    {
        return 1.0F;
    }

    const float rangeComponent =
        computeIndividualPlausibility(range_m, m_rangeGrowthRate, m_settings.plausibilityRangeMidpoint);
    const float azimuthDeg = std::abs(wrapTo180(azimuth_rad * kRadToDeg));
    const float azimuthComponent =
        computeIndividualPlausibility(azimuthDeg, m_azimuthGrowthRate, m_settings.plausibilityAzimuthMidpoint);
    const float amplitudeComponent =
        computeIndividualPlausibility(amplitude_dBsm, m_amplitudeGrowthRate, m_settings.plausibilityAmplitudeMidpoint);

    float combined = 1.0F;
    switch (m_settings.plausibilityMethod)
    {
        case PlausibilityCombinationMethod::Average:
            combined = (rangeComponent + azimuthComponent + amplitudeComponent) / 3.0F;
            break;
        case PlausibilityCombinationMethod::Product:
            combined = rangeComponent * azimuthComponent * amplitudeComponent;
            break;
        case PlausibilityCombinationMethod::Minimum:
            combined = std::min({rangeComponent, azimuthComponent, amplitudeComponent});
            break;
        case PlausibilityCombinationMethod::Custom:
        default:
            combined = range_m > m_settings.customCombinationRangeThreshold
                           ? std::min(rangeComponent, azimuthComponent) * amplitudeComponent
                           : rangeComponent * amplitudeComponent;
            break;
    }

    return std::clamp(combined, 0.0F, 1.0F);
}

void FusedRadarMapping::computeSensorAccuracies(const RadarPoint& point,
                                                float& rangeAccuracy_m,
                                                float& angleAccuracy_rad) const
{
    const bool isMrr = isMrrSensorIndex(point.sensorIndex);
    if (isMrr)
    {
        rangeAccuracy_m = m_settings.mrrRangeAccuracy_m;
        angleAccuracy_rad = m_settings.mrrAngleAccuracy_deg * kDegToRad;
    }
    else
    {
        rangeAccuracy_m = m_settings.srrRangeAccuracy_m;
        angleAccuracy_rad = m_settings.srrAngleAccuracy_deg * kDegToRad;
    }
}

void FusedRadarMapping::addGaussian(const glm::vec2& detectionPosition,
                                    const glm::vec2& relativeVector,
                                    float range_m,
                                    float azimuth_rad,
                                    float rangeAccuracy_m,
                                    float angleAccuracy_rad,
                                    float plausibility)
{
    if (m_settings.maxAdditiveProbability <= 0.0F)
    {
        return;
    }

    const float sigmaLat = std::max(range_m * std::tan(angleAccuracy_rad), m_settings.cellSize * 0.5F);
    const float sigmaLon = std::max(rangeAccuracy_m, m_settings.cellSize * 0.5F);
    const float maxSigma = std::max(sigmaLat, sigmaLon);
    const float radius = std::max(m_settings.cellSize, 3.0F * maxSigma);

    const float invSigmaLon2 = 1.0F / (sigmaLon * sigmaLon);
    const float invSigmaLat2 = 1.0F / (sigmaLat * sigmaLat);
    const float scale = m_settings.maxAdditiveProbability * plausibility;

    glm::vec2 forward(std::sin(azimuth_rad), std::cos(azimuth_rad));
    if (glm::length(relativeVector) > 1e-3F)
    {
        forward = glm::normalize(relativeVector);
    }
    const glm::vec2 right(forward.y, -forward.x);

    const float minX = detectionPosition.x - radius;
    const float maxX = detectionPosition.x + radius;
    const float minY = detectionPosition.y - radius;
    const float maxY = detectionPosition.y + radius;
    const auto toIndex = [this](float value) { return value / m_settings.cellSize + m_gridCenter; };
    int ixMin = static_cast<int>(std::floor(toIndex(minX)));
    int ixMax = static_cast<int>(std::ceil(toIndex(maxX)));
    int iyMin = static_cast<int>(std::floor(toIndex(minY)));
    int iyMax = static_cast<int>(std::ceil(toIndex(maxY)));
    ixMin = std::clamp(ixMin, 0, m_gridSize - 1);
    ixMax = std::clamp(ixMax, 0, m_gridSize - 1);
    iyMin = std::clamp(iyMin, 0, m_gridSize - 1);
    iyMax = std::clamp(iyMax, 0, m_gridSize - 1);

    for (int iy = iyMin; iy <= iyMax; ++iy)
    {
        for (int ix = ixMin; ix <= ixMax; ++ix)
        {
            const glm::vec3 cell = cellCenter(ix, iy);
            const glm::vec2 delta(cell.x - detectionPosition.x, cell.y - detectionPosition.y);
            const float longitudinal = glm::dot(delta, forward);
            const float lateral = glm::dot(delta, right);
            const float exponent = -0.5F * ((longitudinal * longitudinal) * invSigmaLon2 +
                                            (lateral * lateral) * invSigmaLat2);
            const float gaussian = std::exp(exponent);
            float probability = 0.5F + (scale * gaussian);
            probability = std::clamp(probability, kMinProbability, kMaxProbability);
            const float logOdds = std::log(probability / (1.0F - probability));
            updateCell(ix, iy, logOdds);
        }
    }
}

void FusedRadarMapping::addHit(const glm::vec2& detectionPosition, float plausibility)
{
    int ix = 0;
    int iy = 0;
    if (!worldToCell(detectionPosition, ix, iy))
    {
        return;
    }
    updateCell(ix, iy, m_settings.hitIncrement * plausibility);
}

void FusedRadarMapping::addFreespaceCone(const glm::vec2& sensorPosition,
                                         float azimuth_rad,
                                         float range_m,
                                         float rangeAccuracy_m,
                                         float amplitude_dBsm)
{
    if (range_m > m_settings.maxFreeSpaceRange_m)
    {
        return;
    }

    const float freeSpaceRange =
        range_m - (m_settings.freespaceRangeSigmaFactor * std::max(0.0F, rangeAccuracy_m));
    if (freeSpaceRange <= 0.0F)
    {
        return;
    }

    const float freeSpacePlausibility =
        computePlausibility(std::min(freeSpaceRange, 15.0F), azimuth_rad, amplitude_dBsm);
    if (freeSpacePlausibility < m_settings.minPlausibility)
    {
        return;
    }

    const float angle = m_settings.freespaceAngleAccuracy_rad;
    const float angleLeft = azimuth_rad - angle;
    const float angleRight = azimuth_rad + angle;
    const glm::vec2 left =
        sensorPosition + freeSpaceRange * glm::vec2(std::sin(angleLeft), std::cos(angleLeft));
    const glm::vec2 right =
        sensorPosition + freeSpaceRange * glm::vec2(std::sin(angleRight), std::cos(angleRight));

    const float delta = -std::abs(m_settings.missDecrement) * freeSpacePlausibility;

    const float minX = std::min({sensorPosition.x, left.x, right.x});
    const float maxX = std::max({sensorPosition.x, left.x, right.x});
    const float minY = std::min({sensorPosition.y, left.y, right.y});
    const float maxY = std::max({sensorPosition.y, left.y, right.y});
    const auto toIndex = [this](float value) { return value / m_settings.cellSize + m_gridCenter; };
    int ixMin = static_cast<int>(std::floor(toIndex(minX)));
    int ixMax = static_cast<int>(std::ceil(toIndex(maxX)));
    int iyMin = static_cast<int>(std::floor(toIndex(minY)));
    int iyMax = static_cast<int>(std::ceil(toIndex(maxY)));
    ixMin = std::clamp(ixMin, 0, m_gridSize - 1);
    ixMax = std::clamp(ixMax, 0, m_gridSize - 1);
    iyMin = std::clamp(iyMin, 0, m_gridSize - 1);
    iyMax = std::clamp(iyMax, 0, m_gridSize - 1);

    const auto sign = [](const glm::vec2& p1, const glm::vec2& p2, const glm::vec2& p3)
    {
        return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
    };
    const auto pointInTriangle = [&sign](const glm::vec2& p, const glm::vec2& a, const glm::vec2& b, const glm::vec2& c)
    {
        const float d1 = sign(p, a, b);
        const float d2 = sign(p, b, c);
        const float d3 = sign(p, c, a);
        const bool hasNeg = (d1 < 0.0F) || (d2 < 0.0F) || (d3 < 0.0F);
        const bool hasPos = (d1 > 0.0F) || (d2 > 0.0F) || (d3 > 0.0F);
        return !(hasNeg && hasPos);
    };

    for (int iy = iyMin; iy <= iyMax; ++iy)
    {
        for (int ix = ixMin; ix <= ixMax; ++ix)
        {
            const glm::vec3 cell = cellCenter(ix, iy);
            const glm::vec2 position(cell.x, cell.y);
            if (pointInTriangle(position, sensorPosition, left, right))
            {
                updateCell(ix, iy, delta);
            }
        }
    }
}

void FusedRadarMapping::updateCell(int ix, int iy, float delta)
{
    const float& current = m_logOdds[iy * m_gridSize + ix];
    const float next = std::clamp(current + delta, m_settings.minLogOdds, m_settings.maxLogOdds);
    m_logOdds[iy * m_gridSize + ix] = next;
}

glm::vec3 FusedRadarMapping::cellCenter(int ix, int iy) const
{
    const float x = (static_cast<float>(ix) - m_gridCenter) * m_settings.cellSize + (m_settings.cellSize * 0.5F);
    const float y = (static_cast<float>(iy) - m_gridCenter) * m_settings.cellSize + (m_settings.cellSize * 0.5F);
    return glm::vec3(x, y, 0.0F);
}

void FusedRadarMapping::initializeGrid()
{
    m_gridSize = std::max(3, static_cast<int>(std::ceil((m_settings.mapRadius * 2.0F) / m_settings.cellSize)));
    m_gridCenter = (static_cast<float>(m_gridSize) - 1.0F) * 0.5F;
    m_logOdds.assign(m_gridSize * m_gridSize, 0.0F);
}

} // namespace radar
