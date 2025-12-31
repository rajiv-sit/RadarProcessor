#include "mapping/RadarVirtualSensorMapping.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace radar
{
namespace
{
constexpr float kEpsilon = 1e-5F;

float cross2(const glm::vec2& a, const glm::vec2& b)
{
    return a.x * b.y - a.y * b.x;
}
} // namespace

RadarVirtualSensorMapping::RadarVirtualSensorMapping()
{
    setSegmentCount(kDefaultSegmentCount);
}

bool RadarVirtualSensorMapping::setSegmentCount(std::size_t count)
{
    const std::size_t clamped = std::max<std::size_t>(3U, count);
    if (clamped == m_segmentCount && !m_segmentDirections.empty())
    {
        return false;
    }

    m_segmentCount = clamped;
    m_segmentDirections.assign(m_segmentCount, glm::vec2(0.0F));
    m_segmentStartDist.assign(m_segmentCount, 0.0F);
    m_segmentEndDist.assign(m_segmentCount, std::numeric_limits<float>::infinity());

    rebuildSegments();

    if (m_vehicleContour.size() >= 3U)
    {
        setVehicleContour(m_vehicleContour);
    }
    else
    {
        m_ready = false;
    }
    return true;
}

std::size_t RadarVirtualSensorMapping::segmentCount() const
{
    return m_segmentCount;
}

void RadarVirtualSensorMapping::setVehicleContour(const std::vector<glm::vec2>& contour)
{
    if (contour.size() < 3U)
    {
        return;
    }

    m_vehicleContour = contour;

    glm::vec2 center(0.0F);
    for (const auto& point : contour)
    {
        center += point;
    }
    center /= static_cast<float>(contour.size());
    m_vehicleCenter = center;

    for (std::size_t i = 0; i < m_segmentCount; ++i)
    {
        const float distance = contourRayDistance(m_vehicleCenter, m_segmentDirections[i]);
        m_segmentStartDist[i] = std::max(0.0F, distance);
    }

    m_ready = true;
}

void RadarVirtualSensorMapping::update(const std::vector<glm::vec2>& detections,
                                       const std::vector<std::array<glm::vec2, 4>>& trackFootprints)
{
    resetSegments();

    if (!m_ready)
    {
        return;
    }

    for (const auto& point : detections)
    {
        const glm::vec2 delta = point - m_vehicleCenter;
        const float distance = glm::length(delta);
        if (!std::isfinite(distance) || distance <= kEpsilon)
        {
            continue;
        }

        const std::size_t idx = segmentIndex(std::atan2(delta.y, delta.x));
        if (distance <= m_segmentStartDist[idx] + kEpsilon)
        {
            continue;
        }
        if (distance < m_segmentEndDist[idx])
        {
            m_segmentEndDist[idx] = distance;
        }
    }

    for (const auto& footprint : trackFootprints)
    {
        for (std::size_t i = 0; i < m_segmentCount; ++i)
        {
            const float distance = polygonRayDistance(m_vehicleCenter, m_segmentDirections[i], footprint);
            if (!std::isfinite(distance) || distance <= kEpsilon)
            {
                continue;
            }
            if (distance <= m_segmentStartDist[i] + kEpsilon)
            {
                continue;
            }
            if (distance < m_segmentEndDist[i])
            {
                m_segmentEndDist[i] = distance;
            }
        }
    }
}

void RadarVirtualSensorMapping::reset()
{
    resetSegments();
}

std::vector<glm::vec2> RadarVirtualSensorMapping::ring(float fallbackRange) const
{
    std::vector<glm::vec2> ringPoints;
    if (!m_ready || fallbackRange <= 0.0F)
    {
        return ringPoints;
    }

    ringPoints.reserve(m_segmentCount);
    for (std::size_t i = 0; i < m_segmentCount; ++i)
    {
        float length = std::min(m_segmentEndDist[i], fallbackRange);
        length = std::max(length, m_segmentStartDist[i]);
        ringPoints.push_back(m_vehicleCenter + m_segmentDirections[i] * length);
    }

    return ringPoints;
}

std::vector<RadarVirtualSensorMapping::Segment> RadarVirtualSensorMapping::segments(float fallbackRange) const
{
    std::vector<Segment> output;
    if (!m_ready || fallbackRange <= 0.0F)
    {
        return output;
    }

    output.reserve(m_segmentCount);
    for (std::size_t i = 0; i < m_segmentCount; ++i)
    {
        float length = std::min(m_segmentEndDist[i], fallbackRange);
        length = std::max(length, m_segmentStartDist[i]);
        const glm::vec2 start = m_vehicleCenter + m_segmentDirections[i] * m_segmentStartDist[i];
        const glm::vec2 end = m_vehicleCenter + m_segmentDirections[i] * length;
        output.push_back({start, end});
    }

    return output;
}

void RadarVirtualSensorMapping::rebuildSegments()
{
    if (m_segmentCount == 0U)
    {
        return;
    }

    const float delta = glm::two_pi<float>() / static_cast<float>(m_segmentCount);
    for (std::size_t i = 0; i < m_segmentCount; ++i)
    {
        const float angle = (static_cast<float>(i) + 0.5F) * delta;
        m_segmentDirections[i] = glm::vec2(std::cos(angle), std::sin(angle));
        m_segmentStartDist[i] = 0.0F;
    }
}

void RadarVirtualSensorMapping::resetSegments()
{
    std::fill(m_segmentEndDist.begin(), m_segmentEndDist.end(), std::numeric_limits<float>::infinity());
}

float RadarVirtualSensorMapping::normalizeAngle(float angle)
{
    constexpr float twoPi = glm::two_pi<float>();
    float normalized = std::fmod(angle, twoPi);
    if (normalized < 0.0F)
    {
        normalized += twoPi;
    }
    return normalized;
}

std::size_t RadarVirtualSensorMapping::segmentIndex(float angle) const
{
    if (m_segmentCount == 0U)
    {
        return 0U;
    }
    const float normalized = normalizeAngle(angle);
    const float scale = normalized / glm::two_pi<float>();
    const std::size_t idx = static_cast<std::size_t>(scale * static_cast<float>(m_segmentCount));
    return std::min(idx, m_segmentCount - 1U);
}

bool RadarVirtualSensorMapping::raySegmentIntersection(const glm::vec2& origin,
                                                       const glm::vec2& direction,
                                                       const glm::vec2& a,
                                                       const glm::vec2& b,
                                                       float& tOut) const
{
    const glm::vec2 edge = b - a;
    const float denom = cross2(direction, edge);
    if (std::fabs(denom) < kEpsilon)
    {
        return false;
    }

    const glm::vec2 delta = a - origin;
    const float t = cross2(delta, edge) / denom;
    const float u = cross2(delta, direction) / denom;
    if (t >= 0.0F && u >= 0.0F && u <= 1.0F)
    {
        tOut = t;
        return true;
    }
    return false;
}

float RadarVirtualSensorMapping::contourRayDistance(const glm::vec2& origin,
                                                    const glm::vec2& direction) const
{
    if (m_vehicleContour.size() < 3U)
    {
        return 0.0F;
    }

    float best = std::numeric_limits<float>::infinity();
    const std::size_t count = m_vehicleContour.size();
    for (std::size_t i = 0; i < count; ++i)
    {
        const glm::vec2& a = m_vehicleContour[i];
        const glm::vec2& b = m_vehicleContour[(i + 1U) % count];
        float t = 0.0F;
        if (raySegmentIntersection(origin, direction, a, b, t))
        {
            best = std::min(best, t);
        }
    }

    if (!std::isfinite(best))
    {
        return 0.0F;
    }
    return best;
}

float RadarVirtualSensorMapping::polygonRayDistance(const glm::vec2& origin,
                                                    const glm::vec2& direction,
                                                    const std::array<glm::vec2, 4>& polygon) const
{
    float best = std::numeric_limits<float>::infinity();
    for (std::size_t i = 0; i < polygon.size(); ++i)
    {
        const glm::vec2& a = polygon[i];
        const glm::vec2& b = polygon[(i + 1U) % polygon.size()];
        float t = 0.0F;
        if (raySegmentIntersection(origin, direction, a, b, t))
        {
            best = std::min(best, t);
        }
    }

    if (!std::isfinite(best))
    {
        return std::numeric_limits<float>::infinity();
    }
    return best;
}

} // namespace radar
