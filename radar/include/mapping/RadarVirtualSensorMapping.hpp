#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

#include <array>
#include <cstddef>
#include <limits>
#include <vector>

namespace radar
{

class RadarVirtualSensorMapping
{
public:
    static constexpr std::size_t kDefaultSegmentCount = 72U;

    RadarVirtualSensorMapping();

    bool setSegmentCount(std::size_t count);
    std::size_t segmentCount() const;

    void setVehicleContour(const std::vector<glm::vec2>& contour);
    void update(const std::vector<glm::vec2>& detections,
                const std::vector<std::array<glm::vec2, 4>>& trackFootprints);
    void reset();

    std::vector<glm::vec2> ring(float fallbackRange) const;
    struct Segment
    {
        glm::vec2 start;
        glm::vec2 end;
    };
    std::vector<Segment> segments(float fallbackRange) const;

private:
    void rebuildSegments();
    void resetSegments();
    static float normalizeAngle(float angle);
    std::size_t segmentIndex(float angle) const;
    bool raySegmentIntersection(const glm::vec2& origin,
                                const glm::vec2& direction,
                                const glm::vec2& a,
                                const glm::vec2& b,
                                float& tOut) const;
    float contourRayDistance(const glm::vec2& origin, const glm::vec2& direction) const;
    float polygonRayDistance(const glm::vec2& origin,
                             const glm::vec2& direction,
                             const std::array<glm::vec2, 4>& polygon) const;

    std::vector<glm::vec2> m_vehicleContour;
    glm::vec2 m_vehicleCenter = glm::vec2(0.0F);
    std::size_t m_segmentCount = kDefaultSegmentCount;
    std::vector<glm::vec2> m_segmentDirections;
    std::vector<float> m_segmentStartDist;
    std::vector<float> m_segmentEndDist;
    bool m_ready = false;
};

} // namespace radar
