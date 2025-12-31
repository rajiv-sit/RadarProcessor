#include "visualization/RadarVisualizer.hpp"

#include <algorithm>
#include <unordered_map>

namespace visualization
{
namespace
{
std::unordered_map<const RadarVisualizer*, int> g_renderCounts;
constexpr int kCloseAfterFrames = 1;
constexpr int kSegmentMin = 12;
constexpr int kSegmentMax = 360;
} // namespace

RadarVisualizer::~RadarVisualizer()
{
    g_renderCounts.erase(this);
}

bool RadarVisualizer::initialize()
{
    g_renderCounts[this] = 0;
    return true;
}

void RadarVisualizer::updatePoints(const radar::BaseRadarSensor::PointCloud&,
                                   uint64_t,
                                   const std::vector<std::string>&)
{
}

void RadarVisualizer::updateFrameInfo(uint64_t, const std::vector<std::string>&)
{
}

void RadarVisualizer::updateTracks(const std::vector<radar::RadarTrack>&)
{
}

void RadarVisualizer::updateMapPoints(const std::vector<glm::vec3>&)
{
}

void RadarVisualizer::updateMapSegments(const std::vector<glm::vec3>&)
{
}

void RadarVisualizer::updateVehicleContour(const std::vector<glm::vec2>&)
{
}

void RadarVisualizer::setVcsToIsoTransform(float distRearAxle)
{
    m_vcsToIsoEnabled = true;
    m_vcsToIsoLongitudinalOffset = distRearAxle;
}

void RadarVisualizer::setResetMapCallback(std::function<void()> callback)
{
    m_resetMapCallback = std::move(callback);
}

void RadarVisualizer::render()
{
    auto& count = g_renderCounts[this];
    ++count;
}

bool RadarVisualizer::windowShouldClose() const
{
    auto it = g_renderCounts.find(this);
    if (it == g_renderCounts.end())
    {
        return false;
    }
    return it->second >= kCloseAfterFrames;
}

float RadarVisualizer::frameSpeedScale() const
{
    return 1.0f;
}

std::size_t RadarVisualizer::mapSegmentCount() const
{
    const int clamped = std::clamp(m_mapSegmentCount, kSegmentMin, kSegmentMax);
    return static_cast<std::size_t>(clamped);
}

} // namespace visualization
