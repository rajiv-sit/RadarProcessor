#pragma once

#include "mapping/RadarVirtualSensorMapping.hpp"
#include "processing/RadarPlayback.hpp"
#include "visualization/RadarVisualizer.hpp"

#include <glm/glm.hpp>

#include <chrono>
#include <cstdint>
#include <vector>

namespace radar
{

class RadarPlaybackEngine
{
public:
    explicit RadarPlaybackEngine(RadarPlayback playback);

    bool initialize();
    void run();

private:
    static constexpr std::chrono::milliseconds kTargetFrameDuration{33};

    RadarPlayback m_playback;
    visualization::RadarVisualizer m_visualizer;
    RadarVirtualSensorMapping m_mapping;
    std::vector<glm::vec2> m_mapPoints;
    std::vector<glm::vec3> m_mapVertices;
    std::vector<glm::vec3> m_mapSegmentVertices;
    std::vector<RadarTrack> m_latestTracks;
    std::size_t m_lastSegmentCount = 0U;
    uint64_t m_previousTimestampUs = 0U;
    bool m_hasPreviousTimestamp = false;
};

} // namespace radar
