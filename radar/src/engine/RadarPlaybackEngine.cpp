#include "engine/RadarPlaybackEngine.hpp"

#include "logging/Logger.hpp"
#include "utility/radar_types.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <utility>

namespace
{
constexpr float kMapMaxRange = 120.0F;

std::vector<glm::vec2> convertContourIsoToVcs(const std::vector<glm::vec2>& isoContour,
                                              float distRearAxle)
{
    std::vector<glm::vec2> vcsContour;
    vcsContour.reserve(isoContour.size());
    for (const auto& point : isoContour)
    {
        const float lateralVcs = -point.x;
        const float longitudinalVcs = point.y - distRearAxle;
        vcsContour.emplace_back(lateralVcs, longitudinalVcs);
    }
    return vcsContour;
}

std::array<glm::vec2, 4> buildTrackFootprint(const radar::RadarTrack& track)
{
    const float halfLength = std::max(track.length, 0.1F) * 0.5F;
    const float halfWidth = std::max(track.width, 0.1F) * 0.5F;

    const glm::vec2 center(track.isoPosition.y, track.isoPosition.x);
    const float heading = track.headingRad;
    const glm::vec2 forward(std::sin(heading), std::cos(heading));
    const glm::vec2 right(forward.y, -forward.x);

    const glm::vec2 p0 = center + forward * halfLength + right * halfWidth;
    const glm::vec2 p1 = center - forward * halfLength + right * halfWidth;
    const glm::vec2 p2 = center - forward * halfLength - right * halfWidth;
    const glm::vec2 p3 = center + forward * halfLength - right * halfWidth;

    return {p0, p1, p2, p3};
}
} // namespace

namespace radar
{

RadarPlaybackEngine::RadarPlaybackEngine(RadarPlayback playback)
    : m_playback(std::move(playback))
{
}

bool RadarPlaybackEngine::initialize()
{
    if (!m_playback.initialize())
    {
        std::cerr << "Failed to initialize radar playback\n";
        return false;
    }

    float distRearAxle = 0.0F;
    if (const auto* props = m_playback.vehicleParameters())
    {
        distRearAxle = props->distRearAxleToFrontBumper_m;
        m_visualizer.setVcsToIsoTransform(distRearAxle);
    }
    if (const auto& contour = m_playback.vehicleContour(); !contour.empty())
    {
        const auto contourVcs = convertContourIsoToVcs(contour, distRearAxle);
        m_visualizer.updateVehicleContour(contour);
        m_mapping.setVehicleContour(contourVcs);
    }

    m_visualizer.setResetMapCallback(
        [this]()
        {
            m_mapping.reset();
            m_visualizer.updateMapPoints({});
            m_visualizer.updateMapSegments({});
        });

    const bool visualizerReady = m_visualizer.initialize();
    Logger::log(Logger::Level::Info, visualizerReady ? "Visualizer initialized" : "Visualizer failed to initialize");
    return visualizerReady;
}

void RadarPlaybackEngine::run()
{
    if (!initialize())
    {
        return;
    }

    RadarFrame frame;
    while (!m_visualizer.windowShouldClose())
    {
        const auto frameStart = std::chrono::steady_clock::now();

        if (!m_playback.readNextFrame(frame))
        {
            std::cerr << "Radar playback has no more data\n";
            break;
        }

        if (frame.hasDetections)
        {
            m_visualizer.updatePoints(frame.detections, frame.timestampUs, frame.sources);
        }
        else
        {
            m_visualizer.updateFrameInfo(frame.timestampUs, frame.sources);
        }

        if (frame.hasTracks)
        {
            m_visualizer.updateTracks(frame.tracks);
            m_latestTracks = frame.tracks;
        }

        m_mapPoints.clear();
        m_mapPoints.reserve(frame.detections.size());
        for (const auto& point : frame.detections)
        {
            m_mapPoints.emplace_back(point.x, point.y);
        }

        const std::size_t desiredSegments = m_visualizer.mapSegmentCount();
        if (desiredSegments != m_lastSegmentCount)
        {
            m_mapping.setSegmentCount(desiredSegments);
            m_lastSegmentCount = desiredSegments;
        }

        std::vector<std::array<glm::vec2, 4>> trackFootprints;
        trackFootprints.reserve(m_latestTracks.size());
        for (const auto& track : m_latestTracks)
        {
            trackFootprints.push_back(buildTrackFootprint(track));
        }

        m_mapping.update(m_mapPoints, trackFootprints);
        const auto ring = m_mapping.ring(kMapMaxRange);
        const auto segments = m_mapping.segments(kMapMaxRange);
        m_mapVertices.clear();
        m_mapVertices.reserve(ring.size());
        for (const auto& point : ring)
        {
            m_mapVertices.emplace_back(point.x, point.y, 0.0F);
        }
        m_mapSegmentVertices.clear();
        m_mapSegmentVertices.reserve(segments.size() * 2U);
        for (const auto& segment : segments)
        {
            m_mapSegmentVertices.emplace_back(segment.start.x, segment.start.y, 0.0F);
            m_mapSegmentVertices.emplace_back(segment.end.x, segment.end.y, 0.0F);
        }
        m_visualizer.updateMapPoints(m_mapVertices);
        m_visualizer.updateMapSegments(m_mapSegmentVertices);

        m_visualizer.render();

        std::chrono::microseconds targetDurationUs =
            std::chrono::duration_cast<std::chrono::microseconds>(kTargetFrameDuration);
        if (m_hasPreviousTimestamp && frame.timestampUs > m_previousTimestampUs)
        {
            targetDurationUs = std::chrono::microseconds(frame.timestampUs - m_previousTimestampUs);
        }
        m_previousTimestampUs = frame.timestampUs;
        m_hasPreviousTimestamp = true;

        const float speedScale = std::max(0.01F, m_visualizer.frameSpeedScale());
        const std::int64_t scaledUs = static_cast<std::int64_t>(targetDurationUs.count() / speedScale);
        const auto scaledTarget =
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::microseconds(std::max<std::int64_t>(1, scaledUs)));
        const auto frameDuration = std::chrono::steady_clock::now() - frameStart;
        if (frameDuration < scaledTarget)
        {
            std::this_thread::sleep_for(scaledTarget - frameDuration);
        }
    }
}

} // namespace radar
