#include "engine/RadarEngine.hpp"

#include "config/VehicleProfile.hpp"
#include "logging/Logger.hpp"
#include "sensors/OfflineRadarSensor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <thread>
#include <vector>

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
} // namespace

namespace radar
{

RadarEngine::RadarEngine(std::unique_ptr<BaseRadarSensor> sensor)
    : m_sensor(std::move(sensor))
{
}

RadarEngine::~RadarEngine() = default;

bool RadarEngine::initialize()
{
    if (!m_sensor)
    {
        std::cerr << "No radar sensor configured\n";
        return false;
    }

    Logger::initialize(std::filesystem::current_path() / "radar_reader.log");
    Logger::log(Logger::Level::Info, "Initializing radar engine for sensor: " + m_sensor->identifier());
    Logger::log(Logger::Level::Info, "Configuring sensor with max range 120.0");
    m_sensor->configure(120.0F);
    std::cout << "Preparing radar sensor " << m_sensor->identifier() << '\n';
    if (const VehicleProfile* profile = m_sensor->vehicleProfile())
    {
        const float distRearAxle = profile->distRearAxle();
        const auto& contourIso = profile->contourPoints();
        const auto contourVcs = convertContourIsoToVcs(contourIso, distRearAxle);
        m_visualizer.updateVehicleContour(contourIso);
        m_mapping.setVehicleContour(contourVcs);
        m_visualizer.setVcsToIsoTransform(distRearAxle);
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

void RadarEngine::run()
{
    if (!initialize())
    {
        return;
    }

    while (!m_visualizer.windowShouldClose())
    {
        const auto frameStart = std::chrono::steady_clock::now();

        uint64_t timestampUs = 0U;
        if (!captureFrame(timestampUs))
        {
            std::cerr << "Radar sensor exhausted the capture" << '\n';
            break;
        }

        BaseRadarSensor::PointCloud& currentBuffer = m_pointBuffers[m_readIndex];
        m_visualizer.updatePoints(currentBuffer, timestampUs, m_currentSources);
        m_mapPoints.clear();
        m_mapPoints.reserve(currentBuffer.size());
        for (const auto& point : currentBuffer)
        {
            m_mapPoints.emplace_back(point.x, point.y);
        }

        const std::size_t desiredSegments = m_visualizer.mapSegmentCount();
        if (desiredSegments != m_lastSegmentCount)
        {
            m_mapping.setSegmentCount(desiredSegments);
            m_lastSegmentCount = desiredSegments;
        }

        m_mapping.update(m_mapPoints, {});
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

        m_readIndex = (m_readIndex + 1U) % m_pointBuffers.size();

        std::chrono::microseconds targetDurationUs =
            std::chrono::duration_cast<std::chrono::microseconds>(kTargetFrameDuration);
        if (m_hasPreviousTimestamp && timestampUs > m_previousTimestampUs)
        {
            targetDurationUs = std::chrono::microseconds(timestampUs - m_previousTimestampUs);
        }
        m_previousTimestampUs = timestampUs;
        m_hasPreviousTimestamp = true;

        const float speedScale = std::max(0.01F, m_visualizer.frameSpeedScale());
        const std::int64_t scaledUs = static_cast<std::int64_t>(targetDurationUs.count() / speedScale);
        const auto scaledTarget =
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::microseconds(std::max<std::int64_t>(1, scaledUs)));
        const auto frameDuration = std::chrono::steady_clock::now() - frameStart;
        if (frameDuration < scaledTarget)
        {
            std::this_thread::sleep_for(scaledTarget - frameDuration);
        }
    }
}

bool RadarEngine::captureFrame(uint64_t& timestampUs)
{
    BaseRadarSensor::PointCloud& buffer = m_pointBuffers[m_readIndex];
    buffer.clear();
    if (!m_sensor->readNextScan(buffer, timestampUs))
    {
        return false;
    }

    if (auto* offlineSensor = dynamic_cast<OfflineRadarSensor*>(m_sensor.get()))
    {
        m_currentSources = offlineSensor->lastFrameSources();
    }
    else
    {
        m_currentSources.clear();
    }
    return true;
}

} // namespace radar
