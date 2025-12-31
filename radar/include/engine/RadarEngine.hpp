#pragma once

#include "mapping/RadarVirtualSensorMapping.hpp"
#include "sensors/BaseRadarSensor.hpp"
#include "visualization/RadarVisualizer.hpp"

#include <glm/glm.hpp>

#include <array>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <vector>

namespace radar
{

class RadarEngine
{
public:
    explicit RadarEngine(std::unique_ptr<BaseRadarSensor> sensor);
    ~RadarEngine();

    bool initialize();
    void run();

private:
    bool captureFrame(uint64_t& timestampUs);

    static constexpr std::chrono::milliseconds kTargetFrameDuration{33};

    std::unique_ptr<BaseRadarSensor> m_sensor;
    visualization::RadarVisualizer m_visualizer;
    std::array<BaseRadarSensor::PointCloud, 2> m_pointBuffers;
    size_t m_readIndex = 0U;
    RadarVirtualSensorMapping m_mapping;
    std::vector<glm::vec2> m_mapPoints;
    std::vector<glm::vec3> m_mapVertices;
    std::vector<glm::vec3> m_mapSegmentVertices;
    std::size_t m_lastSegmentCount = 0U;
    uint64_t m_previousTimestampUs = 0U;
    bool m_hasPreviousTimestamp = false;

    struct RadarFrame
    {
        BaseRadarSensor::PointCloud points;
        uint64_t timestampUs = 0U;
        std::vector<std::string> sources;
    };

    void startReader();
    void stopReader();
    void readerLoop();

    std::thread m_readerThread;
    std::deque<RadarFrame> m_frameQueue;
    std::mutex m_queueMutex;
    std::condition_variable m_queueCond;
    bool m_readerRunning = false;
    bool m_readerFinished = false;
    bool m_stopReader = false;
    std::vector<std::string> m_currentSources;
};

} // namespace radar
