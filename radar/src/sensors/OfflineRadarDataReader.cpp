#include "logging/Logger.hpp"
#include "sensors/OfflineRadarDataReader.hpp"

#include "sensors/TextRadarSensor.hpp"

#include <filesystem>
#include <iostream>
#include <limits>
#include <numeric>
#include <utility>

namespace radar
{

namespace fs = std::filesystem;

OfflineRadarDataReader::OfflineRadarDataReader(std::filesystem::path dataDirectory,
                                               std::vector<std::string> dataFiles)
    : m_dataDirectory(std::move(dataDirectory))
    , m_files(std::move(dataFiles))
{
    for (const std::string& filename : m_files)
    {
        const fs::path resolved = findRadarFile(filename);
        if (resolved.empty())
        {
            Logger::log(Logger::Level::Warning, "Offline radar file missing: " + filename);
            continue;
        }

        auto sensor = std::make_unique<TextRadarSensor>(resolved);
        m_sensors.push_back(std::move(sensor));
        m_frames.emplace_back();
        Logger::log(Logger::Level::Info, "Loaded radar file: " + resolved.string());
    }
}

bool OfflineRadarDataReader::configure(float maxRangeMeters)
{
    bool configured = false;
    for (const auto& sensor : m_sensors)
    {
        sensor->configure(maxRangeMeters);
        configured = true;
    }
    if (configured)
    {
        Logger::log(Logger::Level::Info,
                    "Offline radar data reader configured with max range " + std::to_string(maxRangeMeters));
    }
    return configured;
}

bool OfflineRadarDataReader::readNextScan(BaseRadarSensor::PointCloud& destination, uint64_t& timestampUs)
{
    if (m_sensors.empty())
    {
        return false;
    }

    if (!prepareFrames())
    {
        return false;
    }

    size_t bestIndex = m_frames.size();
    uint64_t bestTimestamp = std::numeric_limits<uint64_t>::max();
    for (size_t index = 0; index < m_frames.size(); ++index)
    {
        const SourceFrame& frame = m_frames.at(index);
        if (!frame.ready)
        {
            continue;
        }
        if (frame.timestampUs < bestTimestamp)
        {
            bestTimestamp = frame.timestampUs;
            bestIndex = index;
        }
    }

    if (bestIndex == m_frames.size())
    {
        return false;
    }

    destination.clear();
    timestampUs = bestTimestamp;
    m_lastFrameSources.clear();

    for (size_t index = 0; index < m_frames.size(); ++index)
    {
        SourceFrame& frame = m_frames.at(index);
        if (!frame.ready || frame.timestampUs != bestTimestamp)
        {
            continue;
        }
        destination.insert(destination.end(), frame.points.begin(), frame.points.end());
        frame.points.clear();
        frame.ready = false;
        if (index < m_files.size())
        {
            m_lastFrameSources.push_back(m_files.at(index));
        }
    }

    Logger::log(Logger::Level::Info,
                "Read combined scan at " + std::to_string(timestampUs) + "us with " + std::to_string(destination.size()) +
                    " points and sources: " + (m_lastFrameSources.empty() ? "none"
                                                                     : std::accumulate(m_lastFrameSources.begin(),
                                                                                       m_lastFrameSources.end(),
                                                                                       std::string(),
                                                                                       [](const std::string& accumulated,
                                                                                          const std::string& next)
                                                                                       {
                                                                                           return accumulated.empty()
                                                                                                      ? next
                                                                                                      : accumulated + ", " + next;
                                                                                       })));
    return true;
}

bool OfflineRadarDataReader::prepareFrames()
{
    bool anyReady = false;
    for (size_t index = 0; index < m_sensors.size(); ++index)
    {
        SourceFrame& frame = m_frames.at(index);
        if (frame.ready || frame.finished)
        {
            if (frame.ready)
            {
                anyReady = true;
            }
            continue;
        }

        if (m_sensors.at(index)->readNextScan(frame.points, frame.timestampUs))
        {
            frame.ready = true;
            frame.finished = false;
            anyReady = true;
        }
        else
        {
            frame.points.clear();
            frame.ready = false;
            frame.finished = true;
            if (index < m_files.size())
            {
                Logger::log(Logger::Level::Info, "Completed reading from " + m_files.at(index));
            }
        }
    }

    return anyReady;
}

std::filesystem::path OfflineRadarDataReader::findRadarFile(const std::string& filename) const
{
    const std::vector<fs::path> searchRoots = {
        m_dataDirectory,
        fs::current_path(),
        fs::current_path() / "data",
        fs::current_path().parent_path() / "data",
        fs::current_path().parent_path().parent_path() / "data",
        fs::current_path() / ".." / "data",
        fs::current_path() / ".." / ".." / "Test" / "data",
        fs::current_path() / "Test" / "data",
    };

    for (const fs::path& root : searchRoots)
    {
        if (root.empty())
        {
            continue;
        }

        const fs::path candidate = root / filename;
        if (fs::exists(candidate))
        {
            return fs::weakly_canonical(candidate);
        }
    }

    return {};
}

const std::vector<std::string>& OfflineRadarDataReader::lastFrameSources() const noexcept
{
    return m_lastFrameSources;
}

} // namespace radar
