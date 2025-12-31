#include "sensors/RadarFactoryHelpers.hpp"

#include <algorithm>
#include <iostream>
#include <utility>

namespace fs = std::filesystem;

namespace radar
{
namespace
{
void appendUniqueCandidate(std::vector<std::filesystem::path>& candidates,
                           std::filesystem::path candidate)
{
    if (candidate.empty())
    {
        return;
    }

    candidate = candidate.lexically_normal();
    if (candidate.empty())
    {
        return;
    }

    if (std::find(candidates.begin(), candidates.end(), candidate) == candidates.end())
    {
        candidates.push_back(std::move(candidate));
    }
}

fs::path radarLogPath()
{
    return fs::current_path() / "radar_reader.log";
}

std::unique_ptr<BaseRadarSensor> instantiateDefaultTextRadarSensor(fs::path path)
{
    Logger::initialize(radarLogPath());
    Logger::log(Logger::Level::Info,
                "instantiateDefaultTextRadarSensor with path: " + path.string() +
                    (fs::exists(path) ? " (exists)" : " (missing)"));
    try
    {
        return std::make_unique<TextRadarSensor>(std::move(path));
    }
    catch (const std::exception& ex)
    {
        Logger::log(Logger::Level::Error,
                    std::string("instantiateDefaultTextRadarSensor threw exception: ") + ex.what());
        throw;
    }
    catch (...)
    {
        Logger::log(Logger::Level::Error, "instantiateDefaultTextRadarSensor threw unknown exception");
        throw;
    }
}

factory::TextRadarSensorFactory s_textRadarSensorFactory = instantiateDefaultTextRadarSensor;

} // namespace

namespace factory
{
void ensureLoggerInitialized()
{
    static bool initialized = false;
    if (!initialized)
    {
        const fs::path logPath = radarLogPath();
        Logger::initialize(logPath);
        Logger::log(Logger::Level::Info, "Radar log initialized at " + logPath.string());
        initialized = true;
    }
}

std::unique_ptr<BaseRadarSensor> instantiateTextRadarSensor(fs::path path)
{
    return s_textRadarSensorFactory(std::move(path));
}

void setTextRadarSensorFactory(TextRadarSensorFactory factory)
{
    s_textRadarSensorFactory = factory ? factory : instantiateDefaultTextRadarSensor;
}

std::unique_ptr<BaseRadarSensor> createTextRadarSensor(const std::string& filename)
{
    ensureLoggerInitialized();
    fs::path requested(filename);
    Logger::log(Logger::Level::Info, "Resolving radar data file: " + filename);
    for (const fs::path& candidate : radarDataCandidatePaths(filename))
    {
        if (fs::exists(candidate))
        {
            requested = candidate;
            break;
        }
        Logger::log(Logger::Level::Info, "Checked candidate path: " + candidate.string());
    }

    if (!fs::exists(requested))
    {
        std::cerr << "Radar data file not found: " << requested << '\n';
        Logger::log(Logger::Level::Error, "Radar data file not found: " + requested.string());
        return nullptr;
    }

    Logger::log(Logger::Level::Info, "Creating text radar sensor from: " + requested.string());

    auto sensor = instantiateTextRadarSensor(std::move(requested));
    if (sensor)
    {
        Logger::log(Logger::Level::Info, "Text radar sensor successfully instantiated");
    }
    return sensor;
}

std::vector<std::filesystem::path> radarDataCandidatePaths(const std::string& filename,
                                                           const std::filesystem::path& workingDir)
{
    std::vector<std::filesystem::path> candidates;
    if (!workingDir.empty())
    {
        appendUniqueCandidate(candidates, workingDir / "data" / filename);
    }

    return candidates;
}

std::optional<std::filesystem::path> resolveRadarDataFile(const std::string& filename,
                                                           const std::filesystem::path& workingDir)
{
    for (const auto& candidate : radarDataCandidatePaths(filename, workingDir))
    {
        if (std::filesystem::exists(candidate))
        {
            return candidate;
        }
    }
    return std::nullopt;
}

} // namespace factory

} // namespace radar
