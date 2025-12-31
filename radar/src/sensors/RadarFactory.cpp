#include "sensors/RadarFactoryHelpers.hpp"
#include "sensors/OfflineRadarSensor.hpp"
#include "sensors/MultiRadarSensor.hpp"
#include "sensors/RadarFactory.hpp"

namespace radar
{

namespace fs = std::filesystem;

namespace
{
std::unique_ptr<BaseRadarSensor> createTextSensor(const std::string& filename)
{
    return factory::createTextRadarSensor(filename);
}
} // namespace

std::unique_ptr<BaseRadarSensor> RadarFactory::createSensor(const std::vector<std::string>& filenames)
{
    factory::ensureLoggerInitialized();
    if (filenames.empty())
    {
        Logger::log(Logger::Level::Info, "No radar data filenames provided, falling back to offline sensor.");
        return std::make_unique<OfflineRadarSensor>();
    }

    std::vector<std::unique_ptr<BaseRadarSensor>> sensors;
    sensors.reserve(filenames.size());
    for (const std::string& filename : filenames)
    {
        auto sensor = createTextSensor(filename);
        if (sensor)
        {
            sensors.push_back(std::move(sensor));
        }
    }

    if (sensors.empty())
    {
        Logger::log(Logger::Level::Error, "Failed to load any radar data files.");
        return nullptr;
    }

    if (sensors.size() == 1)
    {
        Logger::log(Logger::Level::Info, "Loaded single radar sensor: " + sensors.front()->identifier());
        return std::move(sensors.front());
    }

    Logger::log(Logger::Level::Info, "Loaded multi-radar sensor with " + std::to_string(sensors.size()) + " sources.");
    return std::make_unique<MultiRadarSensor>(std::move(sensors));
}

} // namespace radar
