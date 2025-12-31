#pragma once

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "logging/Logger.hpp"
#include "sensors/BaseRadarSensor.hpp"
#include "sensors/TextRadarSensor.hpp"
namespace radar
{
namespace factory
{

void ensureLoggerInitialized();


std::vector<std::filesystem::path> radarDataCandidatePaths(const std::string& filename,
                                                           const std::filesystem::path& workingDir =
                                                               std::filesystem::current_path());

std::optional<std::filesystem::path> resolveRadarDataFile(const std::string& filename,
                                                           const std::filesystem::path& workingDir =
                                                               std::filesystem::current_path());

using TextRadarSensorFactory = std::unique_ptr<BaseRadarSensor> (*)(std::filesystem::path path);

void setTextRadarSensorFactory(TextRadarSensorFactory factory);

std::unique_ptr<BaseRadarSensor> createTextRadarSensor(const std::string& filename);

} // namespace factory

} // namespace radar
