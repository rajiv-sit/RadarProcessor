#include "logging/Logger.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <system_error>
#include <ctime>

namespace radar
{

namespace
{
std::ofstream s_stream;
bool s_streamReady = false;
} // namespace

std::mutex Logger::s_mutex;

void Logger::initialize(const std::filesystem::path& logPath)
{
    std::lock_guard<std::mutex> lock(s_mutex);
    if (s_stream.is_open())
    {
        return;
    }

    const std::filesystem::path directory = logPath.parent_path();
    if (!directory.empty())
    {
        std::error_code ec;
        std::filesystem::create_directories(directory, ec);
    }

    s_stream.open(logPath, std::ios::app);
    s_streamReady = s_stream.is_open();
    if (s_streamReady)
    {
        s_stream << buildMessage(Level::Info, "Radar logger initialized at " + logPath.string()) << '\n';
        s_stream.flush();
    }
}

void Logger::log(Level level, const std::string& message)
{
    const std::string formatted = buildMessage(level, message);

    std::lock_guard<std::mutex> lock(s_mutex);
    std::cout << formatted << '\n';
    if (s_streamReady)
    {
        s_stream << formatted << '\n';
        s_stream.flush();
    }
}

const char* Logger::levelName(Level level)
{
    switch (level)
    {
    case Level::Info:
        return "INFO";
    case Level::Warning:
        return "WARN";
    case Level::Error:
        return "ERROR";
    }
    return "DEBUG";
}

std::string Logger::buildMessage(Level level, const std::string& message)
{
    const auto now = std::chrono::system_clock::now();
    const auto secs = std::chrono::system_clock::to_time_t(now);
    const auto micros =
        std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() % 1000000;
    std::tm localTime = {};
    localtime_s(&localTime, &secs);
    std::ostringstream oss;
    oss << '[' << levelName(level) << ']';
    oss << '[' << std::put_time(&localTime, "%F %T") << '.' << std::setw(6) << std::setfill('0') << micros << "] ";
    oss << message;
    return oss.str();
}

} // namespace radar
