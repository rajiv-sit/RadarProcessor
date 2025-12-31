#pragma once

#include <filesystem>
#include <mutex>
#include <string>

namespace radar
{

class Logger
{
public:
    enum class Level
    {
        Info,
        Warning,
        Error,
    };

    static void initialize(const std::filesystem::path& logPath);
    static void log(Level level, const std::string& message);

private:
    static const char* levelName(Level level);
    static std::string buildMessage(Level level, const std::string& message);

    static std::mutex s_mutex;
};

} // namespace radar
