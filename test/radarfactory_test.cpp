#include "sensors/RadarFactoryHelpers.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>

#include <gtest/gtest.h>

namespace fs = std::filesystem;
using highres_clock = std::chrono::high_resolution_clock;

namespace
{
class StubTextRadarSensor final : public radar::BaseRadarSensor
{
public:
    explicit StubTextRadarSensor(fs::path path)
        : m_identifier(path.filename().string())
    {
    }

    const std::string& identifier() const noexcept override
    {
        return m_identifier;
    }

    void configure(float) override
    {
    }

    bool readNextScan(PointCloud&, uint64_t&) override
    {
        return false;
    }

private:
    std::string m_identifier;
};

std::unique_ptr<radar::BaseRadarSensor> createStubSensor(fs::path path)
{
    return std::make_unique<StubTextRadarSensor>(std::move(path));
}
} // namespace

namespace
{
class ScopedWorkingDirectory
{
public:
    explicit ScopedWorkingDirectory(const fs::path& target)
        : m_previous(fs::current_path())
    {
        fs::current_path(target);
    }

    ~ScopedWorkingDirectory()
    {
        try
        {
            fs::current_path(m_previous);
        }
        catch (const std::filesystem::filesystem_error&)
        {
            // Best effort restore; ignore failures during test cleanup.
        }
    }

private:
    fs::path m_previous;
};
} // namespace

class RadarFactoryTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        const auto stamp = highres_clock::now().time_since_epoch().count();
        baseDir = fs::temp_directory_path() / ("radarfactory_test_" + std::to_string(stamp));
        dataDir = baseDir / "data";
        ASSERT_TRUE(fs::create_directories(dataDir));

        sampleFile = dataDir / "radar_sample.txt";
        std::ofstream file(sampleFile);
        ASSERT_TRUE(file);
        file << "test data";

        radar::factory::setTextRadarSensorFactory(createStubSensor);
    }

    void TearDown() override
    {
        std::error_code ec;
        fs::remove_all(baseDir, ec);

        radar::factory::setTextRadarSensorFactory(nullptr);
    }

    fs::path baseDir;
    fs::path dataDir;
    fs::path sampleFile;
};

TEST_F(RadarFactoryTest, ResolvesDataFileThroughCandidates)
{
    const auto candidates = radar::factory::radarDataCandidatePaths(sampleFile.filename().string(), baseDir);
    ASSERT_EQ(candidates.size(), 1U);
    EXPECT_TRUE(fs::exists(candidates.front()));
    EXPECT_TRUE(fs::equivalent(candidates.front(), sampleFile));

    const auto resolved = radar::factory::resolveRadarDataFile(sampleFile.filename().string(), baseDir);
    ASSERT_TRUE(resolved);
    EXPECT_TRUE(fs::equivalent(*resolved, sampleFile));
}

TEST_F(RadarFactoryTest, CreatesTextRadarSensorWhenFileExists)
{
    ScopedWorkingDirectory workingDir(baseDir);
    auto sensor = radar::factory::createTextRadarSensor(sampleFile.filename().string());
    ASSERT_TRUE(sensor);
    EXPECT_EQ(sensor->identifier(), sampleFile.filename().string());
}
