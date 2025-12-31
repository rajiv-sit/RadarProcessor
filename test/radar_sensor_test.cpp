#include "sensors/MultiRadarSensor.hpp"
#include "sensors/OfflineRadarDataReader.hpp"
#include "sensors/OfflineRadarSensor.hpp"
#include "sensors/RadarFactory.hpp"
#include "sensors/RadarFactoryHelpers.hpp"
#include "sensors/TextRadarSensor.hpp"

#include "test_helpers.hpp"

#include <gtest/gtest.h>

namespace fs = std::filesystem;

namespace
{
class StubSensor final : public radar::BaseRadarSensor
{
public:
    StubSensor(std::string id, radar::BaseRadarSensor::PointCloud points, uint64_t timestamp)
        : m_identifier(std::move(id))
        , m_points(std::move(points))
        , m_timestamp(timestamp)
    {
    }

    const std::string& identifier() const noexcept override
    {
        return m_identifier;
    }

    void configure(float maxRangeMeters) override
    {
        m_lastConfig = maxRangeMeters;
    }

    bool readNextScan(PointCloud& destination, uint64_t& timestampUs) override
    {
        if (m_consumed)
        {
            return false;
        }
        destination = m_points;
        timestampUs = m_timestamp;
        m_consumed = true;
        return true;
    }

    float lastConfig() const
    {
        return m_lastConfig;
    }

private:
    std::string m_identifier;
    PointCloud m_points;
    uint64_t m_timestamp = 0U;
    float m_lastConfig = 0.0f;
    bool m_consumed = false;
};

std::unique_ptr<radar::BaseRadarSensor> createStubSensor(fs::path path)
{
    radar::BaseRadarSensor::PointCloud points;
    radar::RadarPoint point{};
    point.x = 1.0f;
    point.y = 2.0f;
    points.push_back(point);
    return std::make_unique<StubSensor>(path.filename().string(), points, 1234U);
}

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
        std::error_code ec;
        fs::current_path(m_previous, ec);
    }

private:
    fs::path m_previous;
};
} // namespace

TEST(TextRadarSensorTest, ParsesRadarReturnLine)
{
    const fs::path tempDir = test_helpers::makeTempDir("text_radar");
    const fs::path dataFile = tempDir / "sample.txt";
    const fs::path profileFile = tempDir / "VehicleProfile.ini";
    test_helpers::writeFile(profileFile, test_helpers::buildVehicleProfileIni(1.0f));
    test_helpers::writeFile(dataFile, test_helpers::buildCornerDetectionsLine(100U, 90U, 0));

    radar::TextRadarSensor sensor(dataFile);
    sensor.configure(120.0f);

    radar::BaseRadarSensor::PointCloud points;
    uint64_t timestamp = 0U;
    ASSERT_TRUE(sensor.readNextScan(points, timestamp));
    EXPECT_EQ(timestamp, 100U);
    EXPECT_FALSE(points.empty());
    EXPECT_NE(sensor.vehicleProfile(), nullptr);
}

TEST(TextRadarSensorTest, ParsesLegacyLine)
{
    const fs::path tempDir = test_helpers::makeTempDir("text_radar_legacy");
    const fs::path dataFile = tempDir / "legacy.txt";
    std::ostringstream oss;
    oss << "0 42 0 0 0 0 0 0 0 1.0 2.0 0.5 3.0 4.0 0.7";
    test_helpers::writeFile(dataFile, oss.str());

    radar::TextRadarSensor sensor(dataFile);
    radar::BaseRadarSensor::PointCloud points;
    uint64_t timestamp = 0U;
    ASSERT_TRUE(sensor.readNextScan(points, timestamp));
    EXPECT_EQ(timestamp, 42U);
    EXPECT_EQ(points.size(), 2U);
}

TEST(OfflineRadarDataReaderTest, ReadsCombinedScan)
{
    const fs::path tempDir = test_helpers::makeTempDir("offline_reader");
    const fs::path dataDir = tempDir / "data";
    const fs::path fileA = dataDir / "a.txt";
    const fs::path fileB = dataDir / "b.txt";
    test_helpers::writeFile(fileA, test_helpers::buildCornerDetectionsLine(100U, 90U, 0));
    test_helpers::writeFile(fileB, test_helpers::buildCornerDetectionsLine(100U, 90U, 1));

    radar::OfflineRadarDataReader reader(dataDir, {"a.txt", "b.txt"});
    ASSERT_TRUE(reader.configure(120.0f));

    radar::BaseRadarSensor::PointCloud points;
    uint64_t timestamp = 0U;
    ASSERT_TRUE(reader.readNextScan(points, timestamp));
    EXPECT_EQ(timestamp, 100U);
    EXPECT_EQ(reader.lastFrameSources().size(), 2U);
    EXPECT_FALSE(points.empty());
}

TEST(OfflineRadarSensorTest, ReadsDefaultFiles)
{
    const fs::path tempDir = test_helpers::makeTempDir("offline_sensor");
    const fs::path dataDir = tempDir / "data";
    const fs::path cornerFile = dataDir / "fourCornersfusedRadarDetections.txt";
    const fs::path frontFile = dataDir / "fusedFrontRadarsDetections.txt";
    test_helpers::writeFile(cornerFile, test_helpers::buildCornerDetectionsLine(120U, 110U, 0));
    test_helpers::writeFile(frontFile, test_helpers::buildFrontDetectionsLine(120U, 110U));

    radar::OfflineRadarSensor sensor(dataDir);
    sensor.configure(120.0f);

    radar::BaseRadarSensor::PointCloud points;
    uint64_t timestamp = 0U;
    ASSERT_TRUE(sensor.readNextScan(points, timestamp));
    EXPECT_EQ(timestamp, 120U);
    EXPECT_FALSE(points.empty());
    EXPECT_FALSE(sensor.lastFrameSources().empty());
}

TEST(MultiRadarSensorTest, AggregatesSensors)
{
    radar::BaseRadarSensor::PointCloud pointsA(1);
    radar::BaseRadarSensor::PointCloud pointsB(2);
    auto sensorA = std::make_unique<StubSensor>("alpha", pointsA, 100U);
    auto sensorB = std::make_unique<StubSensor>("bravo", pointsB, 200U);
    std::vector<std::unique_ptr<radar::BaseRadarSensor>> sensors;
    sensors.push_back(std::move(sensorA));
    sensors.push_back(std::move(sensorB));

    radar::MultiRadarSensor multi(std::move(sensors));
    EXPECT_EQ(multi.identifier(), "alpha+bravo");
    multi.configure(50.0f);

    radar::BaseRadarSensor::PointCloud combined;
    uint64_t timestamp = 0U;
    ASSERT_TRUE(multi.readNextScan(combined, timestamp));
    EXPECT_EQ(combined.size(), 3U);
    EXPECT_EQ(timestamp, 200U);
}

TEST(RadarFactoryTest, CreatesMultiSensorWhenMultipleFilesProvided)
{
    const fs::path tempDir = test_helpers::makeTempDir("radar_factory");
    const fs::path dataDir = tempDir / "data";
    const fs::path fileA = dataDir / "a.txt";
    const fs::path fileB = dataDir / "b.txt";
    test_helpers::writeFile(fileA, "data");
    test_helpers::writeFile(fileB, "data");

    radar::factory::setTextRadarSensorFactory(createStubSensor);
    ScopedWorkingDirectory cwd(tempDir);

    auto sensor = radar::RadarFactory::createSensor({fileA.filename().string(), fileB.filename().string()});
    ASSERT_TRUE(sensor);
    EXPECT_EQ(sensor->identifier(), "a.txt+b.txt");

    radar::factory::setTextRadarSensorFactory(nullptr);
}

TEST(RadarFactoryTest, ReturnsNullWhenNoFilesResolve)
{
    radar::factory::setTextRadarSensorFactory(createStubSensor);
    auto sensor = radar::RadarFactory::createSensor({"does_not_exist.txt"});
    EXPECT_EQ(sensor, nullptr);
    radar::factory::setTextRadarSensorFactory(nullptr);
}
