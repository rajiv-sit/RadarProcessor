#include "engine/RadarEngine.hpp"
#include "engine/RadarPlaybackEngine.hpp"
#include "processing/RadarPlayback.hpp"

#include "sensors/BaseRadarSensor.hpp"
#include "test_helpers.hpp"

#include <gtest/gtest.h>

namespace fs = std::filesystem;

namespace
{
class StubSensor final : public radar::BaseRadarSensor
{
public:
    const std::string& identifier() const noexcept override
    {
        return m_identifier;
    }

    void configure(float maxRangeMeters) override
    {
        configured = true;
        lastRange = maxRangeMeters;
    }

    bool readNextScan(PointCloud& destination, uint64_t& timestampUs) override
    {
        ++readCount;
        if (readCount > 1)
        {
            return false;
        }

        radar::RadarPoint point{};
        point.x = 1.0f;
        point.y = 2.0f;
        destination = {point};
        timestampUs = 100U;
        return true;
    }

    std::string m_identifier = "stub-sensor";
    bool configured = false;
    float lastRange = 0.0f;
    int readCount = 0;
};
} // namespace

TEST(RadarEngineTest, InitializeFailsWithoutSensor)
{
    radar::RadarEngine engine(nullptr);
    EXPECT_FALSE(engine.initialize());
}

TEST(RadarEngineTest, RunsSingleFrameWithStubSensor)
{
    auto sensor = std::make_unique<StubSensor>();
    auto* sensorPtr = sensor.get();
    radar::RadarEngine engine(std::move(sensor));
    engine.run();
    EXPECT_TRUE(sensorPtr->configured);
    EXPECT_GT(sensorPtr->readCount, 0);
}

TEST(RadarPlaybackEngineTest, RunsSingleFrame)
{
    const fs::path tempDir = test_helpers::makeTempDir("radar_playback_engine");
    const fs::path dataDir = tempDir / "data";
    const fs::path vehicleFile = dataDir / "Vehicle.ini";
    const fs::path cornerFile = dataDir / "corner.txt";
    const fs::path frontFile = dataDir / "front.txt";
    const fs::path trackFile = dataDir / "tracks.txt";

    test_helpers::writeFile(vehicleFile, test_helpers::buildVehicleConfigIni(1.2f, true, false));
    test_helpers::writeFile(cornerFile, test_helpers::buildCornerDetectionsLine(100U, 90U, 0));
    test_helpers::writeFile(frontFile, test_helpers::buildFrontDetectionsLine(100U, 90U));
    test_helpers::writeFile(trackFile, test_helpers::buildTrackLine(100U));

    radar::RadarPlayback::Settings settings;
    settings.dataRoot = dataDir;
    settings.inputFiles = {cornerFile.filename().string(),
                           frontFile.filename().string(),
                           trackFile.filename().string()};

    radar::RadarPlayback playback(settings);
    radar::RadarPlaybackEngine engine(std::move(playback));
    engine.run();
}
