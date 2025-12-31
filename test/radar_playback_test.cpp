#include "processing/RadarPlayback.hpp"

#include "test_helpers.hpp"

#include <gtest/gtest.h>

namespace fs = std::filesystem;

TEST(RadarPlaybackTest, InitializeFailsWithoutConfig)
{
    const fs::path tempDir = test_helpers::makeTempDir("radar_playback_missing");
    const fs::path dataDir = tempDir / "data";
    const fs::path dataFile = dataDir / "corner.txt";
    test_helpers::writeFile(dataFile, "invalid");

    radar::RadarPlayback::Settings settings;
    settings.dataRoot = dataDir;
    settings.inputFiles = {dataFile.filename().string()};
    settings.vehicleConfigPath = dataDir / "missing.ini";

    radar::RadarPlayback playback(settings);
    EXPECT_FALSE(playback.initialize());
}

TEST(RadarPlaybackTest, ReadsDetectionsAndTracks)
{
    const fs::path tempDir = test_helpers::makeTempDir("radar_playback");
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
    ASSERT_TRUE(playback.initialize());

    radar::RadarFrame frame;
    ASSERT_TRUE(playback.readNextFrame(frame));
    EXPECT_TRUE(frame.hasDetections);
    EXPECT_TRUE(frame.hasTracks);
    EXPECT_GE(frame.detections.size(), 3U);
    EXPECT_EQ(frame.tracks.size(), 1U);
    EXPECT_GE(frame.sources.size(), 3U);
    EXPECT_FALSE(playback.vehicleContour().empty());

    EXPECT_FALSE(playback.readNextFrame(frame));
}
