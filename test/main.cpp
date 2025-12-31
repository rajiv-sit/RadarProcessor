#include "radar/include/engine/RadarPlaybackEngine.hpp"
#include "radar/include/processing/RadarPlayback.hpp"

#include <filesystem>
#include <iostream>
#include <vector>

int main(int argc, char** argv)
{
    std::vector<std::string> radarFiles;
    if (argc > 1)
    {
        radarFiles.assign(argv + 1, argv + argc);
    }
    else
    {
        radarFiles = {
            "fourCornersfusedRadarDetections.txt",
            "fusedFrontRadarsDetections.txt",
            "fusedRadarTracks.txt",
        };
    }

    radar::RadarPlayback::Settings settings;
    settings.inputFiles = radarFiles;
    settings.dataRoot = std::filesystem::current_path() / "data";
    radar::RadarPlayback playback(std::move(settings));
    radar::RadarPlaybackEngine engine(std::move(playback));
    engine.run();
    return EXIT_SUCCESS;
}
