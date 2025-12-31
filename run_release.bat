@ECHO ON

set BASEDIR=%~dp0
PUSHD %BASEDIR%

RMDIR /Q /S build

conan install . -c tools.system.package_manager:mode=install -c tools.system.package_manager:sudo=True --output-folder=build --build=missing --settings=build_type=Release
cmake -S . -B build -G "Visual Studio 17 2022" -DCMAKE_TOOLCHAIN_FILE=build\\build\\generators\\conan_toolchain.cmake -DCMAKE_POLICY_DEFAULT_CMP0091=NEW
cmake --build build --config Release
mkdir build\Release\shaders 1>NUL 2>NUL
robocopy shaders build\Release\shaders /E /z
robocopy visualization build\Release imgui.ini /z
mkdir build\Release\data 1>NUL 2>NUL
robocopy data build\Release\data /E /z
PUSHD build\Release
radarprocessor.exe fourCornersfusedRadarDetections.txt fusedFrontRadarsDetections.txt fusedRadarTracks.txt
POPD

REM conan create . -s build_type=Release

popd
