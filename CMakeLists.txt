cmake_minimum_required(VERSION 3.30.1)
project(radarprocessor LANGUAGES C CXX VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(BUILD_RESOURCES_DIR ${CMAKE_CURRENT_BINARY_DIR}/resources)
file(MAKE_DIRECTORY ${BUILD_RESOURCES_DIR})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/shaders DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

find_package(Eigen3 REQUIRED)
find_package(glm REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glew REQUIRED)
find_package(imgui REQUIRED)
find_package(opengl REQUIRED)
find_package(GTest CONFIG REQUIRED)

set(RADAR_SOURCES
    test/main.cpp
    radar/src/engine/RadarEngine.cpp
    radar/src/engine/RadarPlaybackEngine.cpp
    radar/src/sensors/RadarFactory.cpp
    radar/src/sensors/TextRadarSensor.cpp
    radar/src/config/VehicleProfile.cpp
    radar/src/sensors/MultiRadarSensor.cpp
    radar/src/sensors/OfflineRadarDataReader.cpp
    radar/src/sensors/OfflineRadarSensor.cpp
    radar/src/mapping/FusedRadarMapping.cpp
    radar/src/mapping/RadarVirtualSensorMapping.cpp
    radar/src/logging/Logger.cpp
    radar/src/processing/RadarPlayback.cpp
    visualization/RadarVisualizer.cpp
    visualization/Shader.cpp
    bindings/imgui_impl_glfw.cpp
    bindings/imgui_impl_opengl3.cpp
)

file(GLOB SPLINTER_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/splinter/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/splinter/src/cinterface/*.cpp"
)

list(APPEND RADAR_SOURCES ${SPLINTER_SOURCES})
list(APPEND RADAR_SOURCES radar/src/sensors/RadarFactoryHelpers.cpp)
list(APPEND RADAR_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/assets/implot/implot.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/assets/implot/implot_items.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/assets/inireader/IniFileParser.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/assets/inireader/ini.c
    ${CMAKE_CURRENT_SOURCE_DIR}/radar_core/odometry_estimator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/radar_core/processing_pipeline.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/utility/vehicle_config.cpp
)

add_executable(radarprocessor ${RADAR_SOURCES})

target_include_directories(radarprocessor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/radar/include
    ${CMAKE_CURRENT_SOURCE_DIR}/visualization
    ${CMAKE_CURRENT_SOURCE_DIR}/shaders
    ${CMAKE_CURRENT_SOURCE_DIR}/bindings
    ${CMAKE_CURRENT_SOURCE_DIR}/splinter/include
    ${CMAKE_CURRENT_SOURCE_DIR}/assets/implot
    ${CMAKE_CURRENT_SOURCE_DIR}/assets/inireader
    ${CMAKE_CURRENT_SOURCE_DIR}/radar_core
)

target_link_libraries(radarprocessor PRIVATE
    Eigen3::Eigen
    glfw
    GLEW::GLEW
    glm::glm
    imgui::imgui
    opengl::opengl
)

if(WIN32)
    target_compile_definitions(radarprocessor PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

add_custom_command(TARGET radarprocessor POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/data $<TARGET_FILE_DIR:radarprocessor>/data
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/shaders $<TARGET_FILE_DIR:radarprocessor>/shaders
)

add_executable(radarfactory_test
    test/radarfactory_test.cpp
    radar/src/sensors/RadarFactoryHelpers.cpp
    radar/src/logging/Logger.cpp
    radar/src/sensors/TextRadarSensor.cpp
    radar/src/config/VehicleProfile.cpp
)

target_include_directories(radarfactory_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/radar/include
)

target_compile_features(radarfactory_test PRIVATE cxx_std_20)
target_link_libraries(radarfactory_test PRIVATE
    GTest::gtest_main
    glm::glm
)

enable_testing()
include(GoogleTest)

add_executable(radar_unit_tests
    test/utility_math_utils_test.cpp
    test/utility_vehicle_config_test.cpp
    test/radar_core_odometry_test.cpp
    test/radar_core_pipeline_test.cpp
    test/radar_mapping_test.cpp
    test/radar_vehicle_profile_test.cpp
    test/radar_sensor_test.cpp
    test/radar_playback_test.cpp
    test/radar_engine_test.cpp
    test/radar_visualizer_stub.cpp
    radar/src/sensors/RadarFactory.cpp
    radar/src/sensors/RadarFactoryHelpers.cpp
    radar/src/sensors/TextRadarSensor.cpp
    radar/src/sensors/OfflineRadarDataReader.cpp
    radar/src/sensors/OfflineRadarSensor.cpp
    radar/src/sensors/MultiRadarSensor.cpp
    radar/src/processing/RadarPlayback.cpp
    radar/src/mapping/FusedRadarMapping.cpp
    radar/src/mapping/RadarVirtualSensorMapping.cpp
    radar/src/logging/Logger.cpp
    radar/src/config/VehicleProfile.cpp
    radar/src/engine/RadarEngine.cpp
    radar/src/engine/RadarPlaybackEngine.cpp
    radar_core/processing_pipeline.cpp
    radar_core/odometry_estimator.cpp
    utility/vehicle_config.cpp
    assets/inireader/IniFileParser.cpp
    assets/inireader/ini.c
    visualization/Shader.cpp
)

target_include_directories(radar_unit_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/radar/include
    ${CMAKE_CURRENT_SOURCE_DIR}/radar_core
    ${CMAKE_CURRENT_SOURCE_DIR}/utility
    ${CMAKE_CURRENT_SOURCE_DIR}/assets/inireader
    ${CMAKE_CURRENT_SOURCE_DIR}/test
)

target_link_libraries(radar_unit_tests PRIVATE
    GTest::gtest_main
    Eigen3::Eigen
    glfw
    GLEW::GLEW
    glm::glm
    opengl::opengl
)

gtest_discover_tests(radarfactory_test)
gtest_discover_tests(radar_unit_tests)
