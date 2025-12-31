#pragma once

#include "visualization/Shader.hpp"

#include "processing/RadarTrack.hpp"
#include "sensors/BaseRadarSensor.hpp"

#include <GL/glew.h>
#include <glm/glm.hpp>

#include <array>
#include <cstdint>
#include <deque>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

struct GLFWwindow;

namespace visualization
{

class RadarVisualizer
{
public:
    RadarVisualizer() = default;
    ~RadarVisualizer();

    bool initialize();
    void updatePoints(const radar::BaseRadarSensor::PointCloud& points,
                      uint64_t timestampUs,
                      const std::vector<std::string>& sources);
    void updateFrameInfo(uint64_t timestampUs, const std::vector<std::string>& sources);
    void updateTracks(const std::vector<radar::RadarTrack>& tracks);
    void updateMapPoints(const std::vector<glm::vec3>& points);
    void updateMapSegments(const std::vector<glm::vec3>& points);
    void updateVehicleContour(const std::vector<glm::vec2>& contourPoints);
    void setVcsToIsoTransform(float distRearAxle);
    void setResetMapCallback(std::function<void()> callback);
    void render();
    bool windowShouldClose() const;
    float frameSpeedScale() const;
    std::size_t mapSegmentCount() const;

private:
    struct Vertex
    {
        glm::vec3 position;
        float intensity;
    };

    struct DetectionFrame
    {
        std::vector<radar::RadarPoint> points;
        uint64_t timestampUs = 0;
    };

    struct FovDescriptor
    {
        float horizontalFovRad = 0.0F;
        float maximumRange = 0.0F;
        float boresightAngleRad = 0.0F;
        float azimuthPolarity = 1.0F;
        float sensorLongitudinal = 0.0F;
        float sensorLateral = 0.0F;
    };

    enum class DetectionColorMode
    {
        MotionState = 0,
        RadarUnit,
        DetectionType
    };

    enum class DetectionAlphaMode
    {
        Constant = 0,
        MotionProbability,
        TimeDecay
    };

    enum class DetectionMotionFilter
    {
        Static = 0,
        Moving,
        All
    };

    enum class DetectionType
    {
        Valid = 0,
        SuperRes,
        NdTarget,
        HostVehicleClutter,
        MultiBounce,
        Unknown
    };

    enum class CameraMode
    {
        FreeOrbit = 0,
        BirdsEye,
        Front,
        Side,
        Rear
    };

    struct Camera
    {
        float distance = 10.0F;
        float yaw = 90.0F;
        float pitch = -25.0F;
        float fov = 45.0F;
        bool rotating = false;
        double lastX = 0.0;
        double lastY = 0.0;
    };

    void cleanup();
    void uploadBuffer();
    void uploadMapBuffer();
    void uploadMapSegmentBuffer();
    void uploadMapSplineBuffer();
    void uploadContourBuffer();
    void uploadGridBuffer();
    void buildGridVertices();
    void setupVertexAttributes(GLuint vao, GLuint vbo);
    void updateFrameTiming(uint64_t timestampUs);
    glm::mat4 computeViewProjection() const;
    glm::vec3 computeCameraDirection() const;
    glm::vec3 computeCameraUp() const;
    void processCursorPos(double xpos, double ypos);
    void processScroll(double yoffset);
    void processMouseButton(int button, int action);
    static void cursorPosCallback(GLFWwindow* window, double xpos, double ypos);
    static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);
    static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
    void drawUI();
    void drawDetections(const glm::mat4& viewProjection);
    void drawFovPolygons(const glm::mat4& viewProjection);
    void drawTracks(const glm::mat4& viewProjection);
    std::vector<glm::vec2> buildMapSplineBoundary(const std::vector<glm::vec2>& basePoints) const;
    std::vector<double> sampleBspline(const std::vector<double>& parameters,
                                      const std::vector<double>& values,
                                      std::size_t resolution) const;
    bool passesMotionFilter(const radar::RadarPoint& point) const;
    DetectionType detectionTypeForPoint(const radar::RadarPoint& point) const;
    glm::vec3 colorForSensor(int sensorIndex) const;
    glm::vec3 colorForDetectionType(DetectionType type) const;
    float computeDetectionAlpha(const radar::RadarPoint& point, float ageSeconds) const;
    glm::vec3 trackColor(const radar::RadarTrack& track) const;

    GLFWwindow* m_window = nullptr;
    GLuint m_vao = 0;
    GLuint m_vbo = 0;
    GLuint m_mapVao = 0;
    GLuint m_mapVbo = 0;
    GLuint m_mapSegmentVao = 0;
    GLuint m_mapSegmentVbo = 0;
    GLuint m_mapSplineVao = 0;
    GLuint m_mapSplineVbo = 0;
    GLuint m_contourVao = 0;
    GLuint m_contourVbo = 0;
    GLuint m_gridVao = 0;
    GLuint m_gridVbo = 0;
    GLuint m_fovVao = 0;
    GLuint m_fovVbo = 0;
    GLuint m_trackVao = 0;
    GLuint m_trackVbo = 0;
    Shader m_shader;
    std::vector<Vertex> m_vertices;
    std::vector<Vertex> m_mapVertices;
    std::vector<Vertex> m_mapSegmentVertices;
    std::vector<Vertex> m_mapSplineVertices;
    std::vector<Vertex> m_contourVertices;
    std::vector<Vertex> m_gridVertices;
    std::vector<radar::RadarTrack> m_tracks;
    bool m_bufferDirty = false;
    bool m_mapDirty = false;
    bool m_mapSegmentDirty = false;
    bool m_mapSplineDirty = false;
    bool m_contourDirty = false;
    bool m_gridDirty = true;
    float m_pointSize = 4.0F;
    float m_intensityScale = 1.0F;
    float m_replaySpeed = 1.0F;
    float m_mapIntensity = 1.5F;
    bool m_vcsToIsoEnabled = false;
    float m_vcsToIsoLongitudinalOffset = 0.0F;
    bool m_showMapping = true;
    bool m_showBsplineMap = false;
    bool m_showVehicleContour = true;
    int m_mapSegmentCount = 72;
    int m_mapSplineControlPointCount = 180;
    float m_contourLineWidth = 2.0F;
    bool m_showGrid = true;
    float m_gridSpacing = 10.0F;
    float m_gridHalfSpan = 60.0F;
    glm::vec2 m_gridMin = glm::vec2(-60.0F, -60.0F);
    glm::vec2 m_gridMax = glm::vec2(60.0F, 60.0F);
    glm::vec3 m_gridColor = glm::vec3(0.6F, 0.6F, 0.6F);
    float m_gridAlpha = 0.6F;
    bool m_showFov = true;
    float m_fovAlpha = 0.25F;
    std::unordered_map<int, FovDescriptor> m_fovBySensor;
    std::unordered_map<int, float> m_fovRangeOverride;
    std::function<void()> m_resetMapCallback;
    bool m_showDetections = true;
    bool m_displayElevation = true;
    bool m_enablePersistentDetections = false;
    int m_detectionScanRetention = 10;
    DetectionColorMode m_detectionColorMode = DetectionColorMode::MotionState;
    DetectionAlphaMode m_detectionAlphaMode = DetectionAlphaMode::Constant;
    DetectionMotionFilter m_detectionMotionFilter = DetectionMotionFilter::All;
    bool m_displayValid = true;
    bool m_displaySuperRes = true;
    bool m_displayNdTarget = true;
    bool m_displayHostVehicleClutter = true;
    bool m_displayMultiBounce = true;
    glm::vec3 m_staticColor = glm::vec3(0.25F, 0.85F, 0.45F);
    glm::vec3 m_movingColor = glm::vec3(0.2F, 0.8F, 1.0F);
    glm::vec3 m_ambiguousColor = glm::vec3(0.85F, 0.75F, 0.2F);
    std::array<glm::vec3, 5> m_detectionTypeColors = {
        glm::vec3(0.85F, 0.85F, 0.85F),
        glm::vec3(0.7F, 0.4F, 1.0F),
        glm::vec3(0.25F, 0.9F, 0.4F),
        glm::vec3(0.95F, 0.65F, 0.2F),
        glm::vec3(1.0F, 0.25F, 0.25F)};
    float m_detectionAlphaConstant = 0.6F;
    float m_detectionAlphaDecay = 0.35F;
    float m_rangeRateStationaryScale = 5.0F;
    std::deque<DetectionFrame> m_detectionHistory;
    std::vector<radar::RadarPoint> m_currentPoints;
    bool m_showTracks = true;
    float m_trackLineWidth = 1.5F;
    float m_trackAlpha = 0.85F;
    float m_trackSpeedThreshold = 1.0F;
    glm::vec3 m_trackMovingColor = glm::vec3(0.2F, 0.85F, 0.3F);
    glm::vec3 m_trackStationaryColor = glm::vec3(0.9F, 0.3F, 0.3F);
    glm::vec3 m_trackUnknownColor = glm::vec3(0.85F, 0.75F, 0.2F);
    float m_lastFramePeriodSec = 0.1F;
    bool m_hasPreviousFrame = false;
    uint64_t m_previousTimestampUs = 0;
    glm::vec3 m_cameraTarget = glm::vec3(0.0F);
    Camera m_camera;
    CameraMode m_cameraMode = CameraMode::FreeOrbit;
    int m_activeMouseButton = -1;
    uint64_t m_lastTimestampUs = 0;
    std::vector<std::string> m_lastSources;
};

} // namespace visualization
