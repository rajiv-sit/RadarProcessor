#include "visualization/RadarVisualizer.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <bsplinebuilder.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <imgui.h>
#include <imgui_impl_glfw.hpp>
#include <imgui_impl_opengl3.hpp>
#include <array>
#include <filesystem>
#include <cmath>
#include <cstddef>
#include <algorithm>
#include <iostream>
#include <limits>
#include <string>
#include <utility>

namespace visualization
{

namespace
{
constexpr const char* kVertexShaderPath = "shaders/point.vs";
constexpr const char* kFragmentShaderPath = "shaders/point.fs";
constexpr std::size_t kMapSplineSampleCount = 192;
constexpr int kMapSegmentMin = 12;
constexpr int kMapSegmentMax = 360;
constexpr int kMapSplineControlPointMin = 12;
constexpr int kMapSplineControlPointMax = 360;
constexpr std::array<const char*, 5> kCameraModeLabels = {
    "Free orbit",
    "Bird's eye",
    "Front",
    "Side",
    "Rear"};
constexpr std::array<const char*, 3> kDetectionColorModeLabels = {
    "Motion state",
    "Radar unit",
    "Detection type"};
constexpr std::array<const char*, 3> kDetectionAlphaModeLabels = {
    "Constant",
    "Probability",
    "Time decay"};
constexpr std::array<const char*, 3> kDetectionMotionFilterLabels = {
    "Static",
    "Moving",
    "All"};
constexpr int kFovArcPointCount = 24;
constexpr float kSplineControlPointEpsilon = 1e-4F;

std::vector<glm::vec2> resampleLoop(const std::vector<glm::vec2>& points, std::size_t targetCount)
{
    if (points.size() < 2U || targetCount == 0U)
    {
        return points;
    }

    std::vector<glm::vec2> filtered;
    filtered.reserve(points.size());
    for (const auto& point : points)
    {
        if (filtered.empty() || glm::length(point - filtered.back()) > kSplineControlPointEpsilon)
        {
            filtered.push_back(point);
        }
    }

    if (filtered.size() < 2U)
    {
        return filtered;
    }

    if (glm::length(filtered.front() - filtered.back()) <= kSplineControlPointEpsilon)
    {
        filtered.pop_back();
    }
    if (filtered.size() < 2U)
    {
        return filtered;
    }

    struct Segment
    {
        glm::vec2 start;
        glm::vec2 end;
        float length = 0.0F;
    };

    std::vector<Segment> segments;
    segments.reserve(filtered.size());
    float totalLength = 0.0F;
    for (std::size_t i = 0; i < filtered.size(); ++i)
    {
        const glm::vec2& a = filtered[i];
        const glm::vec2& b = filtered[(i + 1) % filtered.size()];
        const float length = glm::length(b - a);
        if (length <= kSplineControlPointEpsilon)
        {
            continue;
        }
        segments.push_back({a, b, length});
        totalLength += length;
    }

    if (segments.empty() || totalLength <= kSplineControlPointEpsilon)
    {
        return filtered;
    }

    std::vector<glm::vec2> resampled;
    resampled.reserve(targetCount);
    const float step = totalLength / static_cast<float>(targetCount);
    float segmentStart = 0.0F;
    std::size_t segmentIndex = 0;

    for (std::size_t i = 0; i < targetCount; ++i)
    {
        const float target = step * static_cast<float>(i);
        while (segmentIndex + 1 < segments.size() &&
               target > segmentStart + segments[segmentIndex].length)
        {
            segmentStart += segments[segmentIndex].length;
            ++segmentIndex;
        }

        const Segment& segment = segments[segmentIndex];
        const float localT = segment.length > kSplineControlPointEpsilon
                                 ? (target - segmentStart) / segment.length
                                 : 0.0F;
        const float clampedT = std::clamp(localT, 0.0F, 1.0F);
        resampled.push_back(segment.start + clampedT * (segment.end - segment.start));
    }

    return resampled;
}
}

RadarVisualizer::~RadarVisualizer()
{
    cleanup();
}

bool RadarVisualizer::initialize()
{
    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW\n";
        return false;
    }

    m_window = glfwCreateWindow(1280, 720, "RadarProcessor", nullptr, nullptr);
    if (!m_window)
    {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(m_window);
    glfwSwapInterval(1);
    glfwSetWindowUserPointer(m_window, this);
    glfwSetCursorPosCallback(m_window, RadarVisualizer::cursorPosCallback);
    glfwSetScrollCallback(m_window, RadarVisualizer::scrollCallback);
    glfwSetMouseButtonCallback(m_window, RadarVisualizer::mouseButtonCallback);

    if (glewInit() != GLEW_OK)
    {
        std::cerr << "Failed to initialize GLEW\n";
        return false;
    }

    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_DEPTH_TEST);

    if (!m_shader.load(kVertexShaderPath, kFragmentShaderPath))
    {
        return false;
    }

    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    glGenVertexArrays(1, &m_mapVao);
    glGenBuffers(1, &m_mapVbo);
    glGenVertexArrays(1, &m_mapSegmentVao);
    glGenBuffers(1, &m_mapSegmentVbo);
    glGenVertexArrays(1, &m_mapSplineVao);
    glGenBuffers(1, &m_mapSplineVbo);
    glGenVertexArrays(1, &m_contourVao);
    glGenBuffers(1, &m_contourVbo);
    glGenVertexArrays(1, &m_gridVao);
    glGenBuffers(1, &m_gridVbo);
    glGenVertexArrays(1, &m_fovVao);
    glGenBuffers(1, &m_fovVbo);
    glGenVertexArrays(1, &m_trackVao);
    glGenBuffers(1, &m_trackVbo);

    setupVertexAttributes(m_vao, m_vbo);
    setupVertexAttributes(m_mapVao, m_mapVbo);
    setupVertexAttributes(m_mapSegmentVao, m_mapSegmentVbo);
    setupVertexAttributes(m_mapSplineVao, m_mapSplineVbo);
    setupVertexAttributes(m_contourVao, m_contourVbo);
    setupVertexAttributes(m_gridVao, m_gridVbo);
    setupVertexAttributes(m_fovVao, m_fovVbo);
    setupVertexAttributes(m_trackVao, m_trackVbo);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    static std::string iniPath;
    const std::filesystem::path visualizationIni = std::filesystem::path("visualization") / "imgui.ini";
    if (std::filesystem::exists(visualizationIni))
    {
        iniPath = visualizationIni.string();
    }
    else
    {
        iniPath = "imgui.ini";
    }
    io.IniFilename = iniPath.c_str();
    ImGui_ImplGlfw_InitForOpenGL(m_window, true);
    ImGui_ImplOpenGL3_Init("#version 330 core");

    return true;
}

void RadarVisualizer::cleanup()
{
    if (m_window)
    {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::SaveIniSettingsToDisk(ImGui::GetIO().IniFilename);
        ImGui::DestroyContext();
        glfwDestroyWindow(m_window);
        glfwTerminate();
        m_window = nullptr;
    }

    if (m_vbo != 0)
    {
        glDeleteBuffers(1, &m_vbo);
        m_vbo = 0;
    }

    if (m_mapVbo != 0)
    {
        glDeleteBuffers(1, &m_mapVbo);
        m_mapVbo = 0;
    }

    if (m_mapSegmentVbo != 0)
    {
        glDeleteBuffers(1, &m_mapSegmentVbo);
        m_mapSegmentVbo = 0;
    }

    if (m_mapSplineVbo != 0)
    {
        glDeleteBuffers(1, &m_mapSplineVbo);
        m_mapSplineVbo = 0;
    }

    if (m_contourVbo != 0)
    {
        glDeleteBuffers(1, &m_contourVbo);
        m_contourVbo = 0;
    }

    if (m_gridVbo != 0)
    {
        glDeleteBuffers(1, &m_gridVbo);
        m_gridVbo = 0;
    }

    if (m_fovVbo != 0)
    {
        glDeleteBuffers(1, &m_fovVbo);
        m_fovVbo = 0;
    }

    if (m_trackVbo != 0)
    {
        glDeleteBuffers(1, &m_trackVbo);
        m_trackVbo = 0;
    }

    if (m_vao != 0)
    {
        glDeleteVertexArrays(1, &m_vao);
        m_vao = 0;
    }

    if (m_mapVao != 0)
    {
        glDeleteVertexArrays(1, &m_mapVao);
        m_mapVao = 0;
    }

    if (m_mapSegmentVao != 0)
    {
        glDeleteVertexArrays(1, &m_mapSegmentVao);
        m_mapSegmentVao = 0;
    }

    if (m_mapSplineVao != 0)
    {
        glDeleteVertexArrays(1, &m_mapSplineVao);
        m_mapSplineVao = 0;
    }

    if (m_contourVao != 0)
    {
        glDeleteVertexArrays(1, &m_contourVao);
        m_contourVao = 0;
    }

    if (m_gridVao != 0)
    {
        glDeleteVertexArrays(1, &m_gridVao);
        m_gridVao = 0;
    }

    if (m_fovVao != 0)
    {
        glDeleteVertexArrays(1, &m_fovVao);
        m_fovVao = 0;
    }

    if (m_trackVao != 0)
    {
        glDeleteVertexArrays(1, &m_trackVao);
        m_trackVao = 0;
    }
}

void RadarVisualizer::updatePoints(const radar::BaseRadarSensor::PointCloud& points,
                                  uint64_t timestampUs,
                                  const std::vector<std::string>& sources)
{
    m_currentPoints.clear();
    m_currentPoints.reserve(points.size());
    m_lastSources = sources;
    updateFrameTiming(timestampUs);

    const glm::vec2 defaultMin(-m_gridHalfSpan, -m_gridHalfSpan);
    const glm::vec2 defaultMax(m_gridHalfSpan, m_gridHalfSpan);
    glm::vec2 nextMin = defaultMin;
    glm::vec2 nextMax = defaultMax;
    if (!points.empty())
    {
        float minX = std::numeric_limits<float>::max();
        float maxX = -std::numeric_limits<float>::max();
        float minY = std::numeric_limits<float>::max();
        float maxY = -std::numeric_limits<float>::max();
        for (const auto& point : points)
        {
            radar::RadarPoint adjusted = point;
            if (m_vcsToIsoEnabled)
            {
                adjusted.x = -adjusted.x;
                adjusted.y += m_vcsToIsoLongitudinalOffset;
            }
            m_currentPoints.push_back(adjusted);

            minX = std::min(minX, adjusted.x);
            maxX = std::max(maxX, adjusted.x);
            minY = std::min(minY, adjusted.y);
            maxY = std::max(maxY, adjusted.y);

            if (point.horizontalFov_rad > 0.0F && point.maximumRange_m > 0.0F)
            {
                FovDescriptor descriptor{};
                descriptor.horizontalFovRad = point.horizontalFov_rad;
                descriptor.maximumRange = point.maximumRange_m;
                descriptor.boresightAngleRad =
                    m_vcsToIsoEnabled ? -point.boresightAngle_rad : point.boresightAngle_rad;
                float azimuthPolarity = point.azimuthPolarity == 0.0F ? 1.0F : point.azimuthPolarity;
                if (m_vcsToIsoEnabled)
                {
                    azimuthPolarity = -azimuthPolarity;
                }
                descriptor.azimuthPolarity = azimuthPolarity;
                descriptor.sensorLongitudinal = point.sensorLongitudinal_m;
                descriptor.sensorLateral = point.sensorLateral_m;
                if (m_vcsToIsoEnabled)
                {
                    descriptor.sensorLongitudinal += m_vcsToIsoLongitudinalOffset;
                    descriptor.sensorLateral = -descriptor.sensorLateral;
                }
                m_fovBySensor[point.sensorIndex] = descriptor;
                if (m_fovRangeOverride.find(point.sensorIndex) == m_fovRangeOverride.end())
                {
                    m_fovRangeOverride[point.sensorIndex] = descriptor.maximumRange;
                }
            }
        }

        nextMin = glm::vec2(std::min(minX, defaultMin.x), std::min(minY, defaultMin.y));
        nextMax = glm::vec2(std::max(maxX, defaultMax.x), std::max(maxY, defaultMax.y));
    }
    if (nextMin != m_gridMin || nextMax != m_gridMax)
    {
        m_gridMin = nextMin;
        m_gridMax = nextMax;
        m_gridDirty = true;
    }

    if (!m_enablePersistentDetections)
    {
        m_detectionHistory.clear();
    }

    DetectionFrame frame;
    frame.points = m_currentPoints;
    frame.timestampUs = timestampUs;
    m_detectionHistory.push_back(std::move(frame));

    const int retention = std::max(1, m_detectionScanRetention);
    while (static_cast<int>(m_detectionHistory.size()) > retention)
    {
        m_detectionHistory.pop_front();
    }
}

void RadarVisualizer::updateFrameInfo(uint64_t timestampUs, const std::vector<std::string>& sources)
{
    m_lastSources = sources;
    updateFrameTiming(timestampUs);
}

void RadarVisualizer::updateTracks(const std::vector<radar::RadarTrack>& tracks)
{
    m_tracks.clear();
    m_tracks.reserve(tracks.size());
    for (const auto& track : tracks)
    {
        radar::RadarTrack adjusted = track;
        if (m_vcsToIsoEnabled)
        {
            adjusted.isoPosition.x += m_vcsToIsoLongitudinalOffset;
            adjusted.isoPosition.y = -adjusted.isoPosition.y;
            adjusted.isoVelocity.y = -adjusted.isoVelocity.y;
            adjusted.headingRad = -adjusted.headingRad;
            adjusted.headingRate = -adjusted.headingRate;
        }
        m_tracks.push_back(adjusted);
    }
}

void RadarVisualizer::updateFrameTiming(uint64_t timestampUs)
{
    m_lastTimestampUs = timestampUs;
    if (m_hasPreviousFrame && timestampUs > m_previousTimestampUs)
    {
        m_lastFramePeriodSec =
            static_cast<float>(timestampUs - m_previousTimestampUs) / 1'000'000.0F;
    }
    m_previousTimestampUs = timestampUs;
    m_hasPreviousFrame = true;
}

void RadarVisualizer::uploadBuffer()
{
    if (!m_bufferDirty || m_vertices.empty())
    {
        return;
    }

    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, m_vertices.size() * sizeof(Vertex), m_vertices.data(), GL_DYNAMIC_DRAW);
    m_bufferDirty = false;
}

void RadarVisualizer::updateMapPoints(const std::vector<glm::vec3>& points)
{
    m_mapVertices.clear();
    m_mapVertices.reserve(points.size());
    for (const auto& point : points)
    {
        glm::vec3 position = point;
        if (m_vcsToIsoEnabled)
        {
            position = glm::vec3(-position.x, position.y + m_vcsToIsoLongitudinalOffset, position.z);
        }
        m_mapVertices.push_back({position, 1.0F});
    }
    m_mapDirty = true;

    m_mapSplineVertices.clear();
    if (!m_showBsplineMap)
    {
        m_mapSplineDirty = true;
        return;
    }
    if (m_mapVertices.size() >= 3U)
    {
        std::vector<glm::vec2> basePoints;
        basePoints.reserve(m_mapVertices.size());
        for (const auto& vertex : m_mapVertices)
        {
            basePoints.emplace_back(vertex.position.x, vertex.position.y);
        }

        const auto smoothed = buildMapSplineBoundary(basePoints);
        m_mapSplineVertices.reserve(smoothed.size());
        for (const auto& point : smoothed)
        {
            m_mapSplineVertices.push_back({glm::vec3(point.x, point.y, 0.0F), 1.0F});
        }
    }
    m_mapSplineDirty = true;
}

void RadarVisualizer::updateMapSegments(const std::vector<glm::vec3>& points)
{
    m_mapSegmentVertices.clear();
    m_mapSegmentVertices.reserve(points.size());
    for (const auto& point : points)
    {
        glm::vec3 position = point;
        if (m_vcsToIsoEnabled)
        {
            position = glm::vec3(-position.x, position.y + m_vcsToIsoLongitudinalOffset, position.z);
        }
        m_mapSegmentVertices.push_back({position, 1.0F});
    }
    m_mapSegmentDirty = true;
}

void RadarVisualizer::updateVehicleContour(const std::vector<glm::vec2>& contourPoints)
{
    m_contourVertices.clear();
    m_contourVertices.reserve(contourPoints.size());
    for (const auto& point : contourPoints)
    {
        m_contourVertices.push_back({glm::vec3(point.x, point.y, 0.0F), 1.0F});
    }
    m_contourDirty = true;
}

void RadarVisualizer::uploadMapBuffer()
{
    if (!m_mapDirty || m_mapVertices.empty())
    {
        return;
    }

    glBindBuffer(GL_ARRAY_BUFFER, m_mapVbo);
    glBufferData(GL_ARRAY_BUFFER, m_mapVertices.size() * sizeof(Vertex), m_mapVertices.data(), GL_DYNAMIC_DRAW);
    m_mapDirty = false;
}

void RadarVisualizer::uploadMapSegmentBuffer()
{
    if (!m_mapSegmentDirty || m_mapSegmentVertices.empty())
    {
        return;
    }

    glBindBuffer(GL_ARRAY_BUFFER, m_mapSegmentVbo);
    glBufferData(GL_ARRAY_BUFFER, m_mapSegmentVertices.size() * sizeof(Vertex), m_mapSegmentVertices.data(),
                 GL_DYNAMIC_DRAW);
    m_mapSegmentDirty = false;
}

void RadarVisualizer::uploadMapSplineBuffer()
{
    if (!m_mapSplineDirty || m_mapSplineVertices.empty())
    {
        return;
    }

    glBindBuffer(GL_ARRAY_BUFFER, m_mapSplineVbo);
    glBufferData(GL_ARRAY_BUFFER, m_mapSplineVertices.size() * sizeof(Vertex), m_mapSplineVertices.data(),
                 GL_DYNAMIC_DRAW);
    m_mapSplineDirty = false;
}

void RadarVisualizer::uploadContourBuffer()
{
    if (!m_contourDirty || m_contourVertices.empty())
    {
        return;
    }

    glBindBuffer(GL_ARRAY_BUFFER, m_contourVbo);
    glBufferData(GL_ARRAY_BUFFER, m_contourVertices.size() * sizeof(Vertex), m_contourVertices.data(),
                 GL_DYNAMIC_DRAW);
    m_contourDirty = false;
}

void RadarVisualizer::buildGridVertices()
{
    m_gridVertices.clear();
    if (!m_showGrid)
    {
        return;
    }

    const float gridSpacing = std::max(0.01F, m_gridSpacing);
    const float startX = std::floor(m_gridMin.x / gridSpacing) * gridSpacing;
    const float endX = std::ceil(m_gridMax.x / gridSpacing) * gridSpacing;
    const float startY = std::floor(m_gridMin.y / gridSpacing) * gridSpacing;
    const float endY = std::ceil(m_gridMax.y / gridSpacing) * gridSpacing;

    for (float x = startX; x <= endX; x += gridSpacing)
    {
        m_gridVertices.push_back({glm::vec3(x, startY, 0.0F), m_gridAlpha});
        m_gridVertices.push_back({glm::vec3(x, endY, 0.0F), m_gridAlpha});
    }
    for (float y = startY; y <= endY; y += gridSpacing)
    {
        m_gridVertices.push_back({glm::vec3(startX, y, 0.0F), m_gridAlpha});
        m_gridVertices.push_back({glm::vec3(endX, y, 0.0F), m_gridAlpha});
    }
}

void RadarVisualizer::uploadGridBuffer()
{
    if (!m_gridDirty)
    {
        return;
    }

    buildGridVertices();
    if (m_gridVertices.empty())
    {
        m_gridDirty = false;
        return;
    }

    glBindBuffer(GL_ARRAY_BUFFER, m_gridVbo);
    glBufferData(GL_ARRAY_BUFFER, m_gridVertices.size() * sizeof(Vertex), m_gridVertices.data(), GL_DYNAMIC_DRAW);
    m_gridDirty = false;
}

void RadarVisualizer::setupVertexAttributes(GLuint vao, GLuint vbo)
{
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, position)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, intensity)));
    glBindVertexArray(0);
}

void RadarVisualizer::setResetMapCallback(std::function<void()> callback)
{
    m_resetMapCallback = std::move(callback);
}

void RadarVisualizer::setVcsToIsoTransform(float distRearAxle)
{
    m_vcsToIsoEnabled = true;
    m_vcsToIsoLongitudinalOffset = distRearAxle;
}

glm::mat4 RadarVisualizer::computeViewProjection() const
{
    int width = 1280;
    int height = 720;
    if (m_window)
    {
        glfwGetFramebufferSize(m_window, &width, &height);
    }

    const float aspect = width > 0 && height > 0 ? static_cast<float>(width) / height : 1.0F;
    const glm::mat4 projection = glm::perspective(glm::radians(m_camera.fov), aspect, 0.1F, 500.0F);
    const glm::vec3 direction = computeCameraDirection();
    const glm::vec3 cameraPosition = m_cameraTarget - direction * m_camera.distance;
    const glm::mat4 view = glm::lookAt(cameraPosition, m_cameraTarget, computeCameraUp());
    return projection * view;
}

glm::vec3 RadarVisualizer::computeCameraDirection() const
{
    switch (m_cameraMode)
    {
        case CameraMode::BirdsEye:
            return glm::vec3(0.0F, 0.0F, -1.0F);
        case CameraMode::Front:
            return glm::vec3(0.0F, -1.0F, 0.0F);
        case CameraMode::Side:
            return glm::vec3(1.0F, 0.0F, 0.0F);
        case CameraMode::Rear:
            return glm::vec3(0.0F, 1.0F, 0.0F);
        default:
        {
            const float pitchRad = glm::radians(m_camera.pitch);
            const float yawRad = glm::radians(m_camera.yaw);
            return glm::vec3(std::cos(pitchRad) * std::cos(yawRad),
                             std::cos(pitchRad) * std::sin(yawRad),
                             std::sin(pitchRad));
        }
    }
}

glm::vec3 RadarVisualizer::computeCameraUp() const
{
    if (m_cameraMode == CameraMode::BirdsEye)
    {
        return glm::vec3(0.0F, 1.0F, 0.0F);
    }
    return glm::vec3(0.0F, 0.0F, 1.0F);
}

void RadarVisualizer::processCursorPos(double xpos, double ypos)
{
    if (m_cameraMode != CameraMode::FreeOrbit || !m_camera.rotating || m_activeMouseButton == -1)
    {
        m_camera.lastX = xpos;
        m_camera.lastY = ypos;
        return;
    }

    const float dx = static_cast<float>(xpos - m_camera.lastX);
    const float dy = static_cast<float>(ypos - m_camera.lastY);
    m_camera.lastX = xpos;
    m_camera.lastY = ypos;

    m_camera.yaw += dx * 0.35F;
    m_camera.pitch -= dy * 0.35F;
    m_camera.pitch = std::clamp(m_camera.pitch, -89.0F, 89.0F);
}

void RadarVisualizer::processScroll(double yoffset)
{
    m_camera.distance = std::clamp(m_camera.distance - static_cast<float>(yoffset) * 2.0F, 2.0F, 200.0F);
}

void RadarVisualizer::processMouseButton(int button, int action)
{
    if (m_cameraMode != CameraMode::FreeOrbit)
    {
        return;
    }

    const bool rotationButton =
        button == GLFW_MOUSE_BUTTON_RIGHT || button == GLFW_MOUSE_BUTTON_LEFT || button == GLFW_MOUSE_BUTTON_MIDDLE;
    if (!rotationButton)
    {
        return;
    }

    if (ImGui::GetIO().WantCaptureMouse && action == GLFW_PRESS)
    {
        return;
    }

    if (action == GLFW_PRESS)
    {
        m_camera.rotating = true;
        m_activeMouseButton = button;
        if (m_window)
        {
            glfwGetCursorPos(m_window, &m_camera.lastX, &m_camera.lastY);
        }
    }
    else if (action == GLFW_RELEASE && button == m_activeMouseButton)
    {
        m_camera.rotating = false;
        m_activeMouseButton = -1;
    }
}

void RadarVisualizer::cursorPosCallback(GLFWwindow* window, double xpos, double ypos)
{
    if (auto* self = reinterpret_cast<RadarVisualizer*>(glfwGetWindowUserPointer(window)))
    {
        self->processCursorPos(xpos, ypos);
    }
}

void RadarVisualizer::scrollCallback(GLFWwindow* window, double /*xoffset*/, double yoffset)
{
    if (auto* self = reinterpret_cast<RadarVisualizer*>(glfwGetWindowUserPointer(window)))
    {
        self->processScroll(yoffset);
    }
}

void RadarVisualizer::mouseButtonCallback(GLFWwindow* window, int button, int action, int /*mods*/)
{
    if (auto* self = reinterpret_cast<RadarVisualizer*>(glfwGetWindowUserPointer(window)))
    {
        self->processMouseButton(button, action);
    }
}

bool RadarVisualizer::passesMotionFilter(const radar::RadarPoint& point) const
{
    switch (m_detectionMotionFilter)
    {
        case DetectionMotionFilter::Static:
            return point.motionStatus == 0;
        case DetectionMotionFilter::Moving:
            return point.motionStatus == 1;
        case DetectionMotionFilter::All:
        default:
            return true;
    }
}

RadarVisualizer::DetectionType RadarVisualizer::detectionTypeForPoint(const radar::RadarPoint& point) const
{
    if (point.multibounce)
    {
        return DetectionType::MultiBounce;
    }
    if (point.hostVehicleClutter)
    {
        return DetectionType::HostVehicleClutter;
    }
    if (point.nearTarget)
    {
        return DetectionType::NdTarget;
    }
    if (point.superResolution)
    {
        return DetectionType::SuperRes;
    }
    if (point.radarValid)
    {
        return DetectionType::Valid;
    }
    return DetectionType::Unknown;
}

glm::vec3 RadarVisualizer::colorForSensor(int sensorIndex) const
{
    static const std::array<glm::vec3, 6> kSensorPalette = {
        glm::vec3(0.95F, 0.75F, 0.25F),
        glm::vec3(0.2F, 0.8F, 0.85F),
        glm::vec3(0.25F, 0.45F, 0.95F),
        glm::vec3(0.85F, 0.3F, 0.85F),
        glm::vec3(0.3F, 0.25F, 0.9F),
        glm::vec3(0.7F, 0.7F, 0.7F)};
    if (sensorIndex < 0)
    {
        return kSensorPalette.back();
    }
    const size_t paletteIndex = static_cast<size_t>(sensorIndex) % (kSensorPalette.size() - 1);
    return kSensorPalette[paletteIndex];
}

glm::vec3 RadarVisualizer::colorForDetectionType(DetectionType type) const
{
    switch (type)
    {
        case DetectionType::Valid:
            return m_detectionTypeColors[0];
        case DetectionType::SuperRes:
            return m_detectionTypeColors[1];
        case DetectionType::NdTarget:
            return m_detectionTypeColors[2];
        case DetectionType::HostVehicleClutter:
            return m_detectionTypeColors[3];
        case DetectionType::MultiBounce:
            return m_detectionTypeColors[4];
        default:
            return glm::vec3(0.4F, 0.4F, 0.4F);
    }
}

glm::vec3 RadarVisualizer::trackColor(const radar::RadarTrack& track) const
{
    const float speed = glm::length(track.isoVelocity);
    if (track.isMoving || speed >= m_trackSpeedThreshold)
    {
        return m_trackMovingColor;
    }
    if (track.isStationary)
    {
        return m_trackStationaryColor;
    }
    return m_trackUnknownColor;
}

float RadarVisualizer::computeDetectionAlpha(const radar::RadarPoint& point, float ageSeconds) const
{
    float alpha = m_detectionAlphaConstant;
    switch (m_detectionAlphaMode)
    {
        case DetectionAlphaMode::MotionProbability:
        {
            float stationaryProbability = point.stationaryProbability;
            if (stationaryProbability <= 0.0F)
            {
                const float scale = std::max(0.1F, m_rangeRateStationaryScale);
                stationaryProbability = std::exp(-std::abs(point.rangeRate_ms) / scale);
            }

            if (point.isStationary || point.motionStatus == 0)
            {
                alpha = stationaryProbability;
            }
            else if (point.isMoveable || point.motionStatus == 1)
            {
                alpha = 1.0F - stationaryProbability;
            }
            else
            {
                alpha = 0.5F;
            }
            break;
        }
        case DetectionAlphaMode::TimeDecay:
        {
            const float window = std::max(0.01F, m_lastFramePeriodSec * std::max(1, m_detectionScanRetention));
            alpha = std::exp(-m_detectionAlphaDecay * ageSeconds / window);
            break;
        }
        case DetectionAlphaMode::Constant:
        default:
            break;
    }

    alpha = std::clamp(alpha * m_intensityScale, 0.05F, 1.0F);
    return alpha;
}

void RadarVisualizer::drawUI()
{
    ImGui::Begin("Radar Controls");
    ImGui::SliderFloat("Point size", &m_pointSize, 1.0F, 12.0F);
    ImGui::SliderFloat("Intensity scale", &m_intensityScale, 0.1F, 5.0F);
    ImGui::SliderFloat("Map intensity", &m_mapIntensity, 0.2F, 5.0F);
    ImGui::SliderFloat("Replay speed", &m_replaySpeed, 0.1F, 3.0F);
    ImGui::Checkbox("Show radar map overlay", &m_showMapping);
    ImGui::Checkbox("Show B-spline map", &m_showBsplineMap);
    ImGui::SliderInt("Map segments", &m_mapSegmentCount, kMapSegmentMin, kMapSegmentMax);
    ImGui::SliderInt("B-spline control points", &m_mapSplineControlPointCount,
                     kMapSplineControlPointMin, kMapSplineControlPointMax);
    ImGui::Checkbox("Show vehicle contour", &m_showVehicleContour);
    ImGui::SliderFloat("Contour width", &m_contourLineWidth, 1.0F, 6.0F);
    bool gridDirty = false;
    gridDirty |= ImGui::Checkbox("Show grid", &m_showGrid);
    gridDirty |= ImGui::SliderFloat("Grid spacing (m)", &m_gridSpacing, 5.0F, 100.0F, "%.0f");
    gridDirty |= ImGui::SliderFloat("Grid size (m)", &m_gridHalfSpan, 10.0F, 200.0F, "%.0f");
    gridDirty |= ImGui::SliderFloat("Grid alpha", &m_gridAlpha, 0.1F, 1.0F, "%.2f");
    if (gridDirty)
    {
        m_gridMin = glm::vec2(-m_gridHalfSpan, -m_gridHalfSpan);
        m_gridMax = glm::vec2(m_gridHalfSpan, m_gridHalfSpan);
        m_gridDirty = true;
    }
    if (ImGui::Button("Reset radar map"))
    {
        if (m_resetMapCallback)
        {
            m_resetMapCallback();
        }
    }
    if (ImGui::TreeNodeEx("Detections", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Checkbox("Enable detections", &m_showDetections);
        ImGui::Checkbox("Display elevation", &m_displayElevation);
        ImGui::Checkbox("Persistent detections", &m_enablePersistentDetections);
        if (m_enablePersistentDetections)
        {
            ImGui::SliderInt("Scan retention", &m_detectionScanRetention, 1, 300);
        }

        int colorMode = static_cast<int>(m_detectionColorMode);
        if (ImGui::Combo("Color mode", &colorMode, kDetectionColorModeLabels.data(),
                         static_cast<int>(kDetectionColorModeLabels.size())))
        {
            m_detectionColorMode = static_cast<DetectionColorMode>(colorMode);
        }

        int alphaMode = static_cast<int>(m_detectionAlphaMode);
        if (ImGui::Combo("Alpha mode", &alphaMode, kDetectionAlphaModeLabels.data(),
                         static_cast<int>(kDetectionAlphaModeLabels.size())))
        {
            m_detectionAlphaMode = static_cast<DetectionAlphaMode>(alphaMode);
        }

        int motionFilter = static_cast<int>(m_detectionMotionFilter);
        if (ImGui::Combo("Motion filter", &motionFilter, kDetectionMotionFilterLabels.data(),
                         static_cast<int>(kDetectionMotionFilterLabels.size())))
        {
            m_detectionMotionFilter = static_cast<DetectionMotionFilter>(motionFilter);
        }

        if (m_detectionAlphaMode == DetectionAlphaMode::Constant)
        {
            ImGui::SliderFloat("Alpha constant", &m_detectionAlphaConstant, 0.05F, 1.0F);
        }
        else if (m_detectionAlphaMode == DetectionAlphaMode::TimeDecay)
        {
            ImGui::SliderFloat("Decay constant", &m_detectionAlphaDecay, 0.05F, 2.0F);
        }

        if (m_detectionColorMode == DetectionColorMode::MotionState)
        {
            ImGuiColorEditFlags colorFlags = ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_Float;
            ImGui::ColorEdit3("Static color", &m_staticColor.x, colorFlags);
            ImGui::ColorEdit3("Moving color", &m_movingColor.x, colorFlags);
            ImGui::ColorEdit3("Ambiguous color", &m_ambiguousColor.x, colorFlags);
        }
        else if (m_detectionColorMode == DetectionColorMode::DetectionType)
        {
            ImGuiColorEditFlags colorFlags = ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_Float;
            ImGui::ColorEdit3("Valid", &m_detectionTypeColors[0].x, colorFlags);
            ImGui::SameLine();
            ImGui::Checkbox("Show", &m_displayValid);
            ImGui::ColorEdit3("Super res", &m_detectionTypeColors[1].x, colorFlags);
            ImGui::SameLine();
            ImGui::Checkbox("Show##Super", &m_displaySuperRes);
            ImGui::ColorEdit3("ND target", &m_detectionTypeColors[2].x, colorFlags);
            ImGui::SameLine();
            ImGui::Checkbox("Show##Nd", &m_displayNdTarget);
            ImGui::ColorEdit3("Host clutter", &m_detectionTypeColors[3].x, colorFlags);
            ImGui::SameLine();
            ImGui::Checkbox("Show##Host", &m_displayHostVehicleClutter);
            ImGui::ColorEdit3("Multi-bounce", &m_detectionTypeColors[4].x, colorFlags);
            ImGui::SameLine();
            ImGui::Checkbox("Show##Multi", &m_displayMultiBounce);
        }
        ImGui::TreePop();
    }
    if (ImGui::TreeNodeEx("Tracks", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Checkbox("Show tracks", &m_showTracks);
        ImGui::SliderFloat("Line width", &m_trackLineWidth, 0.5F, 4.0F);
        ImGui::SliderFloat("Alpha", &m_trackAlpha, 0.1F, 1.0F);
        ImGui::SliderFloat("Moving speed [m/s]", &m_trackSpeedThreshold, 0.0F, 5.0F);
        ImGuiColorEditFlags colorFlags = ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_Float;
        ImGui::ColorEdit3("Moving", &m_trackMovingColor.x, colorFlags);
        ImGui::ColorEdit3("Stationary", &m_trackStationaryColor.x, colorFlags);
        ImGui::ColorEdit3("Unknown", &m_trackUnknownColor.x, colorFlags);
        ImGui::TreePop();
    }
    if (ImGui::TreeNodeEx("Sensor FOV", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Checkbox("Show FOV polygons", &m_showFov);
        ImGui::SliderFloat("FOV alpha", &m_fovAlpha, 0.05F, 0.8F);
        for (auto& entry : m_fovRangeOverride)
        {
            const int sensorId = entry.first;
            std::string label = "Range (sensor " + std::to_string(sensorId) + ")";
            ImGui::SliderFloat(label.c_str(), &entry.second, 5.0F, 200.0F);
        }
        ImGui::TreePop();
    }
    ImGui::Text("Points: %zu", m_currentPoints.size());
    ImGui::Text("Map points: %zu", m_mapVertices.size());
    ImGui::Text("Map segments: %zu", m_mapSegmentVertices.size() / 2U);
    ImGui::Text("B-spline points: %zu", m_mapSplineVertices.size());
    ImGui::Text("Tracks: %zu", m_tracks.size());
    ImGui::Text("Timestamp: %llu", static_cast<unsigned long long>(m_lastTimestampUs));
    {
        std::string sourcesLabel = m_lastSources.empty() ? "none" : "";
        for (size_t i = 0; i < m_lastSources.size(); ++i)
        {
            if (i > 0)
            {
                sourcesLabel += ", ";
            }
            sourcesLabel += m_lastSources[i];
        }
        ImGui::Text("Sources: %s", sourcesLabel.c_str());
    }
    ImGui::End();

    ImGui::Begin("Camera");
    int cameraModeIdx = static_cast<int>(m_cameraMode);
    if (ImGui::Combo("Camera view", &cameraModeIdx, kCameraModeLabels.data(),
                     static_cast<int>(kCameraModeLabels.size())))
    {
        m_cameraMode = static_cast<CameraMode>(cameraModeIdx);
        m_camera.rotating = false;
        m_activeMouseButton = -1;
    }
    ImGui::SliderFloat("Distance", &m_camera.distance, 2.0F, 200.0F);
    ImGui::SliderFloat("FOV", &m_camera.fov, 20.0F, 90.0F);
    if (m_cameraMode == CameraMode::FreeOrbit)
    {
        ImGui::SliderFloat("Yaw (deg)", &m_camera.yaw, -180.0F, 180.0F);
        ImGui::SliderFloat("Pitch (deg)", &m_camera.pitch, -89.0F, 89.0F);
    }
    if (ImGui::Button("Reset camera"))
    {
        m_camera.distance = 30.0F;
        m_camera.yaw = -90.0F;
        m_camera.pitch = 30.0F;
        m_camera.fov = 45.0F;
    }
    ImGui::End();

}

void RadarVisualizer::render()
{
    if (!m_window)
    {
        return;
    }

    glfwPollEvents();
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    drawUI();

    int width = 1280;
    int height = 720;
    if (m_window)
    {
        glfwGetFramebufferSize(m_window, &width, &height);
    }
    glViewport(0, 0, width, height);
    glClearColor(0.05F, 0.05F, 0.05F, 1.0F);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    const bool hasDetections = m_showDetections && !m_detectionHistory.empty();
    const bool hasMapLoop = m_showMapping && m_mapVertices.size() > 1U;
    const bool hasMapSegments = m_showMapping && m_mapSegmentVertices.size() > 1U;
    const bool hasMapSpline = m_showBsplineMap && m_mapSplineVertices.size() > 1U;
    const bool hasContour = m_showVehicleContour && m_contourVertices.size() > 1U;
    const bool hasGrid = m_showGrid;
    const bool hasFov = m_showFov && !m_fovBySensor.empty();
    const bool hasTracks = m_showTracks && !m_tracks.empty();
    glm::mat4 vp = glm::mat4(1.0F);
    if (hasDetections || hasMapLoop || hasMapSegments || hasMapSpline || hasContour || hasGrid || hasFov || hasTracks)
    {
        vp = computeViewProjection();
        m_shader.use();
        const GLint vpLoc = m_shader.uniformLocation("uViewProjection");
        if (vpLoc >= 0)
        {
            glUniformMatrix4fv(vpLoc, 1, GL_FALSE, glm::value_ptr(vp));
        }

    }

    if (hasGrid)
    {
        uploadGridBuffer();
        if (!m_gridVertices.empty())
        {
            m_shader.use();
            const GLint vpLoc = m_shader.uniformLocation("uViewProjection");
            if (vpLoc >= 0)
            {
                glUniformMatrix4fv(vpLoc, 1, GL_FALSE, glm::value_ptr(vp));
            }

            const GLint pointSizeLoc = m_shader.uniformLocation("uPointSize");
            if (pointSizeLoc >= 0)
            {
                glUniform1f(pointSizeLoc, 1.0F);
            }

            const GLint intensityLoc = m_shader.uniformLocation("uIntensityScale");
            if (intensityLoc >= 0)
            {
                glUniform1f(intensityLoc, 1.0F);
            }

            const GLint colorLoc = m_shader.uniformLocation("uBaseColor");
            if (colorLoc >= 0)
            {
                glUniform3f(colorLoc, m_gridColor.r, m_gridColor.g, m_gridColor.b);
            }

            glLineWidth(1.0F);
            glBindVertexArray(m_gridVao);
            glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(m_gridVertices.size()));
            glBindVertexArray(0);
        }
    }

    if (hasFov)
    {
        drawFovPolygons(vp);
    }

    if (hasDetections)
    {
        drawDetections(vp);
    }

    if (hasTracks)
    {
        drawTracks(vp);
    }

    if (hasMapLoop || hasMapSegments || hasMapSpline)
    {
        m_shader.use();
        const GLint vpLoc = m_shader.uniformLocation("uViewProjection");
        if (vpLoc >= 0)
        {
            glUniformMatrix4fv(vpLoc, 1, GL_FALSE, glm::value_ptr(vp));
        }

        const GLint pointSizeLoc = m_shader.uniformLocation("uPointSize");
        if (pointSizeLoc >= 0)
        {
            glUniform1f(pointSizeLoc, 1.0F);
        }

        const GLint intensityLoc = m_shader.uniformLocation("uIntensityScale");
        if (intensityLoc >= 0)
        {
            glUniform1f(intensityLoc, m_mapIntensity);
        }

        const glm::vec3 mapColor(1.0F, 0.75F, 0.35F);
        const glm::vec3 mapSplineColor(1.0F, 0.4F, 0.75F);
        const GLint colorLoc = m_shader.uniformLocation("uBaseColor");
        if (colorLoc >= 0)
        {
            glUniform3f(colorLoc, mapColor.r, mapColor.g, mapColor.b);
        }

        glLineWidth(2.0F);
        if (hasMapLoop)
        {
            uploadMapBuffer();
            glBindVertexArray(m_mapVao);
            glDrawArrays(GL_LINE_LOOP, 0, static_cast<GLsizei>(m_mapVertices.size()));
            glBindVertexArray(0);
        }
        if (hasMapSegments)
        {
            uploadMapSegmentBuffer();
            glBindVertexArray(m_mapSegmentVao);
            glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(m_mapSegmentVertices.size()));
            glBindVertexArray(0);
        }
        if (hasMapSpline)
        {
            uploadMapSplineBuffer();
            glLineWidth(2.5F);
            if (colorLoc >= 0)
            {
                glUniform3f(colorLoc, mapSplineColor.r, mapSplineColor.g, mapSplineColor.b);
            }
            glBindVertexArray(m_mapSplineVao);
            glDrawArrays(GL_LINE_LOOP, 0, static_cast<GLsizei>(m_mapSplineVertices.size()));
            glBindVertexArray(0);
        }
        glLineWidth(1.0F);
    }

    if (hasContour)
    {
        uploadContourBuffer();
        m_shader.use();
        const GLint vpLoc = m_shader.uniformLocation("uViewProjection");
        if (vpLoc >= 0)
        {
            glUniformMatrix4fv(vpLoc, 1, GL_FALSE, glm::value_ptr(vp));
        }

        const GLint pointSizeLoc = m_shader.uniformLocation("uPointSize");
        if (pointSizeLoc >= 0)
        {
            glUniform1f(pointSizeLoc, 1.0F);
        }

        const GLint intensityLoc = m_shader.uniformLocation("uIntensityScale");
        if (intensityLoc >= 0)
        {
            glUniform1f(intensityLoc, 1.0F);
        }

        const GLint colorLoc = m_shader.uniformLocation("uBaseColor");
        if (colorLoc >= 0)
        {
            glUniform3f(colorLoc, 1.0F, 0.85F, 0.2F);
        }

        glLineWidth(m_contourLineWidth);
        glBindVertexArray(m_contourVao);
        glDrawArrays(GL_LINE_LOOP, 0, static_cast<GLsizei>(m_contourVertices.size()));
        glBindVertexArray(0);
        glLineWidth(1.0F);
    }

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(m_window);
}

void RadarVisualizer::drawDetections(const glm::mat4& viewProjection)
{
    if (m_detectionHistory.empty())
    {
        return;
    }

    std::unordered_map<int, std::vector<Vertex>> buckets;
    const uint64_t currentTimestamp = m_lastTimestampUs;

    for (const auto& frame : m_detectionHistory)
    {
        const float ageSeconds =
            currentTimestamp > frame.timestampUs
                ? static_cast<float>(currentTimestamp - frame.timestampUs) / 1'000'000.0F
                : 0.0F;

        for (const auto& point : frame.points)
        {
            if (!passesMotionFilter(point))
            {
                continue;
            }

            const DetectionType type = detectionTypeForPoint(point);
            if (type == DetectionType::Valid && !m_displayValid)
            {
                continue;
            }
            if (type == DetectionType::SuperRes && !m_displaySuperRes)
            {
                continue;
            }
            if (type == DetectionType::NdTarget && !m_displayNdTarget)
            {
                continue;
            }
            if (type == DetectionType::HostVehicleClutter && !m_displayHostVehicleClutter)
            {
                continue;
            }
            if (type == DetectionType::MultiBounce && !m_displayMultiBounce)
            {
                continue;
            }

            float alpha = computeDetectionAlpha(point, ageSeconds);
            if (alpha <= 0.05F)
            {
                continue;
            }

            glm::vec3 position(point.x, point.y, m_displayElevation ? point.z : 0.0F);
            int bucketId = 0;
            switch (m_detectionColorMode)
            {
                case DetectionColorMode::RadarUnit:
                    bucketId = point.sensorIndex;
                    break;
                case DetectionColorMode::DetectionType:
                    bucketId = static_cast<int>(type);
                    break;
                case DetectionColorMode::MotionState:
                default:
                    bucketId = point.motionStatus;
                    break;
            }

            buckets[bucketId].push_back({position, alpha});
        }
    }

    if (buckets.empty())
    {
        return;
    }

    m_shader.use();
    const GLint vpLoc = m_shader.uniformLocation("uViewProjection");
    if (vpLoc >= 0)
    {
        glUniformMatrix4fv(vpLoc, 1, GL_FALSE, glm::value_ptr(viewProjection));
    }

    const GLint pointSizeLoc = m_shader.uniformLocation("uPointSize");
    if (pointSizeLoc >= 0)
    {
        glUniform1f(pointSizeLoc, m_pointSize);
    }

    const GLint intensityLoc = m_shader.uniformLocation("uIntensityScale");
    if (intensityLoc >= 0)
    {
        glUniform1f(intensityLoc, 1.0F);
    }

    for (const auto& bucket : buckets)
    {
        if (bucket.second.empty())
        {
            continue;
        }

        glm::vec3 color = m_movingColor;
        if (m_detectionColorMode == DetectionColorMode::RadarUnit)
        {
            color = colorForSensor(bucket.first);
        }
        else if (m_detectionColorMode == DetectionColorMode::DetectionType)
        {
            color = colorForDetectionType(static_cast<DetectionType>(bucket.first));
        }
        else
        {
            if (bucket.first == 0)
            {
                color = m_staticColor;
            }
            else if (bucket.first == 1)
            {
                color = m_movingColor;
            }
            else
            {
                color = m_ambiguousColor;
            }
        }

        const GLint colorLoc = m_shader.uniformLocation("uBaseColor");
        if (colorLoc >= 0)
        {
            glUniform3f(colorLoc, color.r, color.g, color.b);
        }

        glBindVertexArray(m_vao);
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
        glBufferData(GL_ARRAY_BUFFER,
                     bucket.second.size() * sizeof(Vertex),
                     bucket.second.data(),
                     GL_DYNAMIC_DRAW);
        glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(bucket.second.size()));
        glBindVertexArray(0);
    }
}

void RadarVisualizer::drawTracks(const glm::mat4& viewProjection)
{
    if (m_tracks.empty())
    {
        return;
    }

    m_shader.use();
    const GLint vpLoc = m_shader.uniformLocation("uViewProjection");
    if (vpLoc >= 0)
    {
        glUniformMatrix4fv(vpLoc, 1, GL_FALSE, glm::value_ptr(viewProjection));
    }

    const GLint pointSizeLoc = m_shader.uniformLocation("uPointSize");
    if (pointSizeLoc >= 0)
    {
        glUniform1f(pointSizeLoc, 1.0F);
    }

    const GLint intensityLoc = m_shader.uniformLocation("uIntensityScale");
    if (intensityLoc >= 0)
    {
        glUniform1f(intensityLoc, 1.0F);
    }

    glBindVertexArray(m_trackVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_trackVbo);
    glLineWidth(m_trackLineWidth);

    for (const auto& track : m_tracks)
    {
        const float halfLength = std::max(track.length, 0.1F) * 0.5F;
        const float halfWidth = std::max(track.width, 0.1F) * 0.5F;
        const float height = std::max(track.height, 0.05F);

        const glm::vec2 center(track.isoPosition.y, track.isoPosition.x);
        const float heading = track.headingRad;
        const glm::vec2 forward(std::sin(heading), std::cos(heading));
        const glm::vec2 right(forward.y, -forward.x);

        const glm::vec2 p0 = center + forward * halfLength + right * halfWidth;
        const glm::vec2 p1 = center - forward * halfLength + right * halfWidth;
        const glm::vec2 p2 = center - forward * halfLength - right * halfWidth;
        const glm::vec2 p3 = center + forward * halfLength - right * halfWidth;

        const float alpha = std::clamp(m_trackAlpha, 0.05F, 1.0F);
        std::vector<Vertex> vertices;
        vertices.reserve(24 * 2);

        auto pushEdge = [&vertices, alpha](const glm::vec3& a, const glm::vec3& b)
        {
            vertices.push_back({a, alpha});
            vertices.push_back({b, alpha});
        };

        const glm::vec3 b0(p0.x, p0.y, 0.0F);
        const glm::vec3 b1(p1.x, p1.y, 0.0F);
        const glm::vec3 b2(p2.x, p2.y, 0.0F);
        const glm::vec3 b3(p3.x, p3.y, 0.0F);
        const glm::vec3 t0(p0.x, p0.y, height);
        const glm::vec3 t1(p1.x, p1.y, height);
        const glm::vec3 t2(p2.x, p2.y, height);
        const glm::vec3 t3(p3.x, p3.y, height);

        pushEdge(b0, b1);
        pushEdge(b1, b2);
        pushEdge(b2, b3);
        pushEdge(b3, b0);
        pushEdge(t0, t1);
        pushEdge(t1, t2);
        pushEdge(t2, t3);
        pushEdge(t3, t0);
        pushEdge(b0, t0);
        pushEdge(b1, t1);
        pushEdge(b2, t2);
        pushEdge(b3, t3);

        const glm::vec3 color = trackColor(track);
        const GLint colorLoc = m_shader.uniformLocation("uBaseColor");
        if (colorLoc >= 0)
        {
            glUniform3f(colorLoc, color.r, color.g, color.b);
        }

        glBufferData(GL_ARRAY_BUFFER,
                     vertices.size() * sizeof(Vertex),
                     vertices.data(),
                     GL_DYNAMIC_DRAW);
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(vertices.size()));
    }

    glBindVertexArray(0);
    glLineWidth(1.0F);
}

std::vector<glm::vec2> RadarVisualizer::buildMapSplineBoundary(const std::vector<glm::vec2>& basePoints) const
{
    if (basePoints.size() < 3)
    {
        return {};
    }

    try
    {
        const int clampedControl =
            std::clamp(m_mapSplineControlPointCount, kMapSplineControlPointMin, kMapSplineControlPointMax);
        const std::size_t controlCount =
            std::max<std::size_t>(3U, static_cast<std::size_t>(clampedControl));
        const std::vector<glm::vec2> controlPoints =
            resampleLoop(basePoints, controlCount);
        const auto& splineInputs = controlPoints.size() >= 3U ? controlPoints : basePoints;

        std::vector<double> parameters(splineInputs.size());
        std::vector<double> xs(splineInputs.size());
        std::vector<double> ys(splineInputs.size());
        for (std::size_t i = 0; i < splineInputs.size(); ++i)
        {
            parameters[i] = static_cast<double>(i);
            xs[i] = splineInputs[i].x;
            ys[i] = splineInputs[i].y;
        }

        const auto smoothedX = sampleBspline(parameters, xs, kMapSplineSampleCount);
        const auto smoothedY = sampleBspline(parameters, ys, kMapSplineSampleCount);

        if (smoothedX.size() != smoothedY.size() || smoothedX.empty())
        {
            return basePoints;
        }

        std::vector<glm::vec2> result;
        result.reserve(smoothedX.size());
        for (std::size_t i = 0; i < smoothedX.size(); ++i)
        {
            result.emplace_back(static_cast<float>(smoothedX[i]), static_cast<float>(smoothedY[i]));
        }
        return result;
    }
    catch (const SPLINTER::Exception&)
    {
        return basePoints;
    }
    catch (...)
    {
        return basePoints;
    }
}

std::vector<double> RadarVisualizer::sampleBspline(const std::vector<double>& parameters,
                                                   const std::vector<double>& values,
                                                   std::size_t resolution) const
{
    if (parameters.size() != values.size() || parameters.empty() || resolution == 0)
    {
        return {};
    }

    SPLINTER::DataTable data;
    for (std::size_t i = 0; i < parameters.size(); ++i)
    {
        data.addSample(std::vector<double>{parameters[i]}, values[i]);
    }

    SPLINTER::BSpline::Builder builder(data);
    const unsigned int desiredOrder = std::max<unsigned int>(
        3U,
        std::min<unsigned int>(static_cast<unsigned int>(parameters.size() / 16U) + 3U, 5U));
    builder.degree(desiredOrder);

    const unsigned int maxBasis = std::clamp(
        static_cast<unsigned int>(parameters.size() * 10U),
        desiredOrder + 1U,
        1024U);
    builder.numBasisFunctions(std::vector<unsigned int>{maxBasis});

    builder.knotSpacing(SPLINTER::BSpline::KnotSpacing::AS_SAMPLED);
    builder.smoothing(SPLINTER::BSpline::Smoothing::PSPLINE);

    const auto bspline = builder.build();
    const double lower = bspline.getDomainLowerBound()[0];
    const double upper = bspline.getDomainUpperBound()[0];
    if (upper <= lower)
    {
        return std::vector<double>(1, values.front());
    }

    std::vector<double> smoothed;
    smoothed.reserve(resolution + 1);
    for (std::size_t step = 0; step <= resolution; ++step)
    {
        const double factor = static_cast<double>(step) / static_cast<double>(resolution);
        const double t = lower + (upper - lower) * factor;
        SPLINTER::DenseVector argument(1);
        argument(0) = t;
        smoothed.push_back(bspline.eval(argument));
    }

    return smoothed;
}

void RadarVisualizer::drawFovPolygons(const glm::mat4& viewProjection)
{
    if (m_fovBySensor.empty())
    {
        return;
    }

    m_shader.use();
    const GLint vpLoc = m_shader.uniformLocation("uViewProjection");
    if (vpLoc >= 0)
    {
        glUniformMatrix4fv(vpLoc, 1, GL_FALSE, glm::value_ptr(viewProjection));
    }

    const GLint pointSizeLoc = m_shader.uniformLocation("uPointSize");
    if (pointSizeLoc >= 0)
    {
        glUniform1f(pointSizeLoc, 1.0F);
    }

    const GLint intensityLoc = m_shader.uniformLocation("uIntensityScale");
    if (intensityLoc >= 0)
    {
        glUniform1f(intensityLoc, 1.0F);
    }

    for (const auto& entry : m_fovBySensor)
    {
        const int sensorId = entry.first;
        const FovDescriptor& fov = entry.second;
        const float range = m_fovRangeOverride.count(sensorId) > 0
                                ? m_fovRangeOverride.at(sensorId)
                                : fov.maximumRange;
        if (range <= 0.0F || fov.horizontalFovRad <= 0.0F)
        {
            continue;
        }

        const float startAngle = -0.5F * fov.horizontalFovRad;
        const float step = fov.horizontalFovRad / static_cast<float>(kFovArcPointCount - 1);
        std::vector<Vertex> vertices;
        vertices.reserve(kFovArcPointCount + 1);

        for (int i = 0; i < kFovArcPointCount; ++i)
        {
            const float localAngle = startAngle + step * static_cast<float>(i);
            const float angle = fov.boresightAngleRad + fov.azimuthPolarity * localAngle;
            const float x = fov.sensorLateral + range * std::sin(angle);
            const float y = fov.sensorLongitudinal + range * std::cos(angle);
            vertices.push_back({glm::vec3(x, y, 0.0F), m_fovAlpha});
        }
        vertices.push_back({glm::vec3(fov.sensorLateral, fov.sensorLongitudinal, 0.0F), m_fovAlpha});

        const glm::vec3 color = colorForSensor(sensorId);
        const GLint colorLoc = m_shader.uniformLocation("uBaseColor");
        if (colorLoc >= 0)
        {
            glUniform3f(colorLoc, color.r, color.g, color.b);
        }

        glBindVertexArray(m_fovVao);
        glBindBuffer(GL_ARRAY_BUFFER, m_fovVbo);
        glBufferData(GL_ARRAY_BUFFER,
                     vertices.size() * sizeof(Vertex),
                     vertices.data(),
                     GL_DYNAMIC_DRAW);
        glLineWidth(1.2F);
        glDrawArrays(GL_LINE_LOOP, 0, static_cast<GLsizei>(vertices.size()));
        glBindVertexArray(0);
    }
    glLineWidth(1.0F);
}

bool RadarVisualizer::windowShouldClose() const
{
    return m_window == nullptr || glfwWindowShouldClose(m_window);
}

float RadarVisualizer::frameSpeedScale() const
{
    return std::max(0.01F, m_replaySpeed);
}

std::size_t RadarVisualizer::mapSegmentCount() const
{
    const int clamped = std::clamp(m_mapSegmentCount, kMapSegmentMin, kMapSegmentMax);
    return static_cast<std::size_t>(clamped);
}

} // namespace visualization
