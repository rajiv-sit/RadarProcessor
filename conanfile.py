from conan import ConanFile
from conan.tools.cmake import CMakeDeps, CMakeToolchain, CMake, cmake_layout


class RadarProcessorConan(ConanFile):
    name = "radarprocessor"
    version = "0.1.0"
    license = "MIT"
    description = "Orbiting radar replay with GLFW, ImGui, and OpenGL"
    settings = "os", "arch", "compiler", "build_type"
    options = {
        "shared": [True, False],
        "fPIC": [True, False],
    }
    default_options = {
        "shared": False,
        "fPIC": True,
        "imgui/*:glfw": True,
        "imgui/*:opengl3": True,
    }
    exports_sources = (
        "CMakeLists.txt",
        "test/*",
        "radar/*",
        "radar_core/*",
        "utility/*",
        "visualization/*",
        "shaders/*",
        "data/*",
        "assets/*",
        "run_debug.bat",
        "run_release.bat",
        "architecture/*",
    )

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def configure(self):
        self.options["imgui/*"].glfw = True
        self.options["imgui/*"].opengl3 = True

    def requirements(self):
        self.requires("eigen/3.4.0")
        self.requires("glfw/3.4")
        self.requires("glew/2.2.0")
        self.requires("glm/cci.20230113")
        self.requires("imgui/cci.20230105+1.89.2.docking")
        self.requires("opengl/system")
        self.requires("gtest/1.13.0")

    def build_requirements(self):
        self.tool_requires("cmake/3.30.1")

    def layout(self):
        cmake_layout(self)

    def generate(self):
        tc = CMakeToolchain(self)
        tc.generate()
        deps = CMakeDeps(self)
        deps.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
