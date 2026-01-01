# RadarProcessor

RadarProcessor is an offline radar replay and inspection framework that mirrors the `LiDARProcessor` architecture but focuses entirely on replaying real automotive radar captures: four corner sensors plus front short-range and long-range radars. All data—including detections, tracks, vehicle contour, and the minimal configuration keys—has been stripped down to what the project needs, enabling safe publication while still supporting realistic visualization and mapping in vehicle coordinate space (VCS).

## Architecture summary
- `architecture/architecture.md` explains why the project exists, the module layout, the producer/consumer engine–visualizer loop, and the free-space mapping pipeline (segment-based radial mapping plus the B-spline boundary). An enhanced architecture diagram there shows how the data flows from the text sensors through the mapping engine to the visualizer.
- The code follows a double-buffering pattern in `radar::RadarEngine`, observer-like updates in `visualization::RadarVisualizer`, and builder-style helpers in the mapping stack (segments + SPLINTER cleanup).

## Prerequisites
1. **Visual Studio 2022** (C++ workload) with the Windows 10/11 SDK.
2. **Conan 2.x** installed globally (the repo relies on the `conanfile.py` recipe for Eigen, GLFW, GLEW, GLM, ImGui, and GoogleTest).
3. `cmake` 3.30+ on the `PATH`.

## Build & run
1. Open a PowerShell window in the repo root.
2. Run `run_debug.bat` for a full development build. This script:
   - Removes `build/`.
   - Calls `conan install . --output-folder=build --build=missing --settings build_type=Debug`.
   - Configures CMake via the generated toolchain (CMake preset `conan-default` is also provided).
   - Builds `radarprocessor` plus the `radar_unit_tests` suite.
   - Copies `shaders/` and the `data/*.txt` captures into `build/Debug`.
   - Launches `radarprocessor.exe` with the three data files, so the visualizer renders detections, tracks, and mapping immediately.
3. Alternatively, run manually:
   ```bat
   conan install . --output-folder=build --build=missing --settings build_type=Debug
   cmake --preset conan-default
   cmake --build build/build --config Debug --target radarprocessor
   cmake --build build/build --config Debug --target radar_unit_tests
   ```

## Visualization & controls
- Launch `build/build/Debug/radarprocessor.exe` (or run via the script). The UI renders:
  - **Radar detections**: points colored by detection state (static/moving/ambiguous).
  - **Tracks**: polygonal tracks with color-coded motion state.
  - **Segment map**: radial lines drawn from the vehicle contour to each segment endpoint (updated every frame from `RadarVirtualSensorMapping`).
  - **B-spline boundary**: optional smooth boundary built from the same segments when `Show B-spline map` is checked.
 
    <img width="1914" height="1030" alt="image" src="https://github.com/user-attachments/assets/6171309c-533f-4408-ba4d-6abb879de39a" />
    Fig 1: Detection and track visualization

    <img width="1905" height="1025" alt="image" src="https://github.com/user-attachments/assets/6f2d0f29-88d1-42b7-af72-1f13b91242f0" />
    Fig 2: Radial segment map

   <img width="1907" height="1027" alt="image" src="https://github.com/user-attachments/assets/df4efda3-a6a6-4476-a1a0-179284f7093f" />
   Fig 3: B-spline free-space map
  
- ImGui controls let you adjust:
  - Point size, intensity scale, replay speed.
  - Map visibility toggles (segments, spline, vehicle contour).
  - Segment count and B-spline control point count via sliders—for rapidly exploring the trade-off between resolution and smoothness.

## Mapping details
- **Segment-based free-space map**: Each of the configurable radial segments originates at the vehicle contour (converted to VCS once at startup). Detections clip the maximum length, and tracks are represented as 2D rectangular footprints that intersect every segment they cover. The result is a 360° view of free space that updates in real time.
- **B-spline boundary**: The segment ring is resampled, optionally smoothed via SPLINTER (PSpline/none), and sampled into a closed polygon. The spline uses configurable control points and sample counts so you can balance fidelity versus smoothing.
- Both mapping phases run entirely in VCS to avoid repeated coordinate conversions.

## Expected outputs
- Launching `run_debug.bat` produces:
  - `build/Debug/radarprocessor.exe` plus shader/data resources for the visualizer.
  - `build/build/Debug/radar_unit_tests.exe` and `radarfactory_test` runs via CTest when you invoke `ctest --test-dir build/build -C Debug`.
- The visualizer window shows detections (points), tracks, segments, and spline boundaries simultaneously. Logs printed to the console include data root, vehicle config, and B-spline rebuilds.

Refer to `architecture/architecture.md` for the deeper “why/how” narrative, including an enhanced architecture diagram and design rationales. Enjoy replaying your radar sweeps! 
