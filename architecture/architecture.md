# RadarProcessor Architecture

## 1. Purpose and domain
- `RadarProcessor` documents, replays, and evaluates offline radar data captured from real automotive sensors: four corner radars plus the short-range and long-range front radars. The collected files hold only the detections, tracks, vehicle contour, and essential radar configuration parameters required to reproduce visualization and mapping, making the repo safe to publish while remaining faithful to the original coordinate conventions (VCS for radar data, ISO for vehicle contour).
- The repository does **not** implement new tracking algorithms; instead, it relies on precomputed detections/tracks and focuses on two complementary free-space mapping views: a radial segment map and a smoothed B-spline boundary.

## 2. Core design and patterns
- **Producer–consumer loop:** `radar::RadarEngine` (and the playback variant) double-buffers the incoming `RadarPoint` sweeps and pushes them into `visualization::RadarVisualizer`, keeping the render loop independent of data ingestion and allowing a 30 Hz throttle.
- **Observer-style visualization:** The visualizer exposes ImGui sliders to tune point size, intensity scale, replay speed, map toggles, and segment/spline counts. Each data update (`updateMapPoints`, `updateVehicleContour`, etc.) flips a dirty flag so the GL buffers refresh only when necessary.
- **Immutable configuration loaders:** `utility::VehicleConfig` and `radar::VehicleProfile` parse INI files once at startup, fill `VehicleParameters`, and supply contour + radar mount data to the engine/visualizer, ensuring deterministic mapping of coordinate frames.
- **Builder for mapping outputs:** The free-space mapping uses `radar::RadarVirtualSensorMapping` to compute segments and `visualization::RadarVisualizer::buildMapSplineBoundary` to resample control points (via SPLINTER). Both use resampling/build steps that resemble builder patterns, allowing slider-controlled segment counts or control-point counts without touching core mapping logic.

## 3. Data flow and modules
```
Data feeds (fourCornersfusedRadarDetections, fusedFrontRadarsDetections, fusedRadarTracks)
                         ↓
                   TextRadarSensor
                         ↓  (double-buffered BaseRadarSensor::PointCloud)
                      RadarEngine
                         ↓
        Visualizer + RadarVirtualSensorMapping + B-spline builder
```
- The radar files stream into `TextRadarSensor`, which strips metadata, builds `RadarPoint`s, and respects range/intensity filters before handing frames to the engine.
- `RadarEngine` both feeds the visualizer and routes detections to `RadarVirtualSensorMapping` so the map segments update on each frame; tracks are converted to vehicle-contour-aligned rectangles and traced against every radial segment.
- `RadarVirtualSensorMapping` spans 360° around the vehicle center, clips every segment with the nearest detection or vehicle contour, and exposes both a ring boundary and raw start/end segments to the visualizer.
- For the spline map, `buildMapSplineBoundary` resamples the ring, clamps control point counts (via sliders), and feeds the values into SPLINTER (with degree/knot choices and optional P-spline smoothing).

## 4. Free-space mapping focus
- **Segmented map**: A radial array of segments (configurable count) begins at the vehicle contour and stretches outward up to 120 m or the nearest detection. Tracks are treated as 2D polygons; their intersections with segments trim the free-space envelope accordingly so occlusions appear as short segments.
- **B-spline boundary**: The segmented ring is resampled into control points, optionally smoothed with SPLINTER, and sampled to produce a continuous boundary. Sliders allow the operator to edit the number of control points/samples to balance smoothness vs. fidelity.
- This two-pronged mapping emphasizes free space, not occupancy, and keeps all math in VCS, sidestepping ISO-to-VCS drift by converting the vehicle contour once at startup.

## 5. Architecture diagram (enhanced)
- The architecture diagram (see `architecture/visual/architecture.png`) distills the modules above: sensors → engine → mapping → visualizer. Key data paths are annotated with frame names (VCS for calculus, ISO for contour) and the diagram spotlights the segment map + B-spline pipeline.
