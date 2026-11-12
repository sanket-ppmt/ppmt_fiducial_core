# ppmt_fiducial_core

ROS-independent C++ library for fast AprilTag/ArUco fiducial detection with switchable algorithms.
It will replace the detection step of `peppermint_os/fiducial_detection/apriltag_ros2` (today: the
AprilTag C library) for Peppermint's material-handling robots. A thin ROS node is added later.

## How we work (owner's rules — follow them)

- Small steps. Before writing code, explain the step and wait for agreement. After it, explain
  what each file does. Never drop hundreds of lines of unreviewed code at once.
- Tests are intentional and test core behaviour. No test sprawl. Temporary tests are marked
  `TEMPORARY` in the file and the CMake line, and removed later.
- No `#ifdef`s scattered around. Platform/OpenCV-version differences live in one place each.
- Work on branch `feature/nano_aruco`.
- Code style: file names `snake_case.hpp/.cpp`; classes/structs `CamelCase`; functions
  `camelCase`; variables and public struct fields `snake_case`; private members end in `_`.
  No abbreviations or prefixes in names (`FiducialDetectorInterface`, not `IMarkerDetector`;
  `FiducialDetectionResult`, not `Result`). Clean, specific names, no slop.
  Always `FiducialDetector...`, never bare `Detector`; "algorithm", never "backend".
  Config-typed fields end in `_config` (`preprocess_config`, `algorithm_config`, ...).
- Comments: short, point-wise, precise. No paragraphs.
- All library headers live in `include/ppmt_fiducial_core/`; `src/` holds only `.cpp` files.
- Configs (incl. their enums) live in `include/ppmt_fiducial_core/configs/`; all other
  types/structs/enums (detections, results) in `include/ppmt_fiducial_core/data_types/`.
- Build: `make build` / `make test` (also regenerates data) / `make synthetic_markers` / `make oracle`.

## Design decisions

- **Core library**: C++17 + OpenCV (core, later imgproc/calib3d). No ROS, no YAML, no file I/O.
- **Configuration**: plain structs, one file each in `configs/`. `FiducialDetectorConfig` holds the
  shared configs plus `algorithm_config`, a `std::variant` of per-algorithm configs (the held
  type is the choice of algorithm), with `validate()`. The ROS node fills them from ROS params (Peppermint reads robot_params YAML in
  the launch file and passes params; nodes never open config YAML). Tools/tests build structs in code.
- **Two levels**: `FiducialPipeline` (concrete; what the ROS node uses) runs the stages and owns
  one algorithm. `createFiducialDetector(const FiducialDetectorConfig&) -> std::unique_ptr<FiducialDetectorInterface>`
  builds only the algorithm, `std::visit` over `algorithm_config`. Algorithms: `ArucoNano` (first), `AprilTagLibrary`
  (later; the AprilTag C library the current node uses). No OpenCV ArUco algorithm.
- **Errors**: `validate()` returns the problems as text (empty = valid). The factory and the
  pipeline constructor throw on invalid config, with messages naming the field, its value and
  the allowed range. Per-frame `detect()` never throws; frame problems go into the result.
- **Inputs/outputs**: pipeline accepts `bgr8` or `mono8` only (anything else rejected at
  startup); algorithm takes `CV_8UC1`. Caller-owned outputs passed by reference and reused per
  frame. Corners are `cv::Point2f` (as aruco_nano, so Gate A compares exactly).
- **Rejections, two levels**: counts per `RejectionReason` always; the list of rejected
  candidates with corners only behind a debug flag (it allocates).
- **Quality**: provisional definition, refined later. Pose types come in step 7, not before.
- **Narrow algorithm interface**: an algorithm only finds tags (id, 4 corners, quality) in a grayscale
  image, with an optional ROI argument (unused until tracking). Preprocess, filtering, pose and
  tracking are separate shared components:
  `image -> Preprocess -> Detector(aruco_nano|apriltag_library) -> Filter -> Pose -> (later) Tracker -> FiducialDetectionResult`
- **ArucoNano algorithm owns its 36h11 dictionary** (embedded code table), so it needs no OpenCV aruco
  module. SIMD row-skip sits behind one function with a scalar fallback; tests check they agree.
- **Determinism**: retry jitter uses a seeded RNG owned by the detector.
- **Performance hygiene**: detector object reuses buffers (no per-frame allocation), no static
  state, one instance per camera.
- **Observability**: result carries per-stage timings, funnel counts (contours, quads, decoded)
  and rejection reasons. Profiling is compiled out unless enabled. The library never writes files.

## Acceptance gates (tolerances are starting points)

- **A. Nano port**: vs original `aruco_nano.h` (github.com/rmsalinas/aruco_nano, commit
  `c99989bcab795ffa65d9e12df9de15f27677a835`): threshold image bit-identical, contours and quads
  identical, same IDs, corners within 0.001 px. The original lives only in `tools/oracle/`, never
  linked into the library. Its default dict differs; we always use `APRILTAG_36h11`.
- **B. AprilTagLibrary algorithm (later)**: same library as the current node, so with identical
  settings: same IDs and corners as the node on every frame.
- **C. Pose**: vs current node with identical settings (CLAHE on, `iterative` PnP, zero
  distortion): translation median <= 1 cm, p95 <= 2 cm; rotation median <= 1 deg, p95 <= 3 deg;
  lateral y median <= 5 mm.
- "Better" is measured against ground truth: pose accuracy and jitter, recall, detection range,
  latency (p50/p99 at 854x534 on an i7-1360P), CPU. Zero false positives always.

## Conventions (must match the current ROS node)

- Pixels: OpenCV convention, x right, y down, pixel centres at integer coordinates.
- Tag size = outer black border edge (8x8 cells for 36h11), white margin excluded.
- Corner order: TL, TR, BR, BL clockwise as printed; corner 0 fixed by the decoded rotation.
- Tag frame: origin at centre, x right, y to the tag's top, z out of the tag.
- PnP object points: (-s,+s), (+s,+s), (+s,-s), (-s,-s), z = 0, s = size/2.
- Pose output: tag in camera optical frame (x right, y down, z forward), metres.

## Context from peppermint_os

- Current detector: `fiducial_detection/apriltag_ros2/src/apriltag_detector.cpp`
  (`apriltag_detector_detect`; OpenCV only for cvtColor, CLAHE, solvePnP).
- Consumers: vision line follower (stops on tag id using pose y), tag-based localization, docking
  via tag TF frames. ROS output must stay a drop-in `apriltag_msgs/AprilTagDetectionArray`,
  published every frame (empty when nothing found).
- Robots: MP020_Gen2, UAMR_Gen1, ABOPT_Gen1. Tag sizes per id: ids 0-2 = 0.08 m, ids 3-18 = 0.071 m.
- Inputs: OAK-1 RGB preview 854x534 (likely bgr8, not rectified) for line following; Orbbec colour
  1280x720 on ABOPT. Preview camera_info may not account for the preview crop (open question).
- Full decisions note (Claude Docs): https://claude.ai/artifact/KvnGJsvkd4HSa6XYqd3Rn4

## Layout

- `include/ppmt_fiducial_core/`, `src/` — the library; `configs/` = config structs, `data_types/` = other types
- `tools/synth/` — synthetic scene renderer (`ppmt_fiducial_synth`) + `ppmt_fiducial_synth_generate`.
  Renders tags on red tape with true camera geometry; writes PNG + YAML ground truth
  (K, per tag: id, size, corners, rvec, tvec). Uses OpenCV aruco/imgcodecs (tools only).
- `tools/oracle/` — original aruco_nano (pinned, unmodified, `third_party/`) + `ppmt_fiducial_oracle`;
  `compat/` is the single place handling OpenCV < 4.7 (system has 4.5.4).
- `data/sample/synthetic/` — generated scenes; `data/oracle/synthetic/` — oracle output
  (`<name>.yaml` ids + corners, `<name>.png` detected boxes drawn).
- `tests/` — `test_skeleton` (TEMPORARY), `test_synth` (generator geometry and truth checks),
  `test_oracle` (oracle has zero false positives, corners within 1 px of truth),
  `test_configs` (every config loads by name, validate messages, algorithm/encoding selection);
  `map_config_source.hpp` = test config source

## Plan

1. [done] Skeleton: CMake, GTest, version().
2. [done] Synthetic generator + 3 tests.
3. [done] Oracle harness: IDs/corners on synthetic scenes (per-stage outputs added per Nano stage).
   All 4 scenes detected (`far_small` at 1.10 m; aruco_nano's limit there is ~1.2-1.3 m); corner order matches ours.
4. [next] Types/interfaces: TagDetection, FiducialDetectorConfig + validate(), FiducialDetectorInterface,
   factory, FiducialDetectionResult, FiducialPipeline (headers reviewed one at a time).
5. Nano, one stage at a time (each: module + tests + oracle diff + timer): dictionary ->
   grayscale -> threshold -> contour tracer -> quads -> corner pre-refine -> bit sampling ->
   decode/retries/dedupe -> final refinement.
6. End-to-end vs oracle and ground truth; benchmark; profiling (perf/Tracy).
7. Pose module (shared), Gate C. 8. ROS node adapter. Later: AprilTagLibrary algorithm, tracking/ROI.
