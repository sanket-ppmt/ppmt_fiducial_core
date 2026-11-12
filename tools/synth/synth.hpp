#pragma once
// Synthetic scene renderer for tests and benchmarks. NOT part of the core library.
//
// Geometry: a pinhole camera above a flat floor, pitched down, looking forward.
//   World frame: x forward, y left, z up, floor at z = 0, camera at (0, 0, height).
//   Tag frame: origin at the tag centre, x to the tag's right, y to the tag's top,
//   z out of the tag (up from the floor). Yaw 0 = tag top points forward (+x world).
// Pixel convention: OpenCV (pixel centres at integer coordinates).

#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

namespace ppmt::fiducial::synth
{

struct CameraSpec
{
  int width = 854;            // OAK-1 preview used for line following
  int height = 534;
  double fx = 620.0;          // approximate; replaced by real calibration later
  double fy = 620.0;
  double cx = 426.5;
  double cy = 266.5;
  double height_m = 0.60;     // camera height above the floor
  double pitch_deg = 45.0;    // downward tilt from horizontal

  cv::Matx33d K() const { return {fx, 0, cx, 0, fy, cy, 0, 0, 1}; }
};

struct TagSpec
{
  int id = 0;                 // AprilTag 36h11 id
  double size_m = 0.071;      // outer black border edge
  double x_m = 0.6;           // tag centre on the floor, world frame
  double y_m = 0.0;
  double yaw_deg = 0.0;
};

struct SceneSpec
{
  CameraSpec camera;
  std::vector<TagSpec> tags;
  double tape_width_m = 0.10;   // red tape under the tags, along world x through y = 0
  double blur_sigma_px = 0.6;
  double noise_sigma = 2.0;
  std::uint64_t seed = 1;       // floor texture and noise
};

struct TagTruth
{
  int id = -1;
  double size_m = 0.0;
  std::vector<cv::Point2d> corners;  // TL, TR, BR, BL of the black border, in pixels
  cv::Vec3d rvec;                    // tag frame -> camera optical frame
  cv::Vec3d tvec;                    // metres
  bool fully_visible = false;        // all 4 corners inside the image
};

struct Scene
{
  cv::Mat image;  // CV_8UC3, BGR
  std::vector<TagTruth> truth;
};

// Pose of the world frame in the camera optical frame (x right, y down, z forward).
void worldToCamera(const CameraSpec & cam, cv::Matx33d & R, cv::Vec3d & t);

// Ground truth for one tag (no rendering).
TagTruth tagTruth(const CameraSpec & cam, const TagSpec & tag);

// Render the scene and its ground truth. Deterministic for a given spec.
Scene render(const SceneSpec & spec);

}  // namespace ppmt::fiducial::synth
