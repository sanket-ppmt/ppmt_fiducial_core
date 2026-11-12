#include "synth.hpp"

#include <cmath>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace ppmt::fiducial::synth
{
namespace
{

constexpr int kSuperSample = 4;    // render at 4x, then area-downsample (anti-aliasing)
constexpr int kCellPx = 40;        // texture pixels per tag cell
constexpr int kMarkerCells = 8;    // 6x6 data bits + 1-cell black border on each side
constexpr int kLabelCells = 10;    // + 1-cell white margin on each side

double deg2rad(double d) { return d * CV_PI / 180.0; }

// Pose of the tag frame in the world frame.
void tagToWorld(const TagSpec & tag, cv::Matx33d & R, cv::Vec3d & t)
{
  const double c = std::cos(deg2rad(tag.yaw_deg)), s = std::sin(deg2rad(tag.yaw_deg));
  // Columns: tag x (its right), tag y (its top), tag z (up).
  R = cv::Matx33d(s, c, 0, -c, s, 0, 0, 0, 1);
  t = cv::Vec3d(tag.x_m, tag.y_m, 0.0);
}

cv::Point2d project(const cv::Matx33d & K, const cv::Vec3d & p)
{
  return {K(0, 0) * p[0] / p[2] + K(0, 2), K(1, 1) * p[1] / p[2] + K(1, 2)};
}

// Intrinsics for the supersampled canvas, keeping the OpenCV pixel-centre convention.
cv::Matx33d scaledK(const CameraSpec & cam, int s)
{
  return {cam.fx * s, 0, (cam.cx + 0.5) * s - 0.5, 0, cam.fy * s, (cam.cy + 0.5) * s - 0.5, 0, 0, 1};
}

cv::Mat makeFloor(cv::Size size, cv::RNG & rng)
{
  cv::Mat coarse(size.height / 48 + 2, size.width / 48 + 2, CV_8UC3);
  rng.fill(coarse, cv::RNG::UNIFORM, cv::Scalar(120, 125, 128), cv::Scalar(160, 165, 168));
  cv::Mat floor;
  cv::resize(coarse, floor, size, 0, 0, cv::INTER_CUBIC);
  return floor;
}

cv::Mat makeLabel(int id)
{
  const auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
  cv::Mat marker;
  cv::aruco::drawMarker(dict, id, kMarkerCells * kCellPx, marker, 1);
  cv::Mat label(kLabelCells * kCellPx, kLabelCells * kCellPx, CV_8UC1, cv::Scalar(235));
  marker.copyTo(label(cv::Rect(kCellPx, kCellPx, marker.cols, marker.rows)));
  cv::Mat bgr;
  cv::cvtColor(label, bgr, cv::COLOR_GRAY2BGR);
  return bgr;
}

// Draw a planar texture whose four texture corners map to four world-frame points.
void drawPlanar(
  cv::Mat & canvas, const cv::Mat & texture, const cv::Matx33d & K, const cv::Matx33d & Rcw,
  const cv::Vec3d & tcw, const std::vector<cv::Vec3d> & world_corners)
{
  const float w = static_cast<float>(texture.cols) - 0.5f;
  const float h = static_cast<float>(texture.rows) - 0.5f;
  const std::vector<cv::Point2f> src = {{-0.5f, -0.5f}, {w, -0.5f}, {w, h}, {-0.5f, h}};
  std::vector<cv::Point2f> dst;
  for (const auto & p : world_corners) dst.emplace_back(project(K, Rcw * p + tcw));
  const cv::Mat H = cv::getPerspectiveTransform(src, dst);
  cv::Mat warped, mask;
  cv::warpPerspective(texture, warped, H, canvas.size(), cv::INTER_LINEAR);
  cv::warpPerspective(
    cv::Mat(texture.size(), CV_8UC1, cv::Scalar(255)), mask, H, canvas.size(), cv::INTER_NEAREST);
  warped.copyTo(canvas, mask);
}

}  // namespace

void worldToCamera(const CameraSpec & cam, cv::Matx33d & R, cv::Vec3d & t)
{
  // Level camera looking along world +x: cam x = -world y, cam y = -world z, cam z = world x.
  const cv::Matx33d level(0, -1, 0, 0, 0, -1, 1, 0, 0);
  const double c = std::cos(deg2rad(cam.pitch_deg)), s = std::sin(deg2rad(cam.pitch_deg));
  const cv::Matx33d pitch(1, 0, 0, 0, c, -s, 0, s, c);  // tilt the view down about cam x
  R = pitch * level;
  t = -(R * cv::Vec3d(0.0, 0.0, cam.height_m));
}

TagTruth tagTruth(const CameraSpec & cam, const TagSpec & tag)
{
  cv::Matx33d Rcw, Rwt;
  cv::Vec3d tcw, twt;
  worldToCamera(cam, Rcw, tcw);
  tagToWorld(tag, Rwt, twt);
  const cv::Matx33d Rct = Rcw * Rwt;
  const cv::Vec3d tct = Rcw * twt + tcw;

  TagTruth truth;
  truth.id = tag.id;
  truth.size_m = tag.size_m;
  cv::Rodrigues(Rct, truth.rvec);
  truth.tvec = tct;

  const double s = tag.size_m / 2.0;
  const std::vector<cv::Vec3d> corners = {{-s, s, 0}, {s, s, 0}, {s, -s, 0}, {-s, -s, 0}};
  truth.fully_visible = true;
  for (const auto & c : corners) {
    const cv::Point2d px = project(cam.K(), Rct * c + tct);
    truth.corners.push_back(px);
    truth.fully_visible &= px.x >= 0 && px.y >= 0 && px.x <= cam.width - 1 && px.y <= cam.height - 1;
  }
  return truth;
}

Scene render(const SceneSpec & spec)
{
  const CameraSpec & cam = spec.camera;
  cv::RNG rng(spec.seed);
  const cv::Size big(cam.width * kSuperSample, cam.height * kSuperSample);
  const cv::Matx33d Kbig = scaledK(cam, kSuperSample);
  cv::Matx33d Rcw;
  cv::Vec3d tcw;
  worldToCamera(cam, Rcw, tcw);

  cv::Mat canvas = makeFloor(big, rng);

  // Red tape along world x, from just ahead of the camera to 4 m out.
  const double hw = spec.tape_width_m / 2.0;
  std::vector<cv::Point> tape;
  for (const cv::Vec3d & p : {cv::Vec3d(0.05, hw, 0), cv::Vec3d(4.0, hw, 0), cv::Vec3d(4.0, -hw, 0),
                              cv::Vec3d(0.05, -hw, 0)}) {
    tape.emplace_back(project(Kbig, Rcw * p + tcw));
  }
  cv::fillConvexPoly(canvas, tape, cv::Scalar(40, 38, 190), cv::LINE_AA);

  Scene scene;
  for (const auto & tag : spec.tags) {
    cv::Matx33d Rwt;
    cv::Vec3d twt;
    tagToWorld(tag, Rwt, twt);
    const double L = tag.size_m * kLabelCells / kMarkerCells / 2.0;  // half label edge
    std::vector<cv::Vec3d> label_world;
    for (const cv::Vec3d & p : {cv::Vec3d(-L, L, 0), cv::Vec3d(L, L, 0), cv::Vec3d(L, -L, 0),
                                cv::Vec3d(-L, -L, 0)}) {
      label_world.push_back(Rwt * p + twt);
    }
    drawPlanar(canvas, makeLabel(tag.id), Kbig, Rcw, tcw, label_world);
    scene.truth.push_back(tagTruth(cam, tag));
  }

  cv::resize(canvas, scene.image, cv::Size(cam.width, cam.height), 0, 0, cv::INTER_AREA);
  if (spec.blur_sigma_px > 0) {
    cv::GaussianBlur(scene.image, scene.image, cv::Size(0, 0), spec.blur_sigma_px);
  }
  if (spec.noise_sigma > 0) {
    cv::Mat noise(scene.image.size(), CV_16SC3);
    rng.fill(noise, cv::RNG::NORMAL, 0, spec.noise_sigma);
    cv::Mat img16;
    scene.image.convertTo(img16, CV_16SC3);
    img16 += noise;
    img16.convertTo(scene.image, CV_8UC3);  // saturates to [0, 255]
  }
  return scene;
}

}  // namespace ppmt::fiducial::synth
