// The synthetic generator is the reference every later comparison relies on, so these
// check that its geometry and its ground truth agree with the image it renders.
#include <gtest/gtest.h>

#include <cmath>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "synth.hpp"

namespace synth = ppmt::fiducial::synth;

// A floor point straight along the optical axis must land on the principal point.
TEST(Synth, CameraLooksDownAtPitchAngle)
{
  synth::CameraSpec cam;
  cv::Matx33d R;
  cv::Vec3d t;
  synth::worldToCamera(cam, R, t);
  const double ahead = cam.height_m / std::tan(cam.pitch_deg * CV_PI / 180.0);
  const cv::Vec3d p = R * cv::Vec3d(ahead, 0, 0) + t;
  ASSERT_GT(p[2], 0.0);
  EXPECT_NEAR(cam.fx * p[0] / p[2] + cam.cx, cam.cx, 1e-9);
  EXPECT_NEAR(cam.fy * p[1] / p[2] + cam.cy, cam.cy, 1e-9);
}

// The truth pose and the truth corners must describe the same tag.
TEST(Synth, TruthPoseMatchesTruthCorners)
{
  synth::CameraSpec cam;
  const synth::TagSpec tag{3, 0.071, 0.7, 0.05, 25.0};
  const auto truth = synth::tagTruth(cam, tag);
  ASSERT_TRUE(truth.fully_visible);

  const double s = tag.size_m / 2.0;
  const std::vector<cv::Point3d> obj = {{-s, s, 0}, {s, s, 0}, {s, -s, 0}, {-s, -s, 0}};
  cv::Vec3d rvec, tvec;
  ASSERT_TRUE(cv::solvePnP(obj, truth.corners, cam.K(), cv::noArray(), rvec, tvec, false,
                           cv::SOLVEPNP_IPPE_SQUARE));
  EXPECT_LT(cv::norm(tvec - truth.tvec), 1e-6);
  EXPECT_LT(cv::norm(rvec - truth.rvec), 1e-6);
}

// The rendered image must be dark just inside each truth corner (black border)
// and bright just outside it (white label margin).
TEST(Synth, RenderedBorderSitsOnTruthCorners)
{
  synth::SceneSpec spec;
  spec.noise_sigma = 0;
  spec.tags = {{3, 0.071, 0.6, 0.0, 0.0}};
  const auto scene = synth::render(spec);
  ASSERT_EQ(scene.truth.size(), 1u);
  cv::Mat gray;
  cv::cvtColor(scene.image, gray, cv::COLOR_BGR2GRAY);

  const auto & c = scene.truth[0].corners;
  const cv::Point2d centre = (c[0] + c[1] + c[2] + c[3]) / 4.0;
  for (const auto & corner : c) {
    const cv::Point2d dir = (centre - corner) / cv::norm(centre - corner);
    const cv::Point inside(cvRound(corner.x + 3 * dir.x), cvRound(corner.y + 3 * dir.y));
    const cv::Point outside(cvRound(corner.x - 3 * dir.x), cvRound(corner.y - 3 * dir.y));
    EXPECT_LT(gray.at<uchar>(inside), 80) << "corner " << corner;
    EXPECT_GT(gray.at<uchar>(outside), 150) << "corner " << corner;
  }
}
