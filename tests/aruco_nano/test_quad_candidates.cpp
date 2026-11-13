// Stage 4 (quad candidates): identical to the original aruco_nano (Gate A), compared directly
// on the same contours: same candidates, same corners exactly, same contour indices.
#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "oracle_stage_access.hpp"
#include "ppmt_fiducial_core/algorithms/aruco_nano/quad_candidates.hpp"

namespace ppmt::fiducial::aruco_nano
{

namespace
{

using Contours = std::vector<std::vector<cv::Point>>;

const std::filesystem::path sample_directory =
  std::filesystem::path(PPMT_DATA_DIR) / "sample" / "synthetic";

// Contours of a binary image from the original tracer: the stage's input.
Contours originalContours(const cv::Mat & binary_image,
                          const ::aruco_nano::DetectorParameters & original_parameters)
{
  cv::Mat traced_image = binary_image.clone();
  return ::aruco_nano::MarkerDetector::visitedAwareTracingContour(
    traced_image, 4 * static_cast<std::size_t>(original_parameters.minSize),
    original_parameters.maxTimesRevisited);
}

// Runs ours and the original on the same contours; returns the number of candidates.
std::size_t expectSameAsOriginal(const Contours & contours, int min_quad_side_px,
                                 const std::string & label)
{
  ::aruco_nano::DetectorParameters original_parameters;
  original_parameters.minSize = min_quad_side_px;
  std::vector<::aruco_nano::Marker> original_candidates;
  std::vector<int> original_contour_indices;
  oracle::findOriginalQuadCandidates(contours, original_parameters, original_candidates,
                                     original_contour_indices);

  QuadCandidateFinder quad_candidate_finder;
  std::vector<QuadCandidate> quad_candidates;
  quad_candidate_finder.find(contours, min_quad_side_px, quad_candidates);

  EXPECT_EQ(quad_candidates.size(), original_candidates.size()) << label;
  const std::size_t common = std::min(quad_candidates.size(), original_candidates.size());
  for (std::size_t index = 0; index < common; ++index) {
    EXPECT_EQ(quad_candidates[index].contour_index, original_contour_indices[index])
      << label << " candidate " << index;
    for (int corner = 0; corner < 4; ++corner) {
      EXPECT_EQ(quad_candidates[index].corners[corner], original_candidates[index][corner])
        << label << " candidate " << index << " corner " << corner;
    }
  }
  return quad_candidates.size();
}

}  // namespace

// Gate A on every synthetic scene, from the original's threshold and tracer output.
TEST(QuadCandidates, MatchesOriginalOnSyntheticScenes)
{
  const ::aruco_nano::DetectorParameters original_parameters;
  int compared_scenes = 0;
  for (const auto & entry : std::filesystem::directory_iterator(sample_directory)) {
    if (entry.path().extension() != ".png") {
      continue;
    }
    cv::Mat gray_image;
    cv::cvtColor(cv::imread(entry.path().string(), cv::IMREAD_COLOR), gray_image,
                 cv::COLOR_BGR2GRAY);
    const Contours contours = originalContours(
      oracle::computeOriginalThreshold(gray_image, original_parameters), original_parameters);
    const std::size_t candidate_count = expectSameAsOriginal(
      contours, original_parameters.minSize, entry.path().filename().string());
    EXPECT_GT(candidate_count, 0U) << entry.path().filename() << ": scene has tags";
    ++compared_scenes;
  }
  EXPECT_GT(compared_scenes, 0) << "no scenes in " << sample_directory << ", run `make test`";
}

// Gate A on many random rotated rectangles and triangles (fixed seed): quads of every
// orientation and size, plus shapes that must be rejected; strict and loose side limits.
TEST(QuadCandidates, MatchesOriginalOnRandomShapes)
{
  cv::RNG random_generator(4);
  const ::aruco_nano::DetectorParameters original_parameters;
  std::size_t total_candidates = 0;
  for (int image_index = 0; image_index < 20; ++image_index) {
    cv::Mat binary_image(240, 320, CV_8UC1, cv::Scalar(0));
    for (int shape = 0; shape < 12; ++shape) {
      const cv::Point2f centre(random_generator.uniform(20.F, 300.F),
                               random_generator.uniform(20.F, 220.F));
      std::vector<cv::Point> polygon;
      if (shape % 4 == 3) {  // triangle: must never become a quad
        for (int corner = 0; corner < 3; ++corner) {
          polygon.emplace_back(centre + cv::Point2f(random_generator.uniform(-25.F, 25.F),
                                                    random_generator.uniform(-25.F, 25.F)));
        }
      } else {
        const cv::RotatedRect rectangle(
          centre, cv::Size2f(random_generator.uniform(4.F, 60.F), random_generator.uniform(4.F, 60.F)),
          random_generator.uniform(0.F, 180.F));
        cv::Point2f corners[4];
        rectangle.points(corners);
        for (const cv::Point2f & corner : corners) {
          polygon.emplace_back(corner);
        }
      }
      cv::fillConvexPoly(binary_image, polygon, cv::Scalar(255));
    }
    const Contours contours = originalContours(binary_image, original_parameters);
    for (const int min_quad_side_px : {3, 10, 25}) {
      std::ostringstream label;
      label << "image " << image_index << " min side " << min_quad_side_px;
      total_candidates += expectSameAsOriginal(contours, min_quad_side_px, label.str());
    }
  }
  EXPECT_GT(total_candidates, 100U) << "random shapes produced too few quads to be a real test";
}

}  // namespace ppmt::fiducial::aruco_nano
