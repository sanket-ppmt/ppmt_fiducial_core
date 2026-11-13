// Stage 3 (contour tracer): identical to the original aruco_nano (Gate A), compared directly
// on the same binary image: same contours, same points, same marked image. The row scans
// must find the same index as std::find_if.
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
#include "ppmt_fiducial_core/algorithms/aruco_nano/contour_tracer.hpp"
#include "ppmt_fiducial_core/algorithms/aruco_nano/row_scan.hpp"

namespace ppmt::fiducial::aruco_nano
{

namespace
{

using Contours = std::vector<std::vector<cv::Point>>;

const std::filesystem::path sample_directory =
  std::filesystem::path(PPMT_DATA_DIR) / "sample" / "synthetic";

// "" if equal; otherwise where the first difference is.
std::string describeFirstDifference(const Contours & ours, const Contours & original)
{
  std::ostringstream description;
  if (ours.size() != original.size()) {
    description << "contour count " << ours.size() << " vs original " << original.size() << "; ";
  }
  const std::size_t common = std::min(ours.size(), original.size());
  for (std::size_t index = 0; index < common; ++index) {
    if (ours[index] != original[index]) {
      description << "first differing contour " << index << " (" << ours[index].size()
                  << " vs " << original[index].size() << " points)";
      break;
    }
  }
  return description.str();
}

// Runs ours and the original on copies of the same binary image and compares everything:
// the contours, and the image after tracing (border zeroed, traced pixels marked).
void expectSameAsOriginal(const cv::Mat & binary_image, int min_contour_length,
                          float max_revisit_fraction, const std::string & label)
{
  cv::Mat our_image = binary_image.clone();
  cv::Mat original_image = binary_image.clone();

  ContourTracer contour_tracer;
  Contours our_contours;
  contour_tracer.trace(our_image, min_contour_length, max_revisit_fraction, our_contours);
  const Contours original_contours = ::aruco_nano::MarkerDetector::visitedAwareTracingContour(
    original_image, static_cast<std::size_t>(min_contour_length), max_revisit_fraction);

  EXPECT_EQ(describeFirstDifference(our_contours, original_contours), "") << label;
  EXPECT_EQ(cv::countNonZero(our_image != original_image), 0) << label << ": marked image";
}

}  // namespace

// Row scans (whichever implementation is built) equal std::find_if for every start, density
// and row length, including lengths shorter than one SIMD vector.
TEST(ContourTracer, RowScansMatchStdFindIf)
{
  cv::RNG random_generator(7);
  for (int length = 1; length <= 100; ++length) {
    for (const double nonzero_probability : {0.0, 0.02, 0.5, 0.98, 1.0}) {
      std::vector<std::uint8_t> row(length);
      for (auto & pixel : row) {
        pixel = random_generator.uniform(0.0, 1.0) < nonzero_probability ? 255 : 0;
      }
      for (int start = 0; start <= length; ++start) {
        const auto first_nonzero =
          std::find_if(row.begin() + start, row.end(), [](std::uint8_t value) { return value != 0; });
        const auto first_zero =
          std::find_if(row.begin() + start, row.end(), [](std::uint8_t value) { return value == 0; });
        ASSERT_EQ(findNextNonZero(row.data(), start, length), first_nonzero - row.begin())
          << "length " << length << " start " << start;
        ASSERT_EQ(findNextZero(row.data(), start, length), first_zero - row.begin())
          << "length " << length << " start " << start;
      }
    }
  }
}

// Gate A on every synthetic scene, from the original's threshold image (the stage's input).
TEST(ContourTracer, MatchesOriginalOnSyntheticScenes)
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
    const cv::Mat binary_image = oracle::computeOriginalThreshold(gray_image, original_parameters);
    expectSameAsOriginal(binary_image, 4 * original_parameters.minSize,
                         original_parameters.maxTimesRevisited,
                         entry.path().filename().string());
    ++compared_scenes;
  }
  EXPECT_GT(compared_scenes, 0) << "no scenes in " << sample_directory << ", run `make test`";
}

// Gate A on random binary noise (fixed seed): many touching blobs and revisits, sizes that
// are not SIMD multiples, and both strict and loose filter settings.
TEST(ContourTracer, MatchesOriginalOnNoiseImages)
{
  cv::RNG random_generator(99);
  for (const cv::Size & size : {cv::Size(5, 4), cv::Size(33, 40), cv::Size(131, 200)}) {
    for (const double white_probability : {0.1, 0.3, 0.5, 0.7}) {
      cv::Mat binary_image(size, CV_8UC1);
      for (int row = 0; row < size.height; ++row) {
        for (int column = 0; column < size.width; ++column) {
          binary_image.at<std::uint8_t>(row, column) =
            random_generator.uniform(0.0, 1.0) < white_probability ? 255 : 0;
        }
      }
      for (const int min_contour_length : {1, 40}) {
        for (const float max_revisit_fraction : {0.05F, 1.0F}) {
          std::ostringstream label;
          label << size << " white " << white_probability << " min length "
                << min_contour_length << " revisit " << max_revisit_fraction;
          expectSameAsOriginal(binary_image, min_contour_length, max_revisit_fraction,
                               label.str());
        }
      }
    }
  }
}

}  // namespace ppmt::fiducial::aruco_nano
