// Stage 2 (threshold): bit-identical to the original aruco_nano (Gate A), and marks exactly
// the pixels darker than their local mean by more than the offset.
#include <gtest/gtest.h>

#include <filesystem>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "ppmt_fiducial_core/algorithms/aruco_nano/threshold.hpp"
#include "ppmt_fiducial_core/configs/aruco_nano_config.hpp"

namespace ppmt::fiducial::aruco_nano
{

namespace
{
const std::filesystem::path sample_directory =
  std::filesystem::path(PPMT_DATA_DIR) / "sample" / "synthetic";
const std::filesystem::path oracle_directory =
  std::filesystem::path(PPMT_DATA_DIR) / "oracle" / "synthetic";
}  // namespace

// Gate A: every synthetic scene gives the oracle's binary image, pixel for pixel.
TEST(Threshold, MatchesOracleBitIdentical)
{
  const ArucoNanoConfig config;
  int compared_scenes = 0;
  for (const auto & entry : std::filesystem::directory_iterator(sample_directory)) {
    if (entry.path().extension() != ".png") {
      continue;
    }
    const std::string scene = entry.path().stem().string();
    const cv::Mat oracle_binary =
      cv::imread((oracle_directory / (scene + "_threshold.png")).string(), cv::IMREAD_UNCHANGED);
    ASSERT_FALSE(oracle_binary.empty()) << scene << ": missing oracle output, run `make oracle`";

    cv::Mat gray_image;
    cv::cvtColor(cv::imread(entry.path().string(), cv::IMREAD_COLOR), gray_image,
                 cv::COLOR_BGR2GRAY);
    cv::Mat binary_image;
    computeThreshold(gray_image, config.threshold_window_size, config.threshold_offset,
                     binary_image);

    ASSERT_EQ(binary_image.type(), oracle_binary.type()) << scene;
    EXPECT_EQ(cv::countNonZero(binary_image != oracle_binary), 0) << scene;
    ++compared_scenes;
  }
  EXPECT_GT(compared_scenes, 0);
}

// Dark square (50) on bright background (200): only the square's pixels are far enough
// below their 15x15 mean; bright pixels are above their mean and become 0.
TEST(Threshold, MarksPixelsDarkerThanLocalMeanByMoreThanOffset)
{
  cv::Mat gray_image(20, 20, CV_8UC1, cv::Scalar(200));
  const cv::Rect dark_square(7, 7, 6, 6);
  gray_image(dark_square).setTo(50);

  cv::Mat expected_binary(20, 20, CV_8UC1, cv::Scalar(0));
  expected_binary(dark_square).setTo(255);

  cv::Mat binary_image;
  computeThreshold(gray_image, 15, 3, binary_image);
  EXPECT_EQ(cv::countNonZero(binary_image != expected_binary), 0);

  // Uniform image: no pixel is below its mean.
  computeThreshold(cv::Mat(20, 20, CV_8UC1, cv::Scalar(120)), 15, 3, binary_image);
  EXPECT_EQ(cv::countNonZero(binary_image), 0);
}

}  // namespace ppmt::fiducial::aruco_nano
