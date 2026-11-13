// Stage 2 (threshold): bit-identical to the original aruco_nano (Gate A), compared directly
// on the same input, and marks exactly the pixels darker than their local mean by more
// than the offset.
#include <gtest/gtest.h>

#include <filesystem>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "oracle_stage_access.hpp"
#include "ppmt_fiducial_core/algorithms/aruco_nano/threshold.hpp"

namespace ppmt::fiducial::aruco_nano
{

namespace
{

const std::filesystem::path sample_directory =
  std::filesystem::path(PPMT_DATA_DIR) / "sample" / "synthetic";

// Our threshold and the original's on the same gray image; returns differing pixel count.
int countDifferenceFromOriginal(const cv::Mat & gray_image)
{
  const ::aruco_nano::DetectorParameters original_parameters;
  const cv::Mat original_binary =
    oracle::computeOriginalThreshold(gray_image, original_parameters);
  cv::Mat binary_image;
  computeThreshold(gray_image, original_parameters.boxFilterSize, original_parameters.thres,
                   binary_image);
  return cv::countNonZero(binary_image != original_binary);
}

}  // namespace

// Gate A on every synthetic scene.
TEST(Threshold, MatchesOriginalOnSyntheticScenes)
{
  int compared_scenes = 0;
  for (const auto & entry : std::filesystem::directory_iterator(sample_directory)) {
    if (entry.path().extension() != ".png") {
      continue;
    }
    cv::Mat gray_image;
    cv::cvtColor(cv::imread(entry.path().string(), cv::IMREAD_COLOR), gray_image,
                 cv::COLOR_BGR2GRAY);
    EXPECT_EQ(countDifferenceFromOriginal(gray_image), 0) << entry.path().filename();
    ++compared_scenes;
  }
  EXPECT_GT(compared_scenes, 0) << "no scenes in " << sample_directory << ", run `make test`";
}

// Gate A on random noise images (fixed seed), including sizes that are not SIMD multiples.
TEST(Threshold, MatchesOriginalOnNoiseImages)
{
  cv::RNG random_generator(2024);
  for (const cv::Size & size : {cv::Size(17, 9), cv::Size(97, 120), cv::Size(854, 534)}) {
    cv::Mat gray_image(size, CV_8UC1);
    random_generator.fill(gray_image, cv::RNG::UNIFORM, 0, 256);
    EXPECT_EQ(countDifferenceFromOriginal(gray_image), 0) << size;
  }
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
