// FiducialPipeline: rejects invalid configs at construction with every problem listed,
// reports bad frames through frame_status, and processes valid frames on every
// preprocess path. Tag filtering is tested once ArucoNano detects tags (step 5).
#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "ppmt_fiducial_core/fiducial_pipeline.hpp"

namespace ppmt::fiducial
{

namespace
{

cv::Mat grayImage() { return cv::Mat(48, 64, CV_8UC1, cv::Scalar(128)); }
cv::Mat bgrImage() { return cv::Mat(48, 64, CV_8UC3, cv::Scalar(30, 60, 90)); }

}  // namespace

TEST(FiducialPipeline, InvalidConfigThrowsListingEveryProblem)
{
  FiducialDetectorConfig config;
  config.preprocess_config.clahe_clip_limit = 0.0;
  std::get<ArucoNanoConfig>(config.algorithm_config).threshold_window_size = 14;

  try {
    FiducialPipeline pipeline(config);
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument & error) {
    EXPECT_EQ(std::string(error.what()),
              "invalid fiducial detector config:\n"
              "preprocess_config.clahe_clip_limit = 0: must be > 0\n"
              "algorithm_config.aruco_nano.threshold_window_size = 14: must be odd and >= 3\n");
  }
}

// A bad frame never throws; it sets frame_status and leaves no detections.
TEST(FiducialPipeline, BadImageReportsStatusWithoutDetections)
{
  FiducialPipeline pipeline(FiducialDetectorConfig{});  // expects bgr8
  FiducialDetectionResult result;
  result.detections.resize(3);  // leftovers from a previous frame must be cleared

  pipeline.detect(cv::Mat(), result);
  EXPECT_EQ(result.frame_status, FrameStatus::ImageEmpty);
  EXPECT_TRUE(result.detections.empty());

  pipeline.detect(grayImage(), result);
  EXPECT_EQ(result.frame_status, FrameStatus::ImageTypeMismatch);

  pipeline.detect(cv::Mat(48, 64, CV_32FC3, cv::Scalar(0.5)), result);
  EXPECT_EQ(result.frame_status, FrameStatus::ImageTypeMismatch);
}

// Every preprocess path (bgr8, mono8, each with and without CLAHE) accepts its frame.
TEST(FiducialPipeline, ValidImageIsProcessedOnEveryPreprocessPath)
{
  for (const bool enable_clahe : {false, true}) {
    FiducialDetectorConfig bgr_config;
    bgr_config.preprocess_config.enable_clahe = enable_clahe;
    FiducialDetectorConfig mono_config = bgr_config;
    mono_config.input_encoding = ImageEncoding::Mono8;

    FiducialPipeline bgr_pipeline(bgr_config);
    FiducialPipeline mono_pipeline(mono_config);
    FiducialDetectionResult result;

    bgr_pipeline.detect(bgrImage(), result);
    EXPECT_EQ(result.frame_status, FrameStatus::Ok) << "bgr8, clahe " << enable_clahe;

    mono_pipeline.detect(grayImage(), result);
    EXPECT_EQ(result.frame_status, FrameStatus::Ok) << "mono8, clahe " << enable_clahe;
  }
}

}  // namespace ppmt::fiducial
