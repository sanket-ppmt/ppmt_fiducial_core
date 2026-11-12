#include "ppmt_fiducial_core/fiducial_pipeline.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>

#include "ppmt_fiducial_core/create_fiducial_detector.hpp"

namespace ppmt::fiducial
{

FiducialPipeline::FiducialPipeline(const FiducialDetectorConfig & config)
: config_(config)
{
  const std::string problems = config_.validate();
  if (!problems.empty()) {
    throw std::invalid_argument("invalid fiducial detector config:\n" + problems);
  }
  fiducial_detector_ = createFiducialDetector(config_.algorithm_config);

  const PreprocessConfig & preprocess_config = config_.preprocess_config;
  if (preprocess_config.enable_clahe) {
    const cv::Size tile_grid(preprocess_config.clahe_tile_grid_size,
                             preprocess_config.clahe_tile_grid_size);
    clahe_ = cv::createCLAHE(preprocess_config.clahe_clip_limit, tile_grid);
  }
}

void FiducialPipeline::detect(const cv::Mat & image, FiducialDetectionResult & result)
{
  result.detections.clear();
  result.frame_status = checkImage(image);
  if (result.frame_status != FrameStatus::Ok) {
    return;
  }

  const cv::Mat & algorithm_input = preprocess(image);
  fiducial_detector_->detect(algorithm_input, result.detections);
  applyTagFilter(result.detections);
}

FrameStatus FiducialPipeline::checkImage(const cv::Mat & image) const
{
  if (image.empty()) {
    return FrameStatus::ImageEmpty;
  }
  const int expected_type = config_.input_encoding == ImageEncoding::Bgr8 ? CV_8UC3 : CV_8UC1;
  if (image.type() != expected_type) {
    return FrameStatus::ImageTypeMismatch;
  }
  return FrameStatus::Ok;
}

const cv::Mat & FiducialPipeline::preprocess(const cv::Mat & image)
{
  const cv::Mat * gray_image = &image;  // mono8: used as is, no copy
  if (config_.input_encoding == ImageEncoding::Bgr8) {
    cv::cvtColor(image, gray_image_, cv::COLOR_BGR2GRAY);
    gray_image = &gray_image_;
  }
  if (clahe_) {
    clahe_->apply(*gray_image, contrast_enhanced_image_);
    return contrast_enhanced_image_;
  }
  return *gray_image;
}

void FiducialPipeline::applyTagFilter(std::vector<TagDetection> & detections) const
{
  const TagFilterConfig & tag_filter_config = config_.tag_filter_config;
  const std::vector<int> & allowed_ids = tag_filter_config.allowed_ids;

  const auto is_rejected = [&](const TagDetection & detection) {
    const bool id_allowed =
      allowed_ids.empty() ||
      std::find(allowed_ids.begin(), allowed_ids.end(), detection.id) != allowed_ids.end();
    const bool contrast_sufficient =
      detection.bit_contrast_margin >= tag_filter_config.min_bit_contrast_margin;
    return !id_allowed || !contrast_sufficient;
  };
  detections.erase(std::remove_if(detections.begin(), detections.end(), is_rejected),
                   detections.end());
}

}  // namespace ppmt::fiducial
