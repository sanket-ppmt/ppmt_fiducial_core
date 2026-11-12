#pragma once

#include <memory>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include "ppmt_fiducial_core/configs/fiducial_detector_config.hpp"
#include "ppmt_fiducial_core/data_types/fiducial_detection_result.hpp"
#include "ppmt_fiducial_core/fiducial_detector_interface.hpp"

namespace ppmt::fiducial
{

// What the host (e.g. ROS node) uses: image in, tags out.
// - Stages: check image -> preprocess -> algorithm -> tag filter.
// - One instance per camera; reuses its buffers across frames. Movable, not copyable.
class FiducialPipeline
{
public:
  // Validates the whole config and builds the algorithm.
  // - Throws std::invalid_argument listing every problem, one line each.
  explicit FiducialPipeline(const FiducialDetectorConfig & config);

  // Processes one frame.
  // - image: bgr8 (CV_8UC3) or mono8 (CV_8UC1), as config.input_encoding.
  // - result: overwritten; result.detections keeps its capacity across calls.
  // - Bad image: result.frame_status says why, no detections.
  // - Does not throw for any image content; only std::bad_alloc can escape.
  void detect(const cv::Mat & image, FiducialDetectionResult & result);

private:
  // Checks emptiness and that the cv::Mat type matches config_.input_encoding.
  FrameStatus checkImage(const cv::Mat & image) const;

  // Prepares the algorithm's input.
  // - bgr8: converted into gray_image_; mono8: used as is, no copy.
  // - CLAHE enabled: written into contrast_enhanced_image_.
  // - Returns whichever of these the algorithm should use.
  const cv::Mat & preprocess(const cv::Mat & image);

  // Removes detections with an id not in allowed_ids or a low bit_contrast_margin.
  void applyTagFilter(std::vector<TagDetection> & detections) const;

  FiducialDetectorConfig config_;
  std::unique_ptr<FiducialDetectorInterface> fiducial_detector_;
  cv::Ptr<cv::CLAHE> clahe_;           // only created when enable_clahe
  cv::Mat gray_image_;                 // reused buffer
  cv::Mat contrast_enhanced_image_;    // reused buffer, CLAHE output
};

}  // namespace ppmt::fiducial
