#pragma once

#include <vector>

#include <opencv2/core/mat.hpp>

#include "ppmt_fiducial_core/data_types/tag_detection.hpp"

namespace ppmt::fiducial
{

// A fiducial detection algorithm (ArucoNano, later AprilTagLibrary).
// - Only finds tags; preprocess, tag filter and pose are separate pipeline stages.
// - One instance per camera: it owns reusable buffers and state, so it is not copyable.
class FiducialDetectorInterface
{
public:
  virtual ~FiducialDetectorInterface() = default;

  FiducialDetectorInterface(const FiducialDetectorInterface &) = delete;
  FiducialDetectorInterface & operator=(const FiducialDetectorInterface &) = delete;

  // Finds tags in one image.
  // - gray_image: CV_8UC1, not empty; the pipeline guarantees this.
  // - detections: cleared, then filled; capacity is kept across calls.
  // - Does not throw for any image content; only std::bad_alloc can escape.
  virtual void detect(const cv::Mat & gray_image, std::vector<TagDetection> & detections) = 0;

protected:
  FiducialDetectorInterface() = default;
};

}  // namespace ppmt::fiducial
