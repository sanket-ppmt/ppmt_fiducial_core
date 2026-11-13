#pragma once

#include <vector>

#include <opencv2/core/mat.hpp>

#include "ppmt_fiducial_core/algorithms/aruco_nano/contour_tracer.hpp"
#include "ppmt_fiducial_core/configs/aruco_nano_config.hpp"
#include "ppmt_fiducial_core/fiducial_detector_interface.hpp"

namespace ppmt::fiducial
{

// Port of aruco_nano (pinned commit, see tools/oracle/README.md), AprilTag 36h11 only.
// - Stages are added one at a time in step 5, each checked against the oracle.
class ArucoNanoFiducialDetector : public FiducialDetectorInterface
{
public:
  // - config: must be valid; createFiducialDetector() validates it.
  explicit ArucoNanoFiducialDetector(const ArucoNanoConfig & config);

  void detect(const cv::Mat & gray_image, std::vector<TagDetection> & detections) override;

private:
  ArucoNanoConfig config_;
  cv::Mat binary_image_;                        // stage 2 output, reused across frames
  aruco_nano::ContourTracer contour_tracer_;     // stage 3
  std::vector<std::vector<cv::Point>> contours_; // stage 3 output
};

}  // namespace ppmt::fiducial
