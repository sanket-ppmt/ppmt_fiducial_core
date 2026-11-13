#include "ppmt_fiducial_core/algorithms/aruco_nano/aruco_nano_fiducial_detector.hpp"

#include "ppmt_fiducial_core/algorithms/aruco_nano/threshold.hpp"

namespace ppmt::fiducial
{

ArucoNanoFiducialDetector::ArucoNanoFiducialDetector(const ArucoNanoConfig & config)
: config_(config)
{
}

void ArucoNanoFiducialDetector::detect(const cv::Mat & gray_image,
                                       std::vector<TagDetection> & detections)
{
  detections.clear();
  aruco_nano::computeThreshold(gray_image, config_.threshold_window_size,
                               config_.threshold_offset, binary_image_);
  // Next stages: contours -> quads -> refine -> decode -> refine.
}

}  // namespace ppmt::fiducial
