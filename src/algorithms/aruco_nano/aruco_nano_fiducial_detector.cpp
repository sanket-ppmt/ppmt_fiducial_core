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
  // Contours shorter than 4 quad sides cannot become a quad (as aruco_nano).
  contour_tracer_.trace(binary_image_, 4 * config_.min_quad_side_px,
                        config_.max_contour_revisit_fraction, contours_);
  // Next stages: quads -> refine -> decode -> refine.
}

}  // namespace ppmt::fiducial
