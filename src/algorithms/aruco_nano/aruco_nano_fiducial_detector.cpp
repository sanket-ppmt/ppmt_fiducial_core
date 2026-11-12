#include "ppmt_fiducial_core/algorithms/aruco_nano/aruco_nano_fiducial_detector.hpp"

namespace ppmt::fiducial
{

ArucoNanoFiducialDetector::ArucoNanoFiducialDetector(const ArucoNanoConfig & config)
: config_(config)
{
}

void ArucoNanoFiducialDetector::detect(const cv::Mat & /*gray_image*/,
                                       std::vector<TagDetection> & detections)
{
  detections.clear();
  // Stages come in step 5: threshold -> contours -> quads -> refine -> decode -> refine.
}

}  // namespace ppmt::fiducial
