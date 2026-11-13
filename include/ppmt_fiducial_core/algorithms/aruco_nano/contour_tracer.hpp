#pragma once

#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace ppmt::fiducial::aruco_nano
{

// Stage 3: traces the outline of every white blob in the binary image.
// - Port of aruco_nano's visitedAwareTracingContour (aruco_nano.h lines 748-850).
// - Owns its point buffer, reused across frames. One instance per detector.
class ContourTracer
{
public:
  // Finds contours in scan order (rows top to bottom, left to right).
  // - binary_image: CV_8UC1, 0/255. MODIFIED: border set to 0, traced pixels set to 100.
  // - Keeps a contour if length >= min_contour_length and
  //   revisited pixels <= length * max_revisit_fraction.
  // - contours: cleared, then filled; points in tracing order.
  void trace(cv::Mat & binary_image, int min_contour_length, float max_revisit_fraction,
             std::vector<std::vector<cv::Point>> & contours);

private:
  std::vector<cv::Point> contour_points_;  // contour being traced
};

}  // namespace ppmt::fiducial::aruco_nano
