#pragma once

#include <array>

#include <opencv2/core/types.hpp>

namespace ppmt::fiducial::aruco_nano
{

// A convex 4-corner polygon from one contour; may or may not be a tag (stage 4 output).
struct QuadCandidate
{
  // Polygon corners (integer pixel positions as float), clockwise as seen in the image.
  // - Corner 0 is arbitrary until the tag is decoded (stage 7 fixes it by rotation).
  std::array<cv::Point2f, 4> corners{};

  // Index of the contour it came from; stage 5 refines the corners with its points.
  int contour_index = -1;
};

}  // namespace ppmt::fiducial::aruco_nano
