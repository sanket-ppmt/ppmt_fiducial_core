#pragma once

#include <opencv2/core/mat.hpp>

namespace ppmt::fiducial::aruco_nano
{

// Stage 2: adaptive threshold that marks dark tag borders.
// - binary_image(x, y) = 255 if (local mean - gray_image(x, y)) > offset, else 0.
// - Local mean: window_size x window_size box filter, border replicated.
// - Same OpenCV calls as aruco_nano.h lines 335-337, so the result is bit-identical.
// - gray_image: CV_8UC1. binary_image: CV_8UC1, same size; reused if already allocated.
void computeThreshold(const cv::Mat & gray_image, int window_size, int offset,
                      cv::Mat & binary_image);

}  // namespace ppmt::fiducial::aruco_nano
