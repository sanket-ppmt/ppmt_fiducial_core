#include "ppmt_fiducial_core/algorithms/aruco_nano/threshold.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace ppmt::fiducial::aruco_nano
{

void computeThreshold(const cv::Mat & gray_image, int window_size, int offset,
                      cv::Mat & binary_image)
{
  // Local mean, written straight into binary_image to avoid a second buffer.
  cv::boxFilter(gray_image, binary_image, gray_image.type(), cv::Size(window_size, window_size),
                cv::Point(-1, -1), true, cv::BORDER_REPLICATE | cv::BORDER_ISOLATED);
  // mean - pixel; uint8 saturates, so pixels brighter than their mean become 0.
  cv::subtract(binary_image, gray_image, binary_image);
  cv::threshold(binary_image, binary_image, offset, 255, cv::THRESH_BINARY);
}

}  // namespace ppmt::fiducial::aruco_nano
