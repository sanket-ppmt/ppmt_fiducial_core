// Row scans with OpenCV SIMD intrinsics. Built when PPMT_FIDUCIAL_USE_SIMD=ON
// (cmake/FeatureSimdRowScan.cmake checks that OpenCV has SIMD for the target).
#include "ppmt_fiducial_core/algorithms/aruco_nano/row_scan.hpp"

#include <opencv2/core/hal/intrin.hpp>

namespace ppmt::fiducial::aruco_nano
{

namespace
{

// OpenCV SIMD API as in 4.5.4 (ROS Humble; also 4.6 on Jazzy).
// - Future ref: OpenCV >= 4.9 (and aruco_nano) use VTraits<>::vlanes(), v_eq, v_ne instead
//   of nlanes, ==, != ; change these three if we ever build against it.
int laneCount() { return cv::v_uint8::nlanes; }
cv::v_uint8 equalMask(const cv::v_uint8 & a, const cv::v_uint8 & b) { return a == b; }
cv::v_uint8 notEqualMask(const cv::v_uint8 & a, const cv::v_uint8 & b) { return a != b; }

// Advances over full vectors until one has a lane matching `mask_of`; returns the exact index
// of that lane, or where the full vectors end.
template <typename MaskFunction>
int scanVectors(const std::uint8_t * row, int index, int end, MaskFunction mask_of)
{
  const cv::v_uint8 zero = cv::vx_setzero_u8();
  const int lane_count = laneCount();
  for (; index <= end - lane_count; index += lane_count) {
    const cv::v_uint8 mask = mask_of(cv::vx_load(row + index), zero);
    if (cv::v_check_any(mask)) {
      return index + cv::v_scan_forward(mask);
    }
  }
  return index;
}

}  // namespace

int findNextNonZero(const std::uint8_t * row, int start, int end)
{
  int index = scanVectors(row, start, end, notEqualMask);
  for (; index < end && row[index] == 0; ++index) {
  }
  return index;
}

int findNextZero(const std::uint8_t * row, int start, int end)
{
  int index = scanVectors(row, start, end, equalMask);
  for (; index < end && row[index] != 0; ++index) {
  }
  return index;
}

}  // namespace ppmt::fiducial::aruco_nano
