// Row scans with OpenCV SIMD intrinsics. Built when PPMT_FIDUCIAL_USE_SIMD=ON
// (cmake/FeatureSimdRowScan.cmake checks that OpenCV has SIMD for the target).
#include "ppmt_fiducial_core/algorithms/aruco_nano/row_scan.hpp"

#include <algorithm>

#include <opencv2/core/hal/intrin.hpp>

namespace ppmt::fiducial::aruco_nano
{

// OpenCV 4.5.4 SIMD API.
static int laneCount() { return cv::v_uint8::nlanes; }
static cv::v_uint8 equalMask(const cv::v_uint8 & a, const cv::v_uint8 & b) { return a == b; }
static cv::v_uint8 notEqualMask(const cv::v_uint8 & a, const cv::v_uint8 & b) { return a != b; }

// Scans [start, end) one full vector (lane_count pixels) at a time.
// - Returns the index of the first pixel where mask_of(pixel, 0) is true.
// - If none: returns where the full vectors end; the caller scans the remaining tail.
template <typename MaskFunction>
static int scanVectors(const std::uint8_t * row, int start, int end, MaskFunction mask_of)
{
  const int lane_count = laneCount();
  const int vector_count = std::max(0, (end - start) / lane_count);
  const cv::v_uint8 zero = cv::vx_setzero_u8();  // every lane 0

  for (int vector_index = 0; vector_index < vector_count; ++vector_index) {
    const int vector_start = start + vector_index * lane_count;
    // vx_load: lane_count pixels from row[vector_start] (no alignment needed).
    // mask: lane = 0xFF where it matches, else 0.
    const cv::v_uint8 mask = mask_of(cv::vx_load(row + vector_start), zero);
    // v_check_any: any lane set? v_scan_forward: index of the first set lane.
    if (cv::v_check_any(mask)) {
      return vector_start + cv::v_scan_forward(mask);
    }
  }
  return start + vector_count * lane_count;
}

int findNextNonZero(const std::uint8_t * row, int start, int end)
{
  int index = scanVectors(row, start, end, notEqualMask);
  while (index < end && row[index] == 0) {
    ++index;
  }
  return index;
}

int findNextZero(const std::uint8_t * row, int start, int end)
{
  int index = scanVectors(row, start, end, equalMask);
  while (index < end && row[index] != 0) {
    ++index;
  }
  return index;
}

}  // namespace ppmt::fiducial::aruco_nano
