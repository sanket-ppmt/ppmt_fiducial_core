// Row scans with scalar loops. Built when PPMT_FIDUCIAL_USE_SIMD=OFF; same results as SIMD.
#include "ppmt_fiducial_core/algorithms/aruco_nano/row_scan.hpp"

namespace ppmt::fiducial::aruco_nano
{

int findNextNonZero(const std::uint8_t * row, int start, int end)
{
  int index = start;
  while (index < end && row[index] == 0) {
    ++index;
  }
  return index;
}

int findNextZero(const std::uint8_t * row, int start, int end)
{
  int index = start;
  while (index < end && row[index] != 0) {
    ++index;
  }
  return index;
}

}  // namespace ppmt::fiducial::aruco_nano
