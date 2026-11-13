#pragma once

#include <cstdint>

namespace ppmt::fiducial::aruco_nano
{

// Row scans used by the contour tracer to skip runs of equal pixels.
// - Two implementations, one compiled (CMake option PPMT_FIDUCIAL_USE_SIMD):
//   row_scan_simd.cpp (default; as aruco_nano.h lines 780-791) or row_scan_scalar.cpp.
// - Same results either way.

// First index in [start, end) with row[index] != 0; end if none.
int findNextNonZero(const std::uint8_t * row, int start, int end);

// First index in [start, end) with row[index] == 0; end if none.
int findNextZero(const std::uint8_t * row, int start, int end);

}  // namespace ppmt::fiducial::aruco_nano
