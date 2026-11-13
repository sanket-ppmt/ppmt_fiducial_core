# Row scan used by the ArucoNano contour tracer: SIMD (default) or scalar loops.
# - Exactly one of the two .cpp files is compiled; the source has no #if.
# - Results are identical; only speed differs.
option(PPMT_FIDUCIAL_USE_SIMD "SIMD row scan in the ArucoNano contour tracer" ON)

if(PPMT_FIDUCIAL_USE_SIMD)
  include(CheckCXXSourceCompiles)
  set(CMAKE_REQUIRED_INCLUDES ${OpenCV_INCLUDE_DIRS})
  check_cxx_source_compiles("
    #include <opencv2/core/hal/intrin.hpp>
    #if !CV_SIMD
    #error OpenCV has no SIMD for this target
    #endif
    int main() { return 0; }" PPMT_FIDUCIAL_OPENCV_HAS_SIMD)
  unset(CMAKE_REQUIRED_INCLUDES)

  if(NOT PPMT_FIDUCIAL_OPENCV_HAS_SIMD)
    message(FATAL_ERROR
      "OpenCV has no SIMD for this target. Configure with -DPPMT_FIDUCIAL_USE_SIMD=OFF.")
  endif()
  target_sources(ppmt_fiducial_core PRIVATE src/algorithms/aruco_nano/row_scan_simd.cpp)
else()
  target_sources(ppmt_fiducial_core PRIVATE src/algorithms/aruco_nano/row_scan_scalar.cpp)
endif()
