// OpenCV < 4.7 compatibility for the oracle ONLY. Never used by the library.
//
// aruco_nano.h (pinned commit) includes this header, which exists in OpenCV >= 4.7 where
// aruco moved into objdetect. On older OpenCV (e.g. Ubuntu 22.04's 4.5.4) aruco is the
// contrib module and differs in three ways that aruco_nano.h touches:
//   1. getPredefinedDictionary() returns Ptr<Dictionary> instead of Dictionary.
//   2. DICT_ARUCO_MIP_36h12 does not exist. aruco_nano uses it only as the default
//      dictionary; the oracle always passes DICT_APRILTAG_36h11 explicitly.
//   3. The newer universal-intrinsics names VTraits<T>::vlanes(), v_eq, v_ne (used by the
//      contour tracer's row skip) are T::nlanes, ==, != in older OpenCV.
// Everything else aruco_nano uses (Dictionary::markerSize, Dictionary::identify) has the
// same signature in both. All version differences live in this one file.
#pragma once

#include <opencv2/core/version.hpp>

#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#error "OpenCV >= 4.7 provides this header itself; remove tools/oracle/compat from the include path"
#endif

#include <opencv2/aruco.hpp>
#include <opencv2/core/hal/intrin.hpp>

#if CV_SIMD
namespace cv
{
template <typename T>
struct VTraits
{
  static constexpr int vlanes() { return T::nlanes; }
};
template <typename T>
inline T v_eq(const T & a, const T & b) { return a == b; }
template <typename T>
inline T v_ne(const T & a, const T & b) { return a != b; }
}  // namespace cv
#endif

// Function-like macros do not expand recursively, so this rewrites
// cv::aruco::getPredefinedDictionary(x) into cv::aruco::getPredefinedDictionary(x).operator*().
#define getPredefinedDictionary(name) getPredefinedDictionary(name).operator*()
#define DICT_ARUCO_MIP_36h12 DICT_APRILTAG_36h11
