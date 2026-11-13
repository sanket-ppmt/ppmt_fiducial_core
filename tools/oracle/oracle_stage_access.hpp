#pragma once
// TOOLS AND TESTS ONLY. Gives access to the original aruco_nano's private stage functions,
// so each ported stage can be compared with the original on the same input.
// - aruco_nano.h stays unmodified; `private` is redefined only while it is included.
// - Everything aruco_nano.h includes is included first, so OpenCV and std are unaffected.
// - aruco_nano.h defines non-inline functions: include this in one .cpp per executable.

#include <algorithm>
#include <cstring>
#include <sstream>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/hal/intrin.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>  // compat/ on OpenCV < 4.7

#define private public
#include "aruco_nano.h"
#undef private

namespace ppmt::fiducial::oracle
{

// Stage 2 (threshold) of the original. It is inline in MarkerDetector::detect(), so it is
// copied verbatim from aruco_nano.h lines 335-337.
inline cv::Mat computeOriginalThreshold(const cv::Mat & bwimage,
                                        const aruco_nano::DetectorParameters & params)
{
  cv::Mat thresImage;
  cv::boxFilter(bwimage, thresImage, bwimage.type(),
                cv::Size(params.boxFilterSize, params.boxFilterSize), cv::Point(-1, -1), true,
                cv::BORDER_REPLICATE | cv::BORDER_ISOLATED);
  thresImage = thresImage - bwimage;
  cv::threshold(thresImage, thresImage, params.thres, 255, cv::THRESH_BINARY);
  return thresImage;
}

}  // namespace ppmt::fiducial::oracle
