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

// Stage 4 (quad candidates) of the original. Inline in MarkerDetector::detect(), so the loop
// is copied verbatim from aruco_nano.h lines 357-372. Only change: instead of refining,
// it records each sorted (unrefined) candidate and its contour index.
inline void findOriginalQuadCandidates(const std::vector<std::vector<cv::Point>> & contours,
                                       const aruco_nano::DetectorParameters & params,
                                       std::vector<aruco_nano::Marker> & unrefinedCandidates,
                                       std::vector<int> & contourIndices)
{
  std::vector<cv::Point> approxCurve;
  int minSizeSq = params.minSize * params.minSize;
  unrefinedCandidates.clear();
  contourIndices.clear();
  for (unsigned int i = 0; i < contours.size(); i++)
  {
      // can approximate to a convex rect?
      cv::approxPolyDP(contours[i], approxCurve, double(contours[i].size()) * 0.03, true);
      if (approxCurve.size() != 4 || !cv::isContourConvex(approxCurve)) continue;
      //check distance  between corners at least minSize pix
      if(  ((approxCurve[0].x-approxCurve[1].x)*(approxCurve[0].x-approxCurve[1].x) + (approxCurve[0].y-approxCurve[1].y)*(approxCurve[0].y-approxCurve[1].y))<minSizeSq) continue;
      if(  ((approxCurve[1].x-approxCurve[2].x)*(approxCurve[1].x-approxCurve[2].x) + (approxCurve[1].y-approxCurve[2].y)*(approxCurve[1].y-approxCurve[2].y))<minSizeSq) continue;
      if(  ((approxCurve[2].x-approxCurve[3].x)*(approxCurve[2].x-approxCurve[3].x) + (approxCurve[2].y-approxCurve[3].y)*(approxCurve[2].y-approxCurve[3].y))<minSizeSq) continue;
      if(  ((approxCurve[3].x-approxCurve[0].x)*(approxCurve[3].x-approxCurve[0].x) + (approxCurve[3].y-approxCurve[0].y)*(approxCurve[3].y-approxCurve[0].y))<minSizeSq) continue;
      // // add the points
      aruco_nano::Marker marker;marker.reserve(4);
      for (int j = 0; j < 4; j++)
          marker.emplace_back( cv::Point2f( approxCurve[j].x,approxCurve[j].y));
      //sort corner in clockwise direction
      marker=aruco_nano::MarkerDetector::sort(marker);
      unrefinedCandidates.push_back(marker);
      contourIndices.push_back(static_cast<int>(i));
  }
}

}  // namespace ppmt::fiducial::oracle
