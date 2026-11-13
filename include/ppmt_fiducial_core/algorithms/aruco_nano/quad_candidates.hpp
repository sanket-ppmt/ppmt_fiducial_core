#pragma once

#include <vector>

#include <opencv2/core/types.hpp>

#include "ppmt_fiducial_core/data_types/aruco_nano/quad_candidate.hpp"

namespace ppmt::fiducial::aruco_nano
{

// Stage 4: keeps the contours that simplify to a convex quad with long enough sides.
// - Port of aruco_nano.h lines 357-372 (inline in detect()) and sort() (lines 641-651).
// - Owns its polygon buffer, reused across frames.
class QuadCandidateFinder
{
public:
  // For each contour, in order:
  // - simplify to a polygon (cv::approxPolyDP, tolerance 3% of the contour length);
  // - keep it if it has 4 corners, is convex and every side is >= min_quad_side_px;
  // - order its corners clockwise as seen in the image.
  // - quad_candidates: cleared, then filled in contour order.
  void find(const std::vector<std::vector<cv::Point>> & contours, int min_quad_side_px,
            std::vector<QuadCandidate> & quad_candidates);

private:
  std::vector<cv::Point> polygon_;  // approxPolyDP output
};

}  // namespace ppmt::fiducial::aruco_nano
