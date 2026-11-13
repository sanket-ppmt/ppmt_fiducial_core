#include "ppmt_fiducial_core/algorithms/aruco_nano/quad_candidates.hpp"

#include <utility>

#include <opencv2/imgproc.hpp>

namespace ppmt::fiducial::aruco_nano
{

namespace
{

// approxPolyDP tolerance as a fraction of the contour length (hard-coded in aruco_nano).
constexpr double polygon_tolerance_fraction = 0.03;

// Every side, as squared integer length, at least min_side_squared (as aruco_nano).
bool allSidesAtLeast(const std::vector<cv::Point> & polygon, int min_side_squared)
{
  for (int corner = 0; corner < 4; ++corner) {
    const cv::Point side = polygon[corner] - polygon[(corner + 1) % 4];
    if (side.x * side.x + side.y * side.y < min_side_squared) {
      return false;
    }
  }
  return true;
}

// Clockwise as seen in the image (y down): if corner 2 is on the wrong side of the
// line 0 -> 1, swap corners 1 and 3 (aruco_nano's sort()).
void orderCornersClockwise(std::array<cv::Point2f, 4> & corners)
{
  const double dx1 = corners[1].x - corners[0].x;
  const double dy1 = corners[1].y - corners[0].y;
  const double dx2 = corners[2].x - corners[0].x;
  const double dy2 = corners[2].y - corners[0].y;
  if ((dx1 * dy2) - (dy1 * dx2) < 0.0) {
    std::swap(corners[1], corners[3]);
  }
}

}  // namespace

void QuadCandidateFinder::find(const std::vector<std::vector<cv::Point>> & contours,
                               int min_quad_side_px, std::vector<QuadCandidate> & quad_candidates)
{
  quad_candidates.clear();
  const int min_side_squared = min_quad_side_px * min_quad_side_px;

  for (std::size_t contour_index = 0; contour_index < contours.size(); ++contour_index) {
    const std::vector<cv::Point> & contour = contours[contour_index];
    cv::approxPolyDP(contour, polygon_,
                     static_cast<double>(contour.size()) * polygon_tolerance_fraction, true);
    if (polygon_.size() != 4 || !cv::isContourConvex(polygon_) ||
        !allSidesAtLeast(polygon_, min_side_squared)) {
      continue;
    }

    QuadCandidate quad_candidate;
    for (int corner = 0; corner < 4; ++corner) {
      quad_candidate.corners[corner] = cv::Point2f(polygon_[corner].x, polygon_[corner].y);
    }
    orderCornersClockwise(quad_candidate.corners);
    quad_candidate.contour_index = static_cast<int>(contour_index);
    quad_candidates.push_back(quad_candidate);
  }
}

}  // namespace ppmt::fiducial::aruco_nano
