#include "ppmt_fiducial_core/algorithms/aruco_nano/contour_tracer.hpp"

#include <array>
#include <cstdint>

#include "ppmt_fiducial_core/algorithms/aruco_nano/row_scan.hpp"

namespace ppmt::fiducial::aruco_nano
{

// Pixel values in the binary image.
constexpr std::uint8_t background = 0;
constexpr std::uint8_t foreground = 255;
constexpr std::uint8_t visited = 100;

// The 8 neighbours, clockwise as seen in the image: W, NW, N, NE, E, SE, S, SW.
constexpr int neighbour_dx[8] = {-1, -1, 0, 1, 1, 1, 0, -1};
constexpr int neighbour_dy[8] = {0, -1, -1, -1, 0, 1, 1, 1};
constexpr int no_neighbour = -1;

// First search from a contour's start pixel begins at NW (its W neighbour is background).
constexpr int first_search_direction = 1;

using NeighbourOffsets = std::array<int, 8>;

// Pointer offset of each neighbour, for a row stride of `step` bytes.
static NeighbourOffsets neighbourOffsets(int step)
{
  NeighbourOffsets offsets{};
  for (int direction = 0; direction < 8; ++direction) {
    offsets[direction] = neighbour_dy[direction] * step + neighbour_dx[direction];
  }
  return offsets;
}

// Sets the 1-pixel image border to background, so tracing never steps outside the image.
static void clearBorder(cv::Mat & binary_image)
{
  binary_image.row(0).setTo(background);
  binary_image.row(binary_image.rows - 1).setTo(background);
  binary_image.col(0).setTo(background);
  binary_image.col(binary_image.cols - 1).setTo(background);
}

// First non-background neighbour, checking clockwise from first_direction; no_neighbour if none.
static int findNeighbour(const std::uint8_t * pixel, int first_direction,
                         const NeighbourOffsets & offsets)
{
  for (int attempt = 0; attempt < 8; ++attempt) {
    const int direction = (first_direction + attempt) % 8;
    if (pixel[offsets[direction]] != background) {
      return direction;
    }
  }
  return no_neighbour;
}

// Walks the outside of one blob (Moore neighbour tracing) until back at the start pixel.
// - Marks every pixel walked on as visited; contour_points: cleared, then the walk in order.
// - Returns the revisit count: steps onto an already visited pixel.
static std::size_t traceContour(std::uint8_t * start_pixel, cv::Point start_point,
                                const NeighbourOffsets & offsets,
                                std::vector<cv::Point> & contour_points)
{
  contour_points.clear();
  std::uint8_t * pixel = start_pixel;
  cv::Point point = start_point;
  int search_direction = first_search_direction;
  std::size_t revisit_count = 0;
  do {
    contour_points.push_back(point);
    *pixel = visited;
    const int direction = findNeighbour(pixel, search_direction, offsets);
    if (direction == no_neighbour) {
      break;  // a single isolated pixel
    }
    pixel += offsets[direction];
    point += cv::Point(neighbour_dx[direction], neighbour_dy[direction]);
    if (*pixel == visited) {
      ++revisit_count;
    }
    // Next search starts one step clockwise from the pixel we came from (direction + 4).
    search_direction = (direction + 5) % 8;
  } while (pixel != start_pixel);
  return revisit_count;
}

// A contour is kept if it is long enough and does not run back over itself too often.
static bool isContourKept(std::size_t length, std::size_t revisit_count, int min_contour_length,
                          float max_revisit_fraction)
{
  return length >= static_cast<std::size_t>(min_contour_length) &&
         static_cast<float>(revisit_count) <= static_cast<float>(length) * max_revisit_fraction;
}

void ContourTracer::trace(cv::Mat & binary_image, int min_contour_length,
                          float max_revisit_fraction,
                          std::vector<std::vector<cv::Point>> & contours)
{
  contours.clear();
  if (binary_image.empty() || binary_image.type() != CV_8UC1) {
    return;
  }
  clearBorder(binary_image);
  const NeighbourOffsets offsets = neighbourOffsets(static_cast<int>(binary_image.step));
  const int cols = binary_image.cols;

  // Each run of non-zero pixels in a row is checked once, at its left end:
  // - foreground: a new blob, so trace it;
  // - visited: part of a blob already traced, skip it.
  for (int row_index = 1; row_index < binary_image.rows - 1; ++row_index) {
    std::uint8_t * const row = binary_image.ptr<std::uint8_t>(row_index);
    int run_start = findNextNonZero(row, 1, cols);
    while (run_start < cols) {
      if (row[run_start] == foreground) {
        const std::size_t revisit_count = traceContour(
          row + run_start, cv::Point(run_start, row_index), offsets, contour_points_);
        if (isContourKept(contour_points_.size(), revisit_count, min_contour_length,
                          max_revisit_fraction)) {
          contours.push_back(contour_points_);
        }
      }
      const int run_end = findNextZero(row, run_start + 1, cols);
      run_start = findNextNonZero(row, run_end, cols);
    }
  }
}

}  // namespace ppmt::fiducial::aruco_nano
