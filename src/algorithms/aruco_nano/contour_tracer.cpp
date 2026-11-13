#include "ppmt_fiducial_core/algorithms/aruco_nano/contour_tracer.hpp"

#include <cstdint>
#include <cstring>

#include "ppmt_fiducial_core/algorithms/aruco_nano/row_scan.hpp"

namespace ppmt::fiducial::aruco_nano
{

namespace
{
constexpr std::uint8_t background = 0;
constexpr std::uint8_t foreground = 255;
constexpr std::uint8_t visited = 100;

// 8 neighbours in search order W, NW, N, NE, E, SE, S, SW.
constexpr int neighbour_dx[8] = {-1, -1, 0, 1, 1, 1, 0, -1};
constexpr int neighbour_dy[8] = {0, -1, -1, -1, 0, 1, 1, 1};
}  // namespace

void ContourTracer::trace(cv::Mat & binary_image, int min_contour_length,
                          float max_revisit_fraction,
                          std::vector<std::vector<cv::Point>> & contours)
{
  contours.clear();
  if (binary_image.empty() || binary_image.type() != CV_8UC1) {
    return;
  }
  const int rows = binary_image.rows;
  const int cols = binary_image.cols;
  const int step = static_cast<int>(binary_image.step);
  std::uint8_t * const data = binary_image.data;

  // Zero the 1-pixel border, so tracing never steps outside the image.
  std::memset(data, 0, cols);
  std::memset(data + (rows - 1) * step, 0, cols);
  for (int row_index = 1; row_index < rows - 1; ++row_index) {
    data[row_index * step] = 0;
    data[row_index * step + cols - 1] = 0;
  }

  // Pointer offsets of the 8 neighbours, repeated so a search can start at any direction.
  const int neighbour_offsets[16] = {
    -1, -step - 1, -step, -step + 1, 1, step + 1, step, step - 1,
    -1, -step - 1, -step, -step + 1, 1, step + 1, step, step - 1};

  for (int row_index = 1; row_index < rows - 1; ++row_index) {
    std::uint8_t * const row = data + row_index * step;
    for (int column = 1; column < cols - 1;) {
      column = findNextNonZero(row, column, cols);
      if (column == cols) {
        break;
      }

      if (row[column] == foreground) {
        contour_points_.clear();
        int x = column;
        int y = row_index;
        int search_index = 1;
        std::uint8_t * current = row + column;
        const std::uint8_t * const start = current;
        std::size_t revisit_count = 0;
        do {
          contour_points_.emplace_back(x, y);
          *current = visited;
          for (int attempt = 0; attempt < 8; ++attempt) {
            const int offset_index = search_index + attempt;
            std::uint8_t * const neighbour = current + neighbour_offsets[offset_index];
            if (*neighbour != background) {
              current = neighbour;
              const int direction = offset_index & 7;
              x += neighbour_dx[direction];
              y += neighbour_dy[direction];
              revisit_count += (*neighbour == visited) ? 1 : 0;
              search_index = (direction + 5) & 7;
              break;
            }
          }
        } while (current != start);

        const std::size_t length = contour_points_.size();
        if (static_cast<float>(revisit_count) <= static_cast<float>(length) * max_revisit_fraction &&
            length >= static_cast<std::size_t>(min_contour_length)) {
          contours.push_back(contour_points_);
        }
      }

      ++column;
      if (row[column] != 0) {
        column = findNextZero(row, column, cols);
      }
    }
  }
}

}  // namespace ppmt::fiducial::aruco_nano
