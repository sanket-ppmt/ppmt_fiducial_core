#pragma once

#include <array>

#include <opencv2/core/types.hpp>

namespace ppmt::fiducial
{

// One tag found by a detector backend.
// - Image pixels, OpenCV convention: x right, y down, pixel centres at integers.
struct TagDetection
{
  // AprilTag 36h11 id; -1 = not set.
  int id = -1;

  // Outer corners of the black border.
  // - Order: top-left, top-right, bottom-right, bottom-left (clockwise as printed).
  // - Corner 0 is fixed by the decoded rotation, not by the tag's orientation in the image.
  std::array<cv::Point2f, 4> corners{};

  // How clearly the cells read as black or white, in [0, 1].
  // - Per cell: |intensity - threshold| / (half the tag's black-to-white contrast).
  // - Value: minimum over all cells.
  // - 0 = some cell is ambiguous, 1 = every cell fully black or white.
  // - PROVISIONAL: refine once we see real data.
  float bit_contrast_margin = 0.0F;
};

}  // namespace ppmt::fiducial
