#pragma once

#include <array>
#include <cstdint>

#include "ppmt_fiducial_core/algorithms/aruco_nano/april_tag_36h11_codes.hpp"

namespace ppmt::fiducial
{

// Looks up a 36-bit code in AprilTag 36h11, in all 4 rotations.
// - Replaces cv::aruco::Dictionary::identify with error correction 0 (as aruco_nano uses it).
// - Owned by the detector (no static state); built once, no allocation.
class AprilTag36h11Dictionary
{
public:
  AprilTag36h11Dictionary();

  // Finds the tag whose code, in one of its 4 rotations, equals `code` exactly.
  // - code: 36 data bits, layout as april_tag_36h11_codes (bit 35 = cell (0, 0), 1 = white).
  // - On match: returns true, sets id and rotation (numbering as cv::aruco identify).
  // - Several matches (not expected in 36h11): lowest id, then lowest rotation, as OpenCV.
  bool identify(std::uint64_t code, int & id, int & rotation) const;

private:
  struct CodeEntry
  {
    std::uint64_t code;
    int id;
    int rotation;
  };
  static constexpr int rotation_count = 4;

  // All codes x rotations, sorted by (code, id, rotation) for binary search.
  std::array<CodeEntry, april_tag_36h11_code_count * rotation_count> entries_sorted_by_code_;
};

}  // namespace ppmt::fiducial
