#pragma once

#include <cstdint>
#include <string>

#include "ppmt_fiducial_core/configs/config_source_interface.hpp"

namespace ppmt::fiducial
{

// ArucoNano algorithm.
// - Defaults = aruco_nano defaults, so Gate A can compare 1:1.
struct ArucoNanoConfig
{
  // Value of the "algorithm" config that selects this algorithm.
  static constexpr const char * algorithm_name = "aruco_nano";

  // Adaptive threshold: pixel is foreground if (box-filter mean - pixel) > offset.
  int threshold_window_size = 15;  // odd, >= 3
  int threshold_offset = 3;        // [0, 255]

  // Quad candidates: every side at least this long.
  int min_quad_side_px = 10;  // >= 1

  // Contour tracer: max fraction of a contour's pixels visited twice.
  float max_contour_revisit_fraction = 0.05F;  // (0, 1]

  // Decode retries with jittered corners.
  // - First attempt is unjittered; retries add Gaussian noise to each corner.
  // - Seed 0xFFFFFFFF = cv::RNG default, as aruco_nano.
  int max_decode_attempts = 5;          // >= 1
  float retry_jitter_sigma_px = 0.75F;  // >= 0
  std::uint64_t retry_jitter_seed = 0xFFFFFFFFULL;

  // Reads every field from `config_source` as `name_prefix + field name`.
  // - Fields missing in the source keep their current value (the default).
  void loadFromConfigSource(ConfigSourceInterface & config_source, const std::string & name_prefix);

  // Checks every field.
  // - Returns "" if valid; otherwise one line per problem: name, value, allowed range.
  std::string validate(const std::string & name_prefix) const;
};

}  // namespace ppmt::fiducial
