#pragma once

#include <string>

#include "ppmt_fiducial_core/configs/config_source_interface.hpp"

namespace ppmt::fiducial
{

// Image preparation before the algorithm.
struct PreprocessConfig
{
  // CLAHE (contrast-limited histogram equalisation), as in the current ROS node.
  // - Off by default: Gate A compares against aruco_nano, which has no CLAHE.
  // - TODO: copy clip limit and tile size from the current node.
  bool enable_clahe = false;
  double clahe_clip_limit = 2.0;  // > 0
  int clahe_tile_grid_size = 8;   // cells per side, >= 1

  // Reads every field from `config_source` as `name_prefix + field name`.
  // - Fields missing in the source keep their current value (the default).
  void loadFromConfigSource(ConfigSourceInterface & config_source, const std::string & name_prefix);

  // Checks every field.
  // - Returns "" if valid; otherwise one line per problem: name, value, allowed range.
  std::string validate(const std::string & name_prefix) const;
};

}  // namespace ppmt::fiducial
