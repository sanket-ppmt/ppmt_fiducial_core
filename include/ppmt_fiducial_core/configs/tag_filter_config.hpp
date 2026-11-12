#pragma once

#include <string>
#include <vector>

#include "ppmt_fiducial_core/configs/config_source_interface.hpp"

namespace ppmt::fiducial
{

// Filtering of the algorithm's detections.
struct TagFilterConfig
{
  std::vector<int> allowed_ids;          // empty = all ids allowed; each in [0, 586]
  float min_bit_contrast_margin = 0.0F;  // [0, 1]; see TagDetection

  // Reads every field from `config_source` as `name_prefix + field name`.
  // - Fields missing in the source keep their current value (the default).
  void loadFromConfigSource(ConfigSourceInterface & config_source, const std::string & name_prefix);

  // Checks every field.
  // - Returns "" if valid; otherwise one line per problem: name, value, allowed range.
  std::string validate(const std::string & name_prefix) const;
};

}  // namespace ppmt::fiducial
