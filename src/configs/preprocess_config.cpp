#include "ppmt_fiducial_core/configs/preprocess_config.hpp"

#include <sstream>

#include "ppmt_fiducial_core/configs/config_value_reading.hpp"

namespace ppmt::fiducial
{

void PreprocessConfig::loadFromConfigSource(ConfigSourceInterface & config_source,
                                            const std::string & name_prefix)
{
  readConfigValue(config_source, name_prefix + "enable_clahe", enable_clahe);
  readConfigValue(config_source, name_prefix + "clahe_clip_limit", clahe_clip_limit);
  readConfigValue(config_source, name_prefix + "clahe_tile_grid_size", clahe_tile_grid_size);
}

std::string PreprocessConfig::validate(const std::string & name_prefix) const
{
  std::ostringstream problems;
  const auto report = [&](const char * field, auto value, const char * rule) {
    problems << name_prefix << field << " = " << value << ": " << rule << "\n";
  };

  if (!(clahe_clip_limit > 0.0)) {
    report("clahe_clip_limit", clahe_clip_limit, "must be > 0");
  }
  if (clahe_tile_grid_size < 1) {
    report("clahe_tile_grid_size", clahe_tile_grid_size, "must be >= 1");
  }
  return problems.str();
}

}  // namespace ppmt::fiducial
