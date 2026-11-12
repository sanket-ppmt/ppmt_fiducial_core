#include "ppmt_fiducial_core/configs/tag_filter_config.hpp"

#include <sstream>

#include "ppmt_fiducial_core/configs/config_value_reading.hpp"

namespace ppmt::fiducial
{

namespace
{
constexpr int max_april_tag_36h11_id = 586;
}  // namespace

void TagFilterConfig::loadFromConfigSource(ConfigSourceInterface & config_source,
                                           const std::string & name_prefix)
{
  readConfigValue(config_source, name_prefix + "allowed_ids", allowed_ids);
  readConfigValue(config_source, name_prefix + "min_bit_contrast_margin", min_bit_contrast_margin);
}

std::string TagFilterConfig::validate(const std::string & name_prefix) const
{
  std::ostringstream problems;

  for (std::size_t index = 0; index < allowed_ids.size(); ++index) {
    const int id = allowed_ids[index];
    if (id < 0 || id > max_april_tag_36h11_id) {
      problems << name_prefix << "allowed_ids[" << index << "] = " << id
               << ": must be in [0, " << max_april_tag_36h11_id << "]\n";
    }
  }
  if (!(min_bit_contrast_margin >= 0.0F && min_bit_contrast_margin <= 1.0F)) {
    problems << name_prefix << "min_bit_contrast_margin = " << min_bit_contrast_margin
             << ": must be in [0, 1]\n";
  }
  return problems.str();
}

}  // namespace ppmt::fiducial
