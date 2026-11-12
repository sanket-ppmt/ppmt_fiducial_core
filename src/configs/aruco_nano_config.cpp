#include "ppmt_fiducial_core/configs/aruco_nano_config.hpp"

#include <sstream>

#include "ppmt_fiducial_core/configs/config_value_reading.hpp"

namespace ppmt::fiducial
{

void ArucoNanoConfig::loadFromConfigSource(ConfigSourceInterface & config_source,
                                           const std::string & name_prefix)
{
  readConfigValue(config_source, name_prefix + "threshold_window_size", threshold_window_size);
  readConfigValue(config_source, name_prefix + "threshold_offset", threshold_offset);
  readConfigValue(config_source, name_prefix + "min_quad_side_px", min_quad_side_px);
  readConfigValue(config_source, name_prefix + "max_contour_revisit_fraction",
                  max_contour_revisit_fraction);
  readConfigValue(config_source, name_prefix + "max_decode_attempts", max_decode_attempts);
  readConfigValue(config_source, name_prefix + "retry_jitter_sigma_px", retry_jitter_sigma_px);
  readConfigValue(config_source, name_prefix + "retry_jitter_seed", retry_jitter_seed);
}

std::string ArucoNanoConfig::validate(const std::string & name_prefix) const
{
  std::ostringstream problems;
  const auto report = [&](const char * field, auto value, const char * rule) {
    problems << name_prefix << field << " = " << value << ": " << rule << "\n";
  };

  if (threshold_window_size < 3 || threshold_window_size % 2 == 0) {
    report("threshold_window_size", threshold_window_size, "must be odd and >= 3");
  }
  if (threshold_offset < 0 || threshold_offset > 255) {
    report("threshold_offset", threshold_offset, "must be in [0, 255]");
  }
  if (min_quad_side_px < 1) {
    report("min_quad_side_px", min_quad_side_px, "must be >= 1");
  }
  if (!(max_contour_revisit_fraction > 0.0F && max_contour_revisit_fraction <= 1.0F)) {
    report("max_contour_revisit_fraction", max_contour_revisit_fraction, "must be in (0, 1]");
  }
  if (max_decode_attempts < 1) {
    report("max_decode_attempts", max_decode_attempts, "must be >= 1");
  }
  if (!(retry_jitter_sigma_px >= 0.0F)) {
    report("retry_jitter_sigma_px", retry_jitter_sigma_px, "must be >= 0");
  }
  return problems.str();
}

}  // namespace ppmt::fiducial
