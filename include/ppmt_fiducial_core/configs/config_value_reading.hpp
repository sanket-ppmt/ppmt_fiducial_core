#pragma once
// Reads config fields of any field type from a ConfigSourceInterface.
// - The source only knows bool, int64, double, string, int64 array.
// - These overloads convert to the field's own type, so every config reads fields the same way.
// - Values that do not fit the field type throw std::invalid_argument naming the config.

#include <cstdint>
#include <string>
#include <vector>

#include "ppmt_fiducial_core/configs/config_source_interface.hpp"

namespace ppmt::fiducial
{

void readConfigValue(ConfigSourceInterface & config_source, const std::string & name, bool & value);
void readConfigValue(ConfigSourceInterface & config_source, const std::string & name, int & value);
void readConfigValue(ConfigSourceInterface & config_source, const std::string & name,
                     std::uint64_t & value);
void readConfigValue(ConfigSourceInterface & config_source, const std::string & name, float & value);
void readConfigValue(ConfigSourceInterface & config_source, const std::string & name, double & value);
void readConfigValue(ConfigSourceInterface & config_source, const std::string & name,
                     std::string & value);
void readConfigValue(ConfigSourceInterface & config_source, const std::string & name,
                     std::vector<int> & value);

}  // namespace ppmt::fiducial
