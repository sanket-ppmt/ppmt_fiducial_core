#include "ppmt_fiducial_core/configs/config_value_reading.hpp"

#include <limits>
#include <stdexcept>

namespace ppmt::fiducial
{

namespace
{

int toInt(const std::string & name, std::int64_t value)
{
  if (value < std::numeric_limits<int>::min() || value > std::numeric_limits<int>::max()) {
    throw std::invalid_argument(name + " = " + std::to_string(value) +
                                ": does not fit in a 32-bit int");
  }
  return static_cast<int>(value);
}

}  // namespace

void readConfigValue(ConfigSourceInterface & config_source, const std::string & name, bool & value)
{
  config_source.read(name, value);
}

void readConfigValue(ConfigSourceInterface & config_source, const std::string & name, int & value)
{
  std::int64_t source_value = value;
  config_source.read(name, source_value);
  value = toInt(name, source_value);
}

void readConfigValue(ConfigSourceInterface & config_source, const std::string & name,
                     std::uint64_t & value)
{
  // Defaults above int64 max cannot be represented in the source; keep them unless overridden.
  const bool default_fits = value <= static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max());
  std::int64_t source_value = default_fits ? static_cast<std::int64_t>(value) : 0;
  config_source.read(name, source_value);
  if (source_value < 0) {
    throw std::invalid_argument(name + " = " + std::to_string(source_value) + ": must be >= 0");
  }
  value = static_cast<std::uint64_t>(source_value);
}

void readConfigValue(ConfigSourceInterface & config_source, const std::string & name, float & value)
{
  double source_value = value;
  config_source.read(name, source_value);
  value = static_cast<float>(source_value);
}

void readConfigValue(ConfigSourceInterface & config_source, const std::string & name, double & value)
{
  config_source.read(name, value);
}

void readConfigValue(ConfigSourceInterface & config_source, const std::string & name,
                     std::string & value)
{
  config_source.read(name, value);
}

void readConfigValue(ConfigSourceInterface & config_source, const std::string & name,
                     std::vector<int> & value)
{
  std::vector<std::int64_t> source_values(value.begin(), value.end());
  config_source.read(name, source_values);
  value.clear();
  for (std::size_t index = 0; index < source_values.size(); ++index) {
    value.push_back(toInt(name + "[" + std::to_string(index) + "]", source_values[index]));
  }
}

}  // namespace ppmt::fiducial
