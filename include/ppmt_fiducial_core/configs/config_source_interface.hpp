#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace ppmt::fiducial
{

// Where config values come from, by name.
// - The library defines it; the host implements it (e.g. RosConfigSource in the ROS node,
//   MapConfigSource in tests). The library never depends on the host.
// - Names are full dotted paths, e.g. "algorithm_config.aruco_nano.threshold_window_size".
// - Only these 5 value types, as ROS parameters; configs convert to their own field types.
class ConfigSourceInterface
{
public:
  virtual ~ConfigSourceInterface() = default;

  // Reads the value named `name` into `value`.
  // - On entry `value` holds the default; it is kept if the source has no such name.
  // - Throws std::invalid_argument if the source has the name with a different type;
  //   the message gives the name, the expected type and the found type.
  virtual void read(const std::string & name, bool & value) = 0;
  virtual void read(const std::string & name, std::int64_t & value) = 0;
  virtual void read(const std::string & name, double & value) = 0;
  virtual void read(const std::string & name, std::string & value) = 0;
  virtual void read(const std::string & name, std::vector<std::int64_t> & value) = 0;
};

}  // namespace ppmt::fiducial
