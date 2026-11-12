#pragma once
// ConfigSourceInterface backed by a std::map, for tests: set values by name, then load.

#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include "ppmt_fiducial_core/configs/config_source_interface.hpp"

namespace ppmt::fiducial
{

class MapConfigSource : public ConfigSourceInterface
{
public:
  using Value = std::variant<bool, std::int64_t, double, std::string, std::vector<std::int64_t>>;

  void set(const std::string & name, Value value) { values_[name] = std::move(value); }

  void read(const std::string & name, bool & value) override { readAs(name, value); }
  void read(const std::string & name, std::int64_t & value) override { readAs(name, value); }
  void read(const std::string & name, double & value) override { readAs(name, value); }
  void read(const std::string & name, std::string & value) override { readAs(name, value); }
  void read(const std::string & name, std::vector<std::int64_t> & value) override
  {
    readAs(name, value);
  }

private:
  template <typename T>
  void readAs(const std::string & name, T & value) const
  {
    const auto found = values_.find(name);
    if (found == values_.end()) {
      return;
    }
    if (!std::holds_alternative<T>(found->second)) {
      throw std::invalid_argument(name + ": stored with a different type than requested");
    }
    value = std::get<T>(found->second);
  }

  std::map<std::string, Value> values_;
};

}  // namespace ppmt::fiducial
