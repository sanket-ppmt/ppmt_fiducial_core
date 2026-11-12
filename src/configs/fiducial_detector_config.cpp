#include "ppmt_fiducial_core/configs/fiducial_detector_config.hpp"

#include <stdexcept>
#include <variant>

#include "ppmt_fiducial_core/configs/config_value_reading.hpp"

namespace ppmt::fiducial
{

namespace
{

// ---- input_encoding: enum <-> name ----

std::string imageEncodingName(ImageEncoding image_encoding)
{
  switch (image_encoding) {
    case ImageEncoding::Bgr8:
      return "bgr8";
    case ImageEncoding::Mono8:
      return "mono8";
  }
  return "unknown";
}

ImageEncoding imageEncodingFromName(const std::string & name, const std::string & config_name)
{
  for (const ImageEncoding image_encoding : {ImageEncoding::Bgr8, ImageEncoding::Mono8}) {
    if (name == imageEncodingName(image_encoding)) {
      return image_encoding;
    }
  }
  throw std::invalid_argument(config_name + " = '" + name + "': must be one of: bgr8, mono8");
}

// ---- algorithm: name <-> variant alternative ----
// - Walks the variant's types; each provides its own `algorithm_name`.
// - A new algorithm only needs adding to FiducialDetectorAlgorithmConfig.

template <std::size_t alternative_index = 0>
bool selectAlgorithmConfig(const std::string & algorithm_name,
                           FiducialDetectorAlgorithmConfig & algorithm_config)
{
  if constexpr (alternative_index < std::variant_size_v<FiducialDetectorAlgorithmConfig>) {
    using AlgorithmConfig =
      std::variant_alternative_t<alternative_index, FiducialDetectorAlgorithmConfig>;
    if (algorithm_name == AlgorithmConfig::algorithm_name) {
      if (algorithm_config.index() != alternative_index) {
        algorithm_config.emplace<alternative_index>();  // defaults of the chosen algorithm
      }
      return true;
    }
    return selectAlgorithmConfig<alternative_index + 1>(algorithm_name, algorithm_config);
  } else {
    return false;
  }
}

template <std::size_t alternative_index = 0>
std::string validAlgorithmNames()
{
  if constexpr (alternative_index < std::variant_size_v<FiducialDetectorAlgorithmConfig>) {
    using AlgorithmConfig =
      std::variant_alternative_t<alternative_index, FiducialDetectorAlgorithmConfig>;
    const std::string rest = validAlgorithmNames<alternative_index + 1>();
    return std::string(AlgorithmConfig::algorithm_name) + (rest.empty() ? "" : ", " + rest);
  } else {
    return "";
  }
}

std::string algorithmName(const FiducialDetectorAlgorithmConfig & algorithm_config)
{
  return std::visit([](const auto & config) { return std::string(config.algorithm_name); },
                    algorithm_config);
}

}  // namespace

void FiducialDetectorConfig::loadFromConfigSource(ConfigSourceInterface & config_source,
                                                  const std::string & name_prefix)
{
  std::string input_encoding_name = imageEncodingName(input_encoding);
  readConfigValue(config_source, name_prefix + "input_encoding", input_encoding_name);
  input_encoding = imageEncodingFromName(input_encoding_name, name_prefix + "input_encoding");

  std::string algorithm_name = algorithmName(algorithm_config);
  readConfigValue(config_source, name_prefix + "algorithm", algorithm_name);
  if (!selectAlgorithmConfig(algorithm_name, algorithm_config)) {
    throw std::invalid_argument(name_prefix + "algorithm = '" + algorithm_name +
                                "': must be one of: " + validAlgorithmNames());
  }
  const std::string algorithm_prefix = name_prefix + "algorithm_config." + algorithm_name + ".";
  std::visit([&](auto & config) { config.loadFromConfigSource(config_source, algorithm_prefix); },
             algorithm_config);

  preprocess_config.loadFromConfigSource(config_source, name_prefix + "preprocess_config.");
  tag_filter_config.loadFromConfigSource(config_source, name_prefix + "tag_filter_config.");
}

std::string FiducialDetectorConfig::validate(const std::string & name_prefix) const
{
  const std::string algorithm_prefix =
    name_prefix + "algorithm_config." + algorithmName(algorithm_config) + ".";
  const std::string algorithm_problems = std::visit(
    [&](const auto & config) { return config.validate(algorithm_prefix); }, algorithm_config);

  return preprocess_config.validate(name_prefix + "preprocess_config.") + algorithm_problems +
         tag_filter_config.validate(name_prefix + "tag_filter_config.");
}

}  // namespace ppmt::fiducial
