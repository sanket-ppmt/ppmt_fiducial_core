#pragma once

#include <string>
#include <variant>

#include "ppmt_fiducial_core/configs/aruco_nano_config.hpp"
#include "ppmt_fiducial_core/configs/config_source_interface.hpp"
#include "ppmt_fiducial_core/configs/preprocess_config.hpp"
#include "ppmt_fiducial_core/configs/tag_filter_config.hpp"

namespace ppmt::fiducial
{

// Pixel format the pipeline accepts; anything else is rejected at startup.
enum class ImageEncoding
{
  Bgr8,
  Mono8,
};

// Which algorithm finds the tags, with its parameters.
// - The type held is the choice; only that algorithm's parameters exist.
// - New algorithm: add its config type here (AprilTagLibraryConfig later).
using FiducialDetectorAlgorithmConfig = std::variant<ArucoNanoConfig>;

struct FiducialDetectorConfig
{
  ImageEncoding input_encoding = ImageEncoding::Bgr8;
  PreprocessConfig preprocess_config;
  FiducialDetectorAlgorithmConfig algorithm_config;
  TagFilterConfig tag_filter_config;

  // Debug: keep every rejected candidate with its corners (allocates per frame).
  bool collect_rejected_candidates = false;

  // Reads the whole config, all sub-configs included.
  // - "input_encoding": "bgr8" | "mono8".
  // - "algorithm": selects the algorithm, e.g. "aruco_nano"; its fields are read from
  //   "algorithm_config.<algorithm>.". Unknown name throws, listing the valid names.
  // - "preprocess_config.", "tag_filter_config.": their fields.
  // - "collect_rejected_candidates".
  void loadFromConfigSource(ConfigSourceInterface & config_source,
                            const std::string & name_prefix = "");

  // Checks every field, all sub-configs included.
  // - Returns "" if valid; otherwise one line per problem: name, value, allowed range.
  //   e.g. "algorithm_config.aruco_nano.threshold_window_size = 14: must be odd and >= 3"
  std::string validate(const std::string & name_prefix = "") const;
};

}  // namespace ppmt::fiducial
