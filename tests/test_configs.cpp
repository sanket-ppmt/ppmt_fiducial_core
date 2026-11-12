// Configs: every field loads under its exact name, validate() reports bad values with
// name, value and rule, and FiducialDetectorConfig routes names to its sub-configs.
#include <gtest/gtest.h>

#include <stdexcept>

#include "map_config_source.hpp"
#include "ppmt_fiducial_core/configs/fiducial_detector_config.hpp"

namespace ppmt::fiducial
{

namespace
{
const std::string aruco_nano_name_prefix = "algorithm_config.aruco_nano.";
}  // namespace

// Every field set to a non-default value must arrive; catches mistyped names.
TEST(ArucoNanoConfig, LoadsEveryFieldByName)
{
  const std::string & prefix = aruco_nano_name_prefix;
  MapConfigSource config_source;
  config_source.set(prefix + "threshold_window_size", std::int64_t{21});
  config_source.set(prefix + "threshold_offset", std::int64_t{7});
  config_source.set(prefix + "min_quad_side_px", std::int64_t{12});
  config_source.set(prefix + "max_contour_revisit_fraction", 0.25);
  config_source.set(prefix + "max_decode_attempts", std::int64_t{3});
  config_source.set(prefix + "retry_jitter_sigma_px", 0.5);
  config_source.set(prefix + "retry_jitter_seed", std::int64_t{42});

  ArucoNanoConfig config;
  config.loadFromConfigSource(config_source, prefix);

  EXPECT_EQ(config.threshold_window_size, 21);
  EXPECT_EQ(config.threshold_offset, 7);
  EXPECT_EQ(config.min_quad_side_px, 12);
  EXPECT_FLOAT_EQ(config.max_contour_revisit_fraction, 0.25F);
  EXPECT_EQ(config.max_decode_attempts, 3);
  EXPECT_FLOAT_EQ(config.retry_jitter_sigma_px, 0.5F);
  EXPECT_EQ(config.retry_jitter_seed, 42U);
}

// Defaults are valid; each bad value gives one line with full name, value and rule.
TEST(ArucoNanoConfig, ValidateReportsEachProblem)
{
  const std::string & prefix = aruco_nano_name_prefix;
  EXPECT_EQ(ArucoNanoConfig{}.validate(prefix), "");

  ArucoNanoConfig config;
  config.threshold_window_size = 14;
  config.max_decode_attempts = 0;
  EXPECT_EQ(config.validate(prefix),
            prefix + "threshold_window_size = 14: must be odd and >= 3\n" +
            prefix + "max_decode_attempts = 0: must be >= 1\n");
}

// A value that does not fit the field type throws, naming the config.
TEST(ArucoNanoConfig, OutOfRangeValueThrowsWithName)
{
  const std::string & prefix = aruco_nano_name_prefix;
  MapConfigSource config_source;
  config_source.set(prefix + "threshold_offset", std::int64_t{5'000'000'000});

  ArucoNanoConfig config;
  try {
    config.loadFromConfigSource(config_source, prefix);
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument & error) {
    EXPECT_EQ(std::string(error.what()),
              prefix + "threshold_offset = 5000000000: does not fit in a 32-bit int");
  }
}

// ---- PreprocessConfig ----

TEST(PreprocessConfig, LoadsEveryFieldByName)
{
  MapConfigSource config_source;
  config_source.set("preprocess_config.enable_clahe", true);
  config_source.set("preprocess_config.clahe_clip_limit", 3.5);
  config_source.set("preprocess_config.clahe_tile_grid_size", std::int64_t{4});

  PreprocessConfig config;
  config.loadFromConfigSource(config_source, "preprocess_config.");

  EXPECT_TRUE(config.enable_clahe);
  EXPECT_DOUBLE_EQ(config.clahe_clip_limit, 3.5);
  EXPECT_EQ(config.clahe_tile_grid_size, 4);
}

// ---- TagFilterConfig ----

TEST(TagFilterConfig, LoadsEveryFieldByName)
{
  MapConfigSource config_source;
  config_source.set("tag_filter_config.allowed_ids", std::vector<std::int64_t>{0, 3, 18});
  config_source.set("tag_filter_config.min_bit_contrast_margin", 0.3);

  TagFilterConfig config;
  config.loadFromConfigSource(config_source, "tag_filter_config.");

  EXPECT_EQ(config.allowed_ids, (std::vector<int>{0, 3, 18}));
  EXPECT_FLOAT_EQ(config.min_bit_contrast_margin, 0.3F);
}

// A bad id is reported with its index, so it can be found in a long list.
TEST(TagFilterConfig, ValidateReportsBadIdWithIndex)
{
  TagFilterConfig config;
  config.allowed_ids = {1, 900};
  EXPECT_EQ(config.validate("tag_filter_config."),
            "tag_filter_config.allowed_ids[1] = 900: must be in [0, 586]\n");
}

// ---- FiducialDetectorConfig ----

// One field per sub-config arrives through its prefix; top-level fields load too.
TEST(FiducialDetectorConfig, LoadsWholeConfigThroughPrefixes)
{
  MapConfigSource config_source;
  config_source.set("input_encoding", std::string("mono8"));
  config_source.set("algorithm", std::string("aruco_nano"));
  config_source.set("algorithm_config.aruco_nano.threshold_offset", std::int64_t{9});
  config_source.set("preprocess_config.enable_clahe", true);
  config_source.set("tag_filter_config.allowed_ids", std::vector<std::int64_t>{5});
  config_source.set("collect_rejected_candidates", true);

  FiducialDetectorConfig config;
  config.loadFromConfigSource(config_source);

  EXPECT_EQ(config.input_encoding, ImageEncoding::Mono8);
  ASSERT_TRUE(std::holds_alternative<ArucoNanoConfig>(config.algorithm_config));
  EXPECT_EQ(std::get<ArucoNanoConfig>(config.algorithm_config).threshold_offset, 9);
  EXPECT_TRUE(config.preprocess_config.enable_clahe);
  EXPECT_EQ(config.tag_filter_config.allowed_ids, (std::vector<int>{5}));
  EXPECT_TRUE(config.collect_rejected_candidates);
  EXPECT_EQ(config.validate(), "");
}

// Unknown names for the enum-like configs throw and list the valid choices.
TEST(FiducialDetectorConfig, UnknownChoiceThrowsListingValidOnes)
{
  const auto loadErrorMessage = [](const std::string & name, const std::string & value) {
    MapConfigSource config_source;
    config_source.set(name, value);
    FiducialDetectorConfig config;
    try {
      config.loadFromConfigSource(config_source);
    } catch (const std::invalid_argument & error) {
      return std::string(error.what());
    }
    return std::string("no exception");
  };

  EXPECT_EQ(loadErrorMessage("algorithm", "apriltag"),
            "algorithm = 'apriltag': must be one of: aruco_nano");
  EXPECT_EQ(loadErrorMessage("input_encoding", "rgb8"),
            "input_encoding = 'rgb8': must be one of: bgr8, mono8");
}

// Problems from every sub-config are collected, each with its full name.
TEST(FiducialDetectorConfig, ValidateCollectsProblemsFromAllSubConfigs)
{
  FiducialDetectorConfig config;
  config.preprocess_config.clahe_tile_grid_size = 0;
  std::get<ArucoNanoConfig>(config.algorithm_config).min_quad_side_px = 0;
  config.tag_filter_config.min_bit_contrast_margin = 2.0F;

  EXPECT_EQ(config.validate(),
            "preprocess_config.clahe_tile_grid_size = 0: must be >= 1\n"
            "algorithm_config.aruco_nano.min_quad_side_px = 0: must be >= 1\n"
            "tag_filter_config.min_bit_contrast_margin = 2: must be in [0, 1]\n");
}

}  // namespace ppmt::fiducial
