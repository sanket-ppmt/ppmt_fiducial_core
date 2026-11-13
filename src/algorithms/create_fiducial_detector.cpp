#include "ppmt_fiducial_core/algorithms/create_fiducial_detector.hpp"

#include <stdexcept>
#include <string>
#include <variant>

#include "ppmt_fiducial_core/algorithms/aruco_nano/aruco_nano_fiducial_detector.hpp"

namespace ppmt::fiducial
{

namespace
{

// One overload per algorithm config type.
// - A variant type without an overload here fails to compile in createFiducialDetector().
std::unique_ptr<FiducialDetectorInterface> createFromAlgorithmConfig(const ArucoNanoConfig & config)
{
  return std::make_unique<ArucoNanoFiducialDetector>(config);
}

}  // namespace

std::unique_ptr<FiducialDetectorInterface> createFiducialDetector(
  const FiducialDetectorAlgorithmConfig & algorithm_config)
{
  return std::visit(
    [](const auto & config) -> std::unique_ptr<FiducialDetectorInterface> {
      const std::string name_prefix =
        std::string("algorithm_config.") + config.algorithm_name + ".";
      const std::string problems = config.validate(name_prefix);
      if (!problems.empty()) {
        throw std::invalid_argument("invalid fiducial detector config:\n" + problems);
      }
      return createFromAlgorithmConfig(config);
    },
    algorithm_config);
}

}  // namespace ppmt::fiducial
