#pragma once

#include <memory>

#include "ppmt_fiducial_core/configs/fiducial_detector_config.hpp"
#include "ppmt_fiducial_core/fiducial_detector_interface.hpp"

namespace ppmt::fiducial
{

// Builds the algorithm held by `algorithm_config` (e.g. ArucoNanoConfig -> ArucoNano).
// - Takes only the algorithm config: the algorithm needs nothing else.
// - Validates it first; throws std::invalid_argument with validate()'s lines if invalid.
// - Never returns nullptr.
std::unique_ptr<FiducialDetectorInterface> createFiducialDetector(
  const FiducialDetectorAlgorithmConfig & algorithm_config);

}  // namespace ppmt::fiducial
