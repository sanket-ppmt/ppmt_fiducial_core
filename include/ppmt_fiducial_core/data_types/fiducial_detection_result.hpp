#pragma once

#include <vector>

#include "ppmt_fiducial_core/data_types/tag_detection.hpp"

namespace ppmt::fiducial
{

// Whether the frame could be processed.
// - detect() never throws; a bad frame is reported here with no detections.
enum class FrameStatus
{
  Ok,
  ImageEmpty,         // cv::Mat has no pixels
  ImageTypeMismatch,  // cv::Mat type does not match the configured input_encoding
};

// Everything the pipeline returns for one frame.
// - Caller owns it and passes it to every detect() call; buffers are reused.
struct FiducialDetectionResult
{
  FrameStatus frame_status = FrameStatus::Ok;

  // Tags found, after the tag filter.
  std::vector<TagDetection> detections;
};

}  // namespace ppmt::fiducial
