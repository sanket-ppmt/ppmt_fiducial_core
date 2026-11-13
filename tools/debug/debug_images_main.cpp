// Writes one image per ArucoNano stage for every scene, to see what each stage does.
// - Runs OUR stages (the library), with the default ArucoNanoConfig.
// - Usage: ppmt_fiducial_debug_images <scene_dir> <out_dir>
// - Per scene <name>.png:
//   - <name>_1_threshold.png: binary image (white = darker than local mean).
//   - <name>_2_contours.png: kept contours, one colour each, over the dimmed image.
//   - <name>_3_quads.png: quad candidates (green), corner 0 (red), contour index.
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "ppmt_fiducial_core/algorithms/aruco_nano/contour_tracer.hpp"
#include "ppmt_fiducial_core/algorithms/aruco_nano/quad_candidates.hpp"
#include "ppmt_fiducial_core/algorithms/aruco_nano/threshold.hpp"
#include "ppmt_fiducial_core/configs/aruco_nano_config.hpp"

namespace
{

using ppmt::fiducial::ArucoNanoConfig;
using Contours = std::vector<std::vector<cv::Point>>;

// Gray image at half brightness, as BGR: the background for drawings.
cv::Mat dimmedBackground(const cv::Mat & gray_image)
{
  cv::Mat background;
  cv::cvtColor(gray_image / 2, background, cv::COLOR_GRAY2BGR);
  return background;
}

cv::Mat drawContours(const cv::Mat & gray_image, const Contours & contours)
{
  cv::Mat drawing = dimmedBackground(gray_image);
  cv::RNG colour_generator(1);  // fixed seed: same colours every run
  for (const std::vector<cv::Point> & contour : contours) {
    const cv::Vec3b colour(colour_generator.uniform(64, 256), colour_generator.uniform(64, 256),
                           colour_generator.uniform(64, 256));
    for (const cv::Point & point : contour) {
      drawing.at<cv::Vec3b>(point) = colour;
    }
  }
  return drawing;
}

cv::Mat drawQuadCandidates(const cv::Mat & gray_image,
                           const std::vector<ppmt::fiducial::aruco_nano::QuadCandidate> & quad_candidates)
{
  cv::Mat drawing = dimmedBackground(gray_image);
  for (const auto & quad_candidate : quad_candidates) {
    for (int corner = 0; corner < 4; ++corner) {
      cv::line(drawing, quad_candidate.corners[corner], quad_candidate.corners[(corner + 1) % 4],
               cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    }
    cv::circle(drawing, quad_candidate.corners[0], 3, cv::Scalar(0, 0, 255), cv::FILLED);
    cv::putText(drawing, std::to_string(quad_candidate.contour_index),
                quad_candidate.corners[0] + cv::Point2f(4.F, -4.F), cv::FONT_HERSHEY_SIMPLEX, 0.4,
                cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
  }
  return drawing;
}

void writeSceneImages(const std::filesystem::path & scene_path,
                      const std::filesystem::path & out_directory, const ArucoNanoConfig & config)
{
  cv::Mat gray_image;
  cv::cvtColor(cv::imread(scene_path.string(), cv::IMREAD_COLOR), gray_image, cv::COLOR_BGR2GRAY);

  cv::Mat binary_image;
  ppmt::fiducial::aruco_nano::computeThreshold(gray_image, config.threshold_window_size,
                                               config.threshold_offset, binary_image);
  const cv::Mat threshold_image = binary_image.clone();  // the tracer modifies binary_image

  Contours contours;
  ppmt::fiducial::aruco_nano::ContourTracer contour_tracer;
  contour_tracer.trace(binary_image, 4 * config.min_quad_side_px,
                       config.max_contour_revisit_fraction, contours);

  std::vector<ppmt::fiducial::aruco_nano::QuadCandidate> quad_candidates;
  ppmt::fiducial::aruco_nano::QuadCandidateFinder quad_candidate_finder;
  quad_candidate_finder.find(contours, config.min_quad_side_px, quad_candidates);

  const std::string name = scene_path.stem().string();
  cv::imwrite((out_directory / (name + "_1_threshold.png")).string(), threshold_image);
  cv::imwrite((out_directory / (name + "_2_contours.png")).string(),
              drawContours(gray_image, contours));
  cv::imwrite((out_directory / (name + "_3_quads.png")).string(),
              drawQuadCandidates(gray_image, quad_candidates));
  std::cout << name << ": " << contours.size() << " contours, " << quad_candidates.size()
            << " quad candidates\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cerr << "usage: " << argv[0] << " <scene_dir> <out_dir>\n";
    return 1;
  }
  const std::filesystem::path scene_directory(argv[1]);
  const std::filesystem::path out_directory(argv[2]);
  std::filesystem::create_directories(out_directory);

  const ArucoNanoConfig config;
  for (const auto & entry : std::filesystem::directory_iterator(scene_directory)) {
    if (entry.path().extension() == ".png") {
      writeSceneImages(entry.path(), out_directory, config);
    }
  }
  return 0;
}
