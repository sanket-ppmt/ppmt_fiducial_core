// Runs the original aruco_nano on every PNG in a folder and writes what it detects as the
// oracle (reference) output: <out_dir>/<name>.yaml with the ids and the 4 corners per marker,
// in the order aruco_nano returns them, plus <out_dir>/<name>.png with the detected boxes drawn
// by aruco_nano's own Marker::draw (filled red square = corner 0, then green = 1, blue = 2).
// Usage: ppmt_fiducial_oracle <in_dir> <out_dir>
#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "aruco_nano.h"

namespace fs = std::filesystem;

namespace
{

constexpr const char * kArucoNanoCommit = "c99989bcab795ffa65d9e12df9de15f27677a835";

void writeOracle(const std::string & path, const aruco_nano::DetectorParameters & p,
                 const std::vector<aruco_nano::Marker> & markers)
{
  cv::FileStorage out(path, cv::FileStorage::WRITE);
  out << "aruco_nano_commit" << kArucoNanoCommit;
  out << "dictionary" << "APRILTAG_36h11";
  out << "params" << "{";
  out << "boxFilterSize" << p.boxFilterSize << "thres" << p.thres << "minSize" << p.minSize;
  out << "maxAttemptsPerCandidate" << p.maxAttemptsPerCandidate;
  out << "maxTimesRevisited" << p.maxTimesRevisited;
  out << "errorCorrectionRate" << p.errorCorrectionRate;
  out << "}";
  out << "markers" << "[";
  for (const auto & m : markers) {
    out << "{" << "id" << m.id;
    out << "corners" << cv::Mat(static_cast<const std::vector<cv::Point2f> &>(m)).reshape(1);
    out << "}";
  }
  out << "]";
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::fprintf(stderr, "usage: %s <in_dir> <out_dir>\n", argv[0]);
    return 1;
  }
  const fs::path in_dir = argv[1];
  const fs::path out_dir = argv[2];

  std::vector<fs::path> images;
  for (const auto & e : fs::directory_iterator(in_dir)) {
    if (e.path().extension() == ".png") {
      images.push_back(e.path());
    }
  }
  std::sort(images.begin(), images.end());

  aruco_nano::DetectorParameters params;
  params.dicts = {cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11)};

  for (const auto & img_path : images) {
    const cv::Mat img = cv::imread(img_path.string(), cv::IMREAD_COLOR);
    if (img.empty()) {
      std::fprintf(stderr, "cannot read %s\n", img_path.c_str());
      return 1;
    }
    const auto markers = aruco_nano::MarkerDetector::detect(img, params);
    const std::string name = img_path.stem().string();
    writeOracle((out_dir / (name + ".yaml")).string(), params, markers);

    cv::Mat boxes = img.clone();
    for (const auto & m : markers) {
      m.draw(boxes, cv::Scalar(0, 255, 0));
    }
    cv::imwrite((out_dir / (name + ".png")).string(), boxes);
    std::printf("%s: %zu marker(s)\n", name.c_str(), markers.size());
  }
  return 0;
}
