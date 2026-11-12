// Writes a small reference set of synthetic scenes: <name>.png + <name>.yaml (ground truth).
// Usage: ppmt_fiducial_synth_generate <output_dir>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "synth.hpp"

using ppmt::fiducial::synth::SceneSpec;
using ppmt::fiducial::synth::TagSpec;

namespace
{

void writeTruth(const std::string & path, const SceneSpec & spec,
                const ppmt::fiducial::synth::Scene & scene)
{
  cv::FileStorage fs(path, cv::FileStorage::WRITE);
  fs << "image_width" << spec.camera.width << "image_height" << spec.camera.height;
  fs << "camera_matrix" << cv::Mat(spec.camera.K());
  fs << "tags" << "[";
  for (const auto & t : scene.truth) {
    fs << "{" << "id" << t.id << "size_m" << t.size_m << "fully_visible" << t.fully_visible;
    fs << "corners_tl_tr_br_bl" << cv::Mat(t.corners).reshape(1);
    fs << "rvec" << cv::Mat(t.rvec) << "tvec" << cv::Mat(t.tvec) << "}";
  }
  fs << "]";
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::fprintf(stderr, "usage: %s <output_dir>\n", argv[0]);
    return 1;
  }
  const std::string out = argv[1];

  std::vector<std::pair<std::string, SceneSpec>> scenes;
  SceneSpec base;  // OAK-1 preview, camera 0.6 m high, 45 deg down
  {
    SceneSpec s = base;  // tag straight ahead, near the image centre
    s.tags = {{3, 0.071, 0.60, 0.00, 0.0}};
    scenes.emplace_back("near_center", s);
  }
  {
    SceneSpec s = base;  // far, small in the image (aruco_nano's limit here is ~1.2-1.3 m)
    s.tags = {{5, 0.071, 1.10, 0.00, 0.0}};
    scenes.emplace_back("far_small", s);
  }
  {
    SceneSpec s = base;  // rotated on the floor and off to the side
    s.tags = {{1, 0.080, 0.70, 0.08, 25.0}};
    scenes.emplace_back("yawed_offset", s);
  }
  {
    SceneSpec s = base;  // two tags on the tape
    s.tags = {{0, 0.080, 0.55, 0.00, 0.0}, {7, 0.071, 0.95, 0.00, 0.0}};
    scenes.emplace_back("two_tags", s);
  }

  for (std::size_t i = 0; i < scenes.size(); ++i) {
    const auto & [name, spec] = scenes[i];
    SceneSpec seeded = spec;
    seeded.seed = i + 1;
    const auto scene = ppmt::fiducial::synth::render(seeded);
    cv::imwrite(out + "/" + name + ".png", scene.image);
    writeTruth(out + "/" + name + ".yaml", seeded, scene);
    std::printf("%s: %zu tag(s)\n", name.c_str(), scene.truth.size());
  }
  return 0;
}
