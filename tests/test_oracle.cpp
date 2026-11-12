// The oracle files (original aruco_nano output) are the baseline the Nano port is diffed
// against, so they must themselves be sane: every oracle marker exists in the ground truth
// (zero false positives) and its corners sit on the truth corners in our order (TL, TR, BR, BL).
// Misses are allowed: the oracle records what the original does, not what it should do.
#include <gtest/gtest.h>

#include <filesystem>
#include <string>

#include <opencv2/core.hpp>

namespace fs = std::filesystem;

namespace
{
const fs::path kTruthDir = fs::path(PPMT_DATA_DIR) / "sample" / "synthetic";
const fs::path kOracleDir = fs::path(PPMT_DATA_DIR) / "oracle" / "synthetic";
constexpr double kMaxCornerErrorPx = 1.0;
}  // namespace

TEST(Oracle, DetectionsMatchGroundTruth)
{
  ASSERT_TRUE(fs::is_directory(kOracleDir)) << "missing " << kOracleDir << ", run `make oracle`";

  int checked = 0;
  for (const auto & entry : fs::directory_iterator(kOracleDir)) {
    if (entry.path().extension() != ".yaml") {
      continue;
    }
    const std::string name = entry.path().stem().string();
    cv::FileStorage oracle(entry.path().string(), cv::FileStorage::READ);
    cv::FileStorage truth((kTruthDir / (name + ".yaml")).string(), cv::FileStorage::READ);
    ASSERT_TRUE(oracle.isOpened() && truth.isOpened()) << name;

    for (const auto & m : oracle["markers"]) {
      const int id = static_cast<int>(m["id"]);
      cv::Mat corners;
      m["corners"] >> corners;
      corners.convertTo(corners, CV_64F);

      bool found = false;
      for (const auto & t : truth["tags"]) {
        if (static_cast<int>(t["id"]) != id) {
          continue;
        }
        found = true;
        cv::Mat truth_corners;
        t["corners_tl_tr_br_bl"] >> truth_corners;
        for (int c = 0; c < 4; ++c) {
          const double err = cv::norm(corners.row(c) - truth_corners.row(c));
          EXPECT_LT(err, kMaxCornerErrorPx) << name << " id " << id << " corner " << c;
        }
      }
      EXPECT_TRUE(found) << name << ": false positive id " << id;
      ++checked;
    }
  }
  EXPECT_GT(checked, 0) << "no oracle markers found in " << kOracleDir;
}
