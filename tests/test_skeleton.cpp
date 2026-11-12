// TEMPORARY: proves the library, OpenCV and GTest link together. Remove once real tests exist.
#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include "ppmt_fiducial_core/version.hpp"

TEST(Skeleton, BuildsAndLinks)
{
  EXPECT_EQ(ppmt::fiducial::version(), "0.1.0");

  const cv::Mat image(4, 4, CV_8UC1, cv::Scalar(7));
  EXPECT_EQ(cv::sum(image)[0], 16 * 7);
}
