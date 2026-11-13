// AprilTag36h11Dictionary must give exactly what cv::aruco::Dictionary::identify gives
// (error correction 0, as aruco_nano uses it): same found/not found, id and rotation.
// The rotation decides corner order, so it must match, not just the id.
#include <gtest/gtest.h>

#include <cstdint>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

#include "ppmt_fiducial_core/algorithms/aruco_nano/april_tag_36h11_dictionary.hpp"

namespace ppmt::fiducial
{

namespace
{

struct Identification
{
  bool found = false;
  int id = -1;
  int rotation = -1;
};

// 6x6 bits (0/1) from a code laid out as april_tag_36h11_codes.
cv::Mat bitsFromCode(std::uint64_t code)
{
  cv::Mat bits(6, 6, CV_8UC1);
  for (int cell = 0; cell < 36; ++cell) {
    bits.at<std::uint8_t>(cell / 6, cell % 6) = (code >> (35 - cell)) & 1U;
  }
  return bits;
}

Identification identifyWithOpenCv(const cv::aruco::Dictionary & dictionary, std::uint64_t code)
{
  Identification identification;
  identification.found =
    dictionary.identify(bitsFromCode(code), identification.id, identification.rotation, 0.0);
  return identification;
}

Identification identifyWithOurs(const AprilTag36h11Dictionary & dictionary, std::uint64_t code)
{
  Identification identification;
  identification.found = dictionary.identify(code, identification.id, identification.rotation);
  return identification;
}

}  // namespace

// Every tag in every rotation: found, with OpenCV's id and rotation.
TEST(AprilTag36h11Dictionary, MatchesOpenCvForEveryCodeAndRotation)
{
  const auto opencv_dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
  const AprilTag36h11Dictionary our_dictionary;

  for (int id = 0; id < april_tag_36h11_code_count; ++id) {
    for (int rotation = 0; rotation < 4; ++rotation) {
      const std::uint64_t code = april_tag_36h11_codes[id][rotation];
      const Identification expected = identifyWithOpenCv(*opencv_dictionary, code);
      const Identification actual = identifyWithOurs(our_dictionary, code);
      ASSERT_TRUE(expected.found) << "id " << id << " rotation " << rotation;
      EXPECT_EQ(actual.found, expected.found) << "id " << id << " rotation " << rotation;
      EXPECT_EQ(actual.id, expected.id) << "id " << id << " rotation " << rotation;
      EXPECT_EQ(actual.rotation, expected.rotation) << "id " << id << " rotation " << rotation;
    }
  }
}

// Random 36-bit codes (fixed seed): same answer as OpenCV, almost always "not found".
TEST(AprilTag36h11Dictionary, MatchesOpenCvForRandomCodes)
{
  const auto opencv_dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
  const AprilTag36h11Dictionary our_dictionary;
  cv::RNG random_generator(12345);

  for (int sample = 0; sample < 5000; ++sample) {
    const std::uint64_t code =
      ((static_cast<std::uint64_t>(random_generator.next()) << 32) | random_generator.next()) &
      ((std::uint64_t{1} << 36) - 1);
    const Identification expected = identifyWithOpenCv(*opencv_dictionary, code);
    const Identification actual = identifyWithOurs(our_dictionary, code);
    EXPECT_EQ(actual.found, expected.found) << std::hex << "code 0x" << code;
    if (expected.found) {
      EXPECT_EQ(actual.id, expected.id) << std::hex << "code 0x" << code;
      EXPECT_EQ(actual.rotation, expected.rotation) << std::hex << "code 0x" << code;
    }
  }
}

}  // namespace ppmt::fiducial
