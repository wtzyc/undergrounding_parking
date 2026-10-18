#include "parking_lot_perception/pillar_scan_detector.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace parking_lot_perception
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kBeamCount = 360U;

sensor_msgs::msg::LaserScan make_scan()
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "test_lidar";
  scan.angle_min = static_cast<float>(-kPi);
  scan.angle_increment = static_cast<float>(2.0 * kPi / static_cast<double>(kBeamCount));
  scan.range_min = 0.4F;
  scan.range_max = 15.0F;
  scan.ranges.assign(kBeamCount, std::numeric_limits<float>::infinity());
  return scan;
}

bool ray_box_intersection(
  double angle, double center_x, double center_y, double half_width, double & range)
{
  const double direction_x = std::cos(angle);
  const double direction_y = std::sin(angle);
  double near_distance = -std::numeric_limits<double>::infinity();
  double far_distance = std::numeric_limits<double>::infinity();
  const double minimums[2] = {center_x - half_width, center_y - half_width};
  const double maximums[2] = {center_x + half_width, center_y + half_width};
  const double directions[2] = {direction_x, direction_y};

  for (std::size_t axis = 0; axis < 2U; ++axis) {
    if (std::abs(directions[axis]) < 1e-9) {
      if (minimums[axis] > 0.0 || maximums[axis] < 0.0) return false;
      continue;
    }
    double first = minimums[axis] / directions[axis];
    double second = maximums[axis] / directions[axis];
    if (first > second) std::swap(first, second);
    near_distance = std::max(near_distance, first);
    far_distance = std::min(far_distance, second);
  }
  if (far_distance < std::max(near_distance, 0.0)) return false;
  range = near_distance > 0.0 ? near_distance : far_distance;
  return range >= 0.4 && range <= 15.0;
}

void add_square_pillar(
  sensor_msgs::msg::LaserScan & scan, double center_x, double center_y,
  double width = 1.2)
{
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double angle = static_cast<double>(scan.angle_min) +
      static_cast<double>(i) * static_cast<double>(scan.angle_increment);
    double range = 0.0;
    if (ray_box_intersection(angle, center_x, center_y, width * 0.5, range)) {
      scan.ranges[i] = static_cast<float>(range);
    }
  }
}

TEST(PillarScanDetector, FindsPillarAheadAndEstimatesCenterUsingKnownSize)
{
  auto scan = make_scan();
  add_square_pillar(scan, 5.0, 0.0);

  PillarScanDetector detector;
  const auto result = detector.detect(scan);

  ASSERT_EQ(result.status, "ok");
  ASSERT_EQ(result.candidates.size(), 1U);
  EXPECT_NEAR(result.candidates.front().x, 5.0, 0.12);
  EXPECT_NEAR(result.candidates.front().y, 0.0, 0.08);
  EXPECT_GT(result.candidates.front().confidence, 0.5);
}

TEST(PillarScanDetector, PreservesLeftAndRightRelativeDirections)
{
  PillarScanDetector detector;
  auto left_scan = make_scan();
  add_square_pillar(left_scan, 0.0, 5.0);
  const auto left = detector.detect(left_scan);
  ASSERT_EQ(left.candidates.size(), 1U);
  EXPECT_GT(left.candidates.front().y, 0.0);

  auto right_scan = make_scan();
  add_square_pillar(right_scan, 0.0, -5.0);
  const auto right = detector.detect(right_scan);
  ASSERT_EQ(right.candidates.size(), 1U);
  EXPECT_LT(right.candidates.front().y, 0.0);
}

TEST(PillarScanDetector, JoinsClusterAcrossTheNegativePiPositivePiBoundary)
{
  auto scan = make_scan();
  add_square_pillar(scan, -5.0, 0.1);

  PillarScanDetector detector;
  const auto result = detector.detect(scan);

  ASSERT_EQ(result.candidates.size(), 1U);
  EXPECT_LT(result.candidates.front().x, 0.0);
  EXPECT_NEAR(result.candidates.front().x, -5.0, 0.15);
  EXPECT_NEAR(result.candidates.front().y, 0.1, 0.15);
}

TEST(PillarScanDetector, RejectsLongWallAndReportsEmptyOrInvalidScans)
{
  auto wall_scan = make_scan();
  for (std::size_t i = 0; i < wall_scan.ranges.size(); ++i) {
    const double angle = static_cast<double>(wall_scan.angle_min) +
      static_cast<double>(i) * static_cast<double>(wall_scan.angle_increment);
    const double cosine = std::cos(angle);
    if (cosine <= 0.0) continue;
    const double y = 4.0 * std::tan(angle);
    if (std::abs(y) <= 2.0) wall_scan.ranges[i] = static_cast<float>(4.0 / cosine);
  }

  PillarScanDetector detector;
  const auto wall_result = detector.detect(wall_scan);
  EXPECT_EQ(wall_result.status, "no_candidates");

  auto empty_scan = make_scan();
  const auto empty_result = detector.detect(empty_scan);
  EXPECT_EQ(empty_result.status, "invalid_scan");
  EXPECT_EQ(empty_result.reason, "no_valid_ranges");

  auto malformed_scan = make_scan();
  malformed_scan.angle_increment = 0.0F;
  const auto malformed_result = detector.detect(malformed_scan);
  EXPECT_EQ(malformed_result.status, "invalid_scan");
  EXPECT_EQ(malformed_result.reason, "invalid_scan_metadata");
}

TEST(PillarScanDetector, KeepsPartiallyVisiblePillarAsLowerConfidenceCandidate)
{
  auto scan = make_scan();
  add_square_pillar(scan, 4.0, 0.0);
  // Hide roughly half of the visible face to represent an occluded return.
  for (std::size_t i = 180U; i < 360U; ++i) {
    const double angle = static_cast<double>(scan.angle_min) +
      static_cast<double>(i) * static_cast<double>(scan.angle_increment);
    if (std::abs(angle) > 0.08 && std::abs(angle) < 0.18) {
      scan.ranges[i] = std::numeric_limits<float>::infinity();
    }
  }

  PillarScanDetector detector;
  const auto partial = detector.detect(scan);

  ASSERT_FALSE(partial.candidates.empty());
  EXPECT_TRUE(partial.candidates.front().partially_observed);
  EXPECT_LT(partial.candidates.front().confidence, 0.85);
}

}  // namespace
}  // namespace parking_lot_perception
