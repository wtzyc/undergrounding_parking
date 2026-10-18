#include <cmath>
#include <limits>

#include "gtest/gtest.h"

#include "parking_lot_core/geometry.hpp"
#include "parking_lot_core/time_sync.hpp"

namespace parking_lot_core
{
namespace
{

TEST(Geometry, NormalizesAnglesAndComputesShortestDistance)
{
  EXPECT_NEAR(normalize_angle(3.0 * kPi), -kPi, 1e-12);
  EXPECT_NEAR(normalize_angle(-3.0 * kPi), -kPi, 1e-12);
  EXPECT_NEAR(shortest_angular_distance(3.0, -3.0), 0.283185307179586, 1e-12);
}

TEST(Geometry, TransformsPointsAndPoses)
{
  const Pose2D transform{2.0, 3.0, kPi / 2.0};
  const auto point = transform_point(transform, Point2D{1.0, 0.0});
  EXPECT_NEAR(point.x, 2.0, 1e-12);
  EXPECT_NEAR(point.y, 4.0, 1e-12);

  const auto composed = compose_pose(transform, Pose2D{1.0, 0.0, 0.0});
  EXPECT_NEAR(composed.x, 2.0, 1e-12);
  EXPECT_NEAR(composed.y, 4.0, 1e-12);
  EXPECT_NEAR(*composed.yaw, kPi / 2.0, 1e-12);

  const auto recovered = transform_point(inverse_pose(transform), point);
  EXPECT_NEAR(recovered.x, 1.0, 1e-12);
  EXPECT_NEAR(recovered.y, 0.0, 1e-12);
}

TEST(Geometry, AppliesSensorExtrinsics)
{
  const SensorExtrinsics lidar{"lidar_link", Pose2D{2.5, 0.0, 0.0}};
  const auto point = sensor_to_base(Point2D{3.0, 0.0}, lidar);
  EXPECT_NEAR(point.x, 5.5, 1e-12);
  EXPECT_NEAR(point.y, 0.0, 1e-12);
}

TEST(Geometry, RejectsInvalidPoseAndAngle)
{
  EXPECT_THROW(transform_point(Pose2D{0.0, 0.0, std::nullopt}, Point2D{}), CoordinateError);
  EXPECT_THROW(normalize_angle(std::numeric_limits<double>::quiet_NaN()), CoordinateError);
}

TEST(TimeSync, SelectsClosestSampleWithinLimit)
{
  const auto match = match_timestamp(10.0, {9.7, 9.98, 10.2}, 0.05);
  ASSERT_TRUE(match.valid());
  EXPECT_EQ(match.index, 1U);
  EXPECT_NEAR(match.sample_time, 9.98, 1e-12);
  EXPECT_NEAR(match.age_sec, 0.02, 1e-12);
}

TEST(TimeSync, ReportsUnavailableAndStaleSamples)
{
  EXPECT_EQ(match_timestamp(1.0, {}, 0.1).reason(), "empty_samples");
  EXPECT_EQ(match_timestamp(1.0, {0.0}, 0.1).reason(), "stale");
  EXPECT_EQ(match_timestamp(1.0, {std::numeric_limits<double>::quiet_NaN()}, 0.1).reason(), "no_valid_samples");
  EXPECT_EQ(match_timestamp(1.0, {1.0}, -0.1).reason(), "invalid_max_age");
}

}  // namespace
}  // namespace parking_lot_core
