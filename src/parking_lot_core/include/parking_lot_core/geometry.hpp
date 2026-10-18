#pragma once

#include <stdexcept>
#include <string>

#include "parking_lot_core/map.hpp"

namespace parking_lot_core
{

constexpr double kPi = 3.14159265358979323846;

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct CoordinateError : public std::runtime_error
{
  explicit CoordinateError(const std::string & message)
: std::runtime_error(message) {}
};

struct CoordinateFrames
{
  std::string map_frame{"map"};
  std::string odom_frame{"odom"};
  std::string base_frame{"base_footprint"};
  std::string camera_frame{"camera_front"};
  std::string lidar_frame{"lidar_link"};
};

struct SensorExtrinsics
{
  std::string frame_id;
  Pose2D pose_in_base;
};

double normalize_angle(double angle);
double shortest_angular_distance(double from, double to);
Point2D transform_point(const Pose2D & transform, const Point2D & point);
Pose2D compose_pose(const Pose2D & parent, const Pose2D & child);
Pose2D inverse_pose(const Pose2D & pose);
Pose2D transform_pose(const Pose2D & parent, const Pose2D & child);
Point2D sensor_to_base(const Point2D & point_in_sensor, const SensorExtrinsics & extrinsics);

}  // namespace parking_lot_core
