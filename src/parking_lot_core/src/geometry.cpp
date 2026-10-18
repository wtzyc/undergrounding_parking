#include "parking_lot_core/geometry.hpp"

#include <cmath>

namespace parking_lot_core
{

namespace
{

double required_yaw(const Pose2D & pose, const std::string & context)
{
  if (!pose.yaw.has_value() || !std::isfinite(*pose.yaw)) {
    throw CoordinateError(context + " requires a finite yaw");
  }
  return *pose.yaw;
}

}  // namespace

double normalize_angle(double angle)
{
  if (!std::isfinite(angle)) throw CoordinateError("angle must be finite");
  while (angle >= kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

double shortest_angular_distance(double from, double to)
{
  return normalize_angle(to - from);
}

Point2D transform_point(const Pose2D & transform, const Point2D & point)
{
  const double yaw = required_yaw(transform, "transform");
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Point2D{
    transform.x + c * point.x - s * point.y,
    transform.y + s * point.x + c * point.y,
  };
}

Pose2D compose_pose(const Pose2D & parent, const Pose2D & child)
{
  const auto point = transform_point(parent, Point2D{child.x, child.y});
  return Pose2D{
    point.x,
    point.y,
    normalize_angle(required_yaw(parent, "parent") + required_yaw(child, "child")),
  };
}

Pose2D inverse_pose(const Pose2D & pose)
{
  const double yaw = required_yaw(pose, "pose");
  const Pose2D inverse_rotation{0.0, 0.0, normalize_angle(-yaw)};
  const auto point = transform_point(inverse_rotation, Point2D{-pose.x, -pose.y});
  return Pose2D{point.x, point.y, inverse_rotation.yaw};
}

Pose2D transform_pose(const Pose2D & parent, const Pose2D & child)
{
  return compose_pose(parent, child);
}

Point2D sensor_to_base(const Point2D & point_in_sensor, const SensorExtrinsics & extrinsics)
{
  if (extrinsics.frame_id.empty()) throw CoordinateError("sensor frame_id must not be empty");
  return transform_point(extrinsics.pose_in_base, point_in_sensor);
}

}  // namespace parking_lot_core
