#include "parking_lot_core/virtual_road_nodes.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace parking_lot_core
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-9;

double normalize_angle(double angle)
{
  while (angle >= kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

struct Point
{
  double x;
  double y;
};

Point operator+(const Point & first, const Point & second)
{
  return {first.x + second.x, first.y + second.y};
}

Point operator-(const Point & first, const Point & second)
{
  return {first.x - second.x, first.y - second.y};
}

Point operator*(const Point & point, double scale)
{
  return {point.x * scale, point.y * scale};
}

double dot(const Point & first, const Point & second)
{
  return first.x * second.x + first.y * second.y;
}

double norm(const Point & point)
{
  return std::hypot(point.x, point.y);
}

Point rotate_into_frame(const Point & point, double yaw)
{
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  return {cosine * point.x + sine * point.y, -sine * point.x + cosine * point.y};
}

double geometry_value(const LandmarkNode & landmark, const char * key, double fallback)
{
  const auto found = landmark.geometry.find(key);
  return found == landmark.geometry.end() ? fallback : found->second;
}

struct SlotAnchorGeometry
{
  Point center;
  Point direction;
  Point point;
};

bool compute_slot_anchor_geometry(
  const LandmarkNode & landmark, const VirtualRoadNodeConfig & config,
  SlotAnchorGeometry & result)
{
  const double yaw = landmark.pose.yaw.value_or(0.0);
  result.center = {landmark.pose.x, landmark.pose.y};
  if (landmark.entrance_pose.has_value()) {
    result.direction = {
      landmark.entrance_pose->x - result.center.x,
      landmark.entrance_pose->y - result.center.y};
  } else {
    const Point long_axis{std::cos(yaw), std::sin(yaw)};
    const Point lane_vector{
      config.lane_center_x - result.center.x,
      config.lane_center_y - result.center.y};
    result.direction = long_axis * (dot(long_axis, lane_vector) >= 0.0 ? 1.0 : -1.0);
  }
  const double direction_length = norm(result.direction);
  if (direction_length < kEpsilon) return false;
  result.direction = result.direction * (1.0 / direction_length);
  const double length = geometry_value(landmark, "length", 5.8);
  result.point = result.center + result.direction * (length * 0.5 + config.slot_lane_extension_m);
  return true;
}

void validate_config(const VirtualRoadNodeConfig & config)
{
  const std::array<double, 7> values{
    config.slot_lane_extension_m, config.pillar_extension_m,
    config.lane_center_x, config.lane_center_y,
    config.pillar_slot_adjacency_max_distance_m,
    config.obstacle_clearance_m, config.duplicate_distance_m};
  if (!std::all_of(values.begin(), values.end(), [](double value) {
    return std::isfinite(value);
  })) {
    throw std::invalid_argument("virtual road-node parameters must be finite");
  }
  if (config.slot_lane_extension_m < 0.0 || config.pillar_extension_m <= 0.0 ||
    config.obstacle_clearance_m < 0.0 || config.duplicate_distance_m < 0.0 ||
    config.pillar_slot_adjacency_max_distance_m <= 0.0)
  {
    throw std::invalid_argument("invalid virtual road-node distance parameter");
  }
}

bool point_in_landmark(
  const Point & point, const LandmarkNode & landmark, double clearance)
{
  const Point delta{point.x - landmark.pose.x, point.y - landmark.pose.y};
  const Point local = rotate_into_frame(delta, landmark.pose.yaw.value_or(0.0));
  if (landmark.type == "parking_slot") {
    const double length = geometry_value(landmark, "length", 5.8);
    const double width = geometry_value(landmark, "width", 2.8);
    return std::abs(local.x) <= length * 0.5 + clearance &&
           std::abs(local.y) <= width * 0.5 + clearance;
  }
  if (landmark.type == "pillar") {
    const double side = geometry_value(landmark, "side_length", 1.2);
    return std::abs(local.x) <= side * 0.5 + clearance &&
           std::abs(local.y) <= side * 0.5 + clearance;
  }
  return false;
}

bool point_collides_with_other_landmarks(
  const Point & point, const ParkingMap & map, const std::string & ignored_id,
  double clearance)
{
  for (const auto & landmark : map.landmarks) {
    if (landmark.id == ignored_id) continue;
    if (point_in_landmark(point, landmark, clearance)) return true;
  }
  return false;
}

bool ray_hits_slot(
  const Point & start, const Point & direction, double max_distance,
  const LandmarkNode & landmark, double clearance)
{
  if (landmark.type != "parking_slot") return false;
  const double center_distance = std::hypot(
    start.x - landmark.pose.x, start.y - landmark.pose.y);
  if (center_distance > max_distance) return false;

  const double yaw = landmark.pose.yaw.value_or(0.0);
  const Point local_start = rotate_into_frame(
    {start.x - landmark.pose.x, start.y - landmark.pose.y}, yaw);
  const Point local_direction = rotate_into_frame(direction, yaw);
  const double half_length = geometry_value(landmark, "length", 5.8) * 0.5 + clearance;
  const double half_width = geometry_value(landmark, "width", 2.8) * 0.5 + clearance;
  double near_value = 0.0;
  double far_value = max_distance;
  const std::array<double, 4> p{
    -local_direction.x, local_direction.x,
    -local_direction.y, local_direction.y};
  const std::array<double, 4> q{
    local_start.x + half_length, half_length - local_start.x,
    local_start.y + half_width, half_width - local_start.y};
  bool valid_interval = true;
  for (std::size_t index = 0; index < p.size(); ++index) {
    if (std::abs(p[index]) < kEpsilon) {
      if (q[index] < 0.0) valid_interval = false;
      continue;
    }
    const double ratio = q[index] / p[index];
    if (p[index] < 0.0) near_value = std::max(near_value, ratio);
    else far_value = std::min(far_value, ratio);
    if (near_value > far_value) valid_interval = false;
  }
  return valid_interval && far_value >= 0.0 && near_value <= max_distance;
}

bool ray_points_into_adjacent_slot(
  const Point & start, const Point & direction, double max_distance,
  const ParkingMap & map, double clearance)
{
  return std::any_of(map.landmarks.begin(), map.landmarks.end(), [&](const LandmarkNode & landmark) {
    return ray_hits_slot(start, direction, max_distance, landmark, clearance);
  });
}

bool too_close_to_existing(
  const Point & point, const std::vector<RoadNode> & nodes, double distance)
{
  return std::any_of(nodes.begin(), nodes.end(), [&point, distance](const RoadNode & node) {
    return std::hypot(point.x - node.pose.x, point.y - node.pose.y) < distance;
  });
}

RoadNode make_node(
  std::string id, std::string type, std::string anchor_id,
  const Point & point, double yaw, double confidence)
{
  RoadNode node;
  node.id = std::move(id);
  node.node_type = std::move(type);
  node.pose = Pose2D{point.x, point.y, normalize_angle(yaw)};
  node.anchor_id = std::move(anchor_id);
  node.confidence = std::clamp(confidence, 0.0, 1.0);
  node.source = "virtual_road_node_generator";
  return node;
}

struct PillarNodeCandidate
{
  Point point;
  double yaw;
  const char * name;
};

}  // namespace

std::vector<RoadNode> generate_virtual_road_nodes(
  const ParkingMap & map, const VirtualRoadNodeConfig & config)
{
  validate_config(config);
  std::vector<RoadNode> nodes;

  for (const auto & landmark : map.landmarks) {
    if (landmark.type != "parking_slot") continue;
    SlotAnchorGeometry anchor;
    if (!compute_slot_anchor_geometry(landmark, config, anchor)) continue;
    if (point_collides_with_other_landmarks(
        anchor.point, map, landmark.id, config.obstacle_clearance_m) ||
      too_close_to_existing(anchor.point, nodes, config.duplicate_distance_m)) continue;
    nodes.push_back(make_node(
      "road_parking_anchor_" + landmark.id, "parking_anchor", landmark.id, anchor.point,
      std::atan2(anchor.direction.y, anchor.direction.x), landmark.confidence));
  }

  constexpr std::array<double, 4> candidate_yaws{{0.0, kPi * 0.5, kPi, -kPi * 0.5}};
  constexpr std::array<const char *, 4> candidate_names{{"east", "north", "west", "south"}};
  for (const auto & landmark : map.landmarks) {
    if (landmark.type != "pillar") continue;
    const Point center{landmark.pose.x, landmark.pose.y};
    std::vector<PillarNodeCandidate> candidates;
    for (std::size_t index = 0; index < candidate_yaws.size(); ++index) {
      const Point direction{std::cos(candidate_yaws[index]), std::sin(candidate_yaws[index])};
      const Point point = center + direction * config.pillar_extension_m;
      if (ray_points_into_adjacent_slot(
          center, direction, config.pillar_slot_adjacency_max_distance_m,
          map, config.obstacle_clearance_m) ||
        point_collides_with_other_landmarks(
          point, map, landmark.id, config.obstacle_clearance_m) ||
        too_close_to_existing(point, nodes, config.duplicate_distance_m)) continue;
      candidates.push_back({point, candidate_yaws[index], candidate_names[index]});
    }

    const LandmarkNode * adjacent_slot = nullptr;
    double adjacent_slot_distance = std::numeric_limits<double>::infinity();
    for (const auto & candidate_landmark : map.landmarks) {
      if (candidate_landmark.type != "parking_slot") continue;
      const double distance = std::hypot(
        center.x - candidate_landmark.pose.x, center.y - candidate_landmark.pose.y);
      if (distance > config.pillar_slot_adjacency_max_distance_m ||
        distance >= adjacent_slot_distance)
      {
        continue;
      }
      const bool is_adjacent = std::any_of(
        candidate_yaws.begin(), candidate_yaws.end(), [&](double candidate_yaw) {
          return ray_hits_slot(
            center, {std::cos(candidate_yaw), std::sin(candidate_yaw)},
            config.pillar_slot_adjacency_max_distance_m, candidate_landmark,
            config.obstacle_clearance_m);
        });
      if (is_adjacent) {
        adjacent_slot_distance = distance;
        adjacent_slot = &candidate_landmark;
      }
    }

    if (adjacent_slot != nullptr && !candidates.empty()) {
      const auto nearest_candidate = std::min_element(
        candidates.begin(), candidates.end(), [adjacent_slot](
          const PillarNodeCandidate & first, const PillarNodeCandidate & second) {
          const double first_distance = std::hypot(
            first.point.x - adjacent_slot->pose.x, first.point.y - adjacent_slot->pose.y);
          const double second_distance = std::hypot(
            second.point.x - adjacent_slot->pose.x, second.point.y - adjacent_slot->pose.y);
          return first_distance < second_distance;
        });
      candidates.erase(nearest_candidate);

      SlotAnchorGeometry anchor;
      if (compute_slot_anchor_geometry(*adjacent_slot, config, anchor) && !candidates.empty()) {
        const auto aligned_candidate = std::max_element(
          candidates.begin(), candidates.end(), [&anchor](
            const PillarNodeCandidate & first, const PillarNodeCandidate & second) {
            const Point first_direction{std::cos(first.yaw), std::sin(first.yaw)};
            const Point second_direction{std::cos(second.yaw), std::sin(second.yaw)};
            return dot(first_direction, anchor.direction) <
                   dot(second_direction, anchor.direction);
          });
        const double longitudinal_offset = dot(
          anchor.point - aligned_candidate->point, anchor.direction);
        aligned_candidate->point =
          aligned_candidate->point + anchor.direction * longitudinal_offset;

        const double common_distance = std::hypot(
          aligned_candidate->point.x - center.x,
          aligned_candidate->point.y - center.y);
        if (common_distance > kEpsilon) {
          for (auto & candidate : candidates) {
            if (&candidate == &(*aligned_candidate)) continue;
            const Point candidate_direction{std::cos(candidate.yaw), std::sin(candidate.yaw)};
            candidate.point = center + candidate_direction * common_distance;
          }
        }
      }
    }

    for (const auto & candidate : candidates) {
      if (point_collides_with_other_landmarks(
          candidate.point, map, landmark.id, config.obstacle_clearance_m) ||
        too_close_to_existing(candidate.point, nodes, config.duplicate_distance_m)) continue;
      nodes.push_back(make_node(
        "road_pillar_anchor_" + landmark.id + "_" + candidate.name,
        "pillar_anchor", landmark.id, candidate.point, candidate.yaw, landmark.confidence));
    }
  }
  return nodes;
}

}  // namespace parking_lot_core
