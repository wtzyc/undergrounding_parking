#include "parking_lot_core/virtual_road_graph.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace parking_lot_core
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

double normalize_angle(double angle)
{
  while (angle >= kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

std::string direction_name(double local_angle)
{
  constexpr std::array<const char *, 8> names{
    "forward", "forward_left", "left", "backward_left",
    "backward", "backward_right", "right", "forward_right"};
  const double wrapped = normalize_angle(local_angle);
  const double sector = 2.0 * kPi / static_cast<double>(names.size());
  int index = static_cast<int>(std::floor((wrapped + sector * 0.5) / sector));
  index %= static_cast<int>(names.size());
  if (index < 0) index += static_cast<int>(names.size());
  return names[static_cast<std::size_t>(index)];
}

void validate_config(const VirtualRoadGraphConfig & config)
{
  if (!std::isfinite(config.max_neighbor_distance_m) ||
    !std::isfinite(config.min_neighbor_distance_m) ||
    config.max_neighbor_distance_m <= 0.0 ||
    config.min_neighbor_distance_m < 0.0 ||
    config.min_neighbor_distance_m >= config.max_neighbor_distance_m)
  {
    throw std::invalid_argument("invalid virtual road graph distance parameter");
  }
}

Neighbor make_neighbor(const RoadNode & from, const RoadNode & to, double cost)
{
  const double from_yaw = from.pose.yaw.value_or(0.0);
  const double dx = to.pose.x - from.pose.x;
  const double dy = to.pose.y - from.pose.y;
  const double cosine = std::cos(from_yaw);
  const double sine = std::sin(from_yaw);
  const double local_x = cosine * dx + sine * dy;
  const double local_y = -sine * dx + cosine * dy;

  Neighbor neighbor;
  neighbor.neighbor_id = to.id;
  neighbor.relative_pose = Pose2D{
    local_x, local_y,
    normalize_angle(to.pose.yaw.value_or(0.0) - from_yaw)};
  neighbor.direction = direction_name(std::atan2(local_y, local_x));
  neighbor.cost = cost;
  return neighbor;
}

}  // namespace

void build_virtual_road_node_neighbors(
  std::vector<RoadNode> & nodes, const VirtualRoadGraphConfig & config)
{
  validate_config(config);
  for (auto & node : nodes) node.neighbors.clear();

  for (std::size_t from_index = 0; from_index < nodes.size(); ++from_index) {
    for (std::size_t to_index = 0; to_index < nodes.size(); ++to_index) {
      if (from_index == to_index) continue;
      const auto & from = nodes[from_index];
      const auto & to = nodes[to_index];
      const double distance = std::hypot(
        to.pose.x - from.pose.x, to.pose.y - from.pose.y);
      if (distance < config.min_neighbor_distance_m ||
        distance > config.max_neighbor_distance_m)
      {
        continue;
      }
      nodes[from_index].neighbors.push_back(make_neighbor(from, to, distance));
    }
    std::sort(
      nodes[from_index].neighbors.begin(), nodes[from_index].neighbors.end(),
      [](const Neighbor & first, const Neighbor & second) {
        return first.neighbor_id < second.neighbor_id;
      });
  }
}

}  // namespace parking_lot_core
