#pragma once

#include "parking_lot_core/map.hpp"

#include <vector>

namespace parking_lot_core
{

// Parameters for the deterministic first-stage road-node generator. The
// generator creates nodes only; it deliberately does not create graph edges.
struct VirtualRoadNodeConfig
{
  double slot_lane_extension_m{2.07};
  double pillar_extension_m{2.0};
  double lane_center_x{0.0};
  double lane_center_y{0.0};
  double pillar_slot_adjacency_max_distance_m{8.0};
  double obstacle_clearance_m{0.10};
  double duplicate_distance_m{0.25};
};

std::vector<RoadNode> generate_virtual_road_nodes(
  const ParkingMap & map, const VirtualRoadNodeConfig & config = {});

}  // namespace parking_lot_core
