#pragma once

#include "parking_lot_core/map.hpp"

#include <vector>

namespace parking_lot_core
{

// Builds only the local neighbor metadata of virtual road nodes. It does not
// create ParkingMap::edges and does not perform obstacle or dynamic planning.
struct VirtualRoadGraphConfig
{
  double max_neighbor_distance_m{5.0};
  double min_neighbor_distance_m{0.05};
};

void build_virtual_road_node_neighbors(
  std::vector<RoadNode> & nodes,
  const VirtualRoadGraphConfig & config = {});

}  // namespace parking_lot_core
