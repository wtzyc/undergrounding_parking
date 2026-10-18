#include "gtest/gtest.h"

#include "parking_lot_core/virtual_road_graph.hpp"

#include <cmath>

namespace parking_lot_core
{
namespace
{

RoadNode node(const std::string & id, double x, double y, double yaw)
{
  RoadNode result;
  result.id = id;
  result.node_type = "parking_anchor";
  result.pose = Pose2D{x, y, yaw};
  return result;
}

TEST(VirtualRoadGraph, ConnectsNodesWithinDistanceAndStoresRelativePose)
{
  std::vector<RoadNode> nodes{
    node("a", 0.0, 0.0, 0.0),
    node("b", 3.0, 0.0, 0.0),
    node("far", 20.0, 0.0, 0.0)};

  VirtualRoadGraphConfig config;
  config.max_neighbor_distance_m = 5.0;
  build_virtual_road_node_neighbors(nodes, config);

  ASSERT_EQ(nodes[0].neighbors.size(), 1U);
  EXPECT_EQ(nodes[0].neighbors[0].neighbor_id, "b");
  EXPECT_NEAR(nodes[0].neighbors[0].relative_pose.x, 3.0, 1e-9);
  EXPECT_NEAR(nodes[0].neighbors[0].relative_pose.y, 0.0, 1e-9);
  EXPECT_NEAR(nodes[0].neighbors[0].relative_pose.yaw.value(), 0.0, 1e-9);
  EXPECT_EQ(nodes[0].neighbors[0].direction, "forward");
  EXPECT_NEAR(nodes[0].neighbors[0].cost, 3.0, 1e-9);
  EXPECT_TRUE(nodes[1].neighbors[0].neighbor_id == "a");
  EXPECT_TRUE(nodes[2].neighbors.empty());
}

TEST(VirtualRoadGraph, UsesSourceNodeFrameForTurnDirection)
{
  std::vector<RoadNode> nodes{
    node("a", 0.0, 0.0, 0.0),
    node("b", 0.0, 3.0, 0.5)};
  VirtualRoadGraphConfig config;
  config.max_neighbor_distance_m = 4.0;
  build_virtual_road_node_neighbors(nodes, config);

  ASSERT_EQ(nodes[0].neighbors.size(), 1U);
  EXPECT_EQ(nodes[0].neighbors[0].direction, "left");
  EXPECT_NEAR(nodes[0].neighbors[0].relative_pose.yaw.value(), 0.5, 1e-9);
  ASSERT_EQ(nodes[1].neighbors.size(), 1U);
  EXPECT_EQ(nodes[1].neighbors[0].direction, "backward_right");
}

TEST(VirtualRoadGraph, BuildsForkWithoutConnectingDisconnectedBranch)
{
  std::vector<RoadNode> nodes{
    node("center", 0.0, 0.0, 0.0),
    node("east", 4.0, 0.0, 0.0),
    node("north", 0.0, 4.0, 0.0),
    node("isolated", 20.0, 20.0, 0.0)};
  VirtualRoadGraphConfig config;
  config.max_neighbor_distance_m = 4.1;
  build_virtual_road_node_neighbors(nodes, config);

  EXPECT_EQ(nodes[0].neighbors.size(), 2U);
  EXPECT_EQ(nodes[1].neighbors.size(), 1U);
  EXPECT_EQ(nodes[2].neighbors.size(), 1U);
  EXPECT_TRUE(nodes[3].neighbors.empty());
}

TEST(VirtualRoadGraph, RebuildClearsOldNeighborsAndRejectsInvalidConfig)
{
  std::vector<RoadNode> nodes{node("a", 0.0, 0.0, 0.0), node("b", 1.0, 0.0, 0.0)};
  build_virtual_road_node_neighbors(nodes);
  ASSERT_EQ(nodes[0].neighbors.size(), 1U);
  nodes[1].pose.x = 20.0;
  build_virtual_road_node_neighbors(nodes);
  EXPECT_TRUE(nodes[0].neighbors.empty());

  VirtualRoadGraphConfig invalid;
  invalid.min_neighbor_distance_m = invalid.max_neighbor_distance_m;
  EXPECT_THROW(build_virtual_road_node_neighbors(nodes, invalid), std::invalid_argument);
}

}  // namespace
}  // namespace parking_lot_core
