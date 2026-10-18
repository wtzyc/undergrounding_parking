#include "gtest/gtest.h"

#include "parking_lot_core/virtual_road_nodes.hpp"

#include <cmath>
#include <set>

namespace parking_lot_core
{
namespace
{

LandmarkNode slot(
  const std::string & id, double x, double y, double yaw = 0.0,
  double length = 5.8, double width = 2.8)
{
  LandmarkNode landmark;
  landmark.id = id;
  landmark.type = "parking_slot";
  landmark.pose = Pose2D{x, y, yaw};
  landmark.geometry = {{"length", length}, {"width", width}};
  landmark.confidence = 0.9;
  landmark.status = "confirmed";
  return landmark;
}

LandmarkNode pillar(const std::string & id, double x, double y)
{
  LandmarkNode landmark;
  landmark.id = id;
  landmark.type = "pillar";
  landmark.pose = Pose2D{x, y, 0.0};
  landmark.geometry = {{"side_length", 1.2}};
  landmark.confidence = 0.8;
  landmark.status = "confirmed";
  return landmark;
}

TEST(VirtualRoadNodes, ExtendsSlotTowardConfiguredLaneCenter)
{
  ParkingMap map;
  map.landmarks.push_back(slot("slot_1", 0.0, 6.1, 1.5707963267948966));

  VirtualRoadNodeConfig config;
  config.slot_lane_extension_m = 1.5;
  config.lane_center_y = 0.0;
  const auto nodes = generate_virtual_road_nodes(map, config);

  ASSERT_EQ(nodes.size(), 1U);
  EXPECT_EQ(nodes.front().id, "road_parking_anchor_slot_1");
  EXPECT_EQ(nodes.front().node_type, "parking_anchor");
  EXPECT_EQ(nodes.front().anchor_id.value(), "slot_1");
  EXPECT_NEAR(nodes.front().pose.x, 0.0, 1e-9);
  EXPECT_NEAR(nodes.front().pose.y, 1.7, 1e-9);
  EXPECT_NEAR(nodes.front().pose.yaw.value(), -1.5707963267948966, 1e-9);
}

TEST(VirtualRoadNodes, EntrancePoseOverridesLaneCenterDirection)
{
  ParkingMap map;
  auto landmark = slot("slot_1", 0.0, 0.0, 0.0);
  landmark.entrance_pose = Pose2D{0.0, -1.0, 0.0};
  map.landmarks.push_back(landmark);

  VirtualRoadNodeConfig config;
  config.slot_lane_extension_m = 1.0;
  const auto nodes = generate_virtual_road_nodes(map, config);

  ASSERT_EQ(nodes.size(), 1U);
  EXPECT_NEAR(nodes.front().pose.x, 0.0, 1e-9);
  EXPECT_NEAR(nodes.front().pose.y, -3.9, 1e-9);
  EXPECT_NEAR(nodes.front().pose.yaw.value(), -1.5707963267948966, 1e-9);
}

TEST(VirtualRoadNodes, BlocksPillarDirectionPointingIntoParkingSlot)
{
  ParkingMap map;
  map.landmarks.push_back(pillar("pillar_1", 3.0, 0.0));
  map.landmarks.push_back(slot("slot_1", 4.0, 5.2, 1.5707963267948966));

  VirtualRoadNodeConfig config;
  config.slot_lane_extension_m = 1.5;
  config.pillar_extension_m = 3.0;
  const auto nodes = generate_virtual_road_nodes(map, config);

  ASSERT_EQ(nodes.size(), 1U + 2U);
  int pillar_nodes = 0;
  std::set<std::string> pillar_ids;
  for (const auto & node : nodes) {
    if (node.node_type == "pillar_anchor") {
      ++pillar_nodes;
      pillar_ids.insert(node.id);
    }
  }
  EXPECT_EQ(pillar_nodes, 2);
  EXPECT_EQ(
    pillar_ids,
    (std::set<std::string>{
      "road_pillar_anchor_pillar_1_south",
      "road_pillar_anchor_pillar_1_west"}));
  for (const auto & node : nodes) {
    if (node.node_type == "pillar_anchor") {
      EXPECT_NEAR(
        std::hypot(node.pose.x - 3.0, node.pose.y),
        std::hypot(3.0 - 3.0, 0.8), 1e-9);
    }
  }
}

TEST(VirtualRoadNodes, DistantSlotDoesNotTriggerAdjacentPillarFiltering)
{
  ParkingMap map;
  map.landmarks.push_back(pillar("pillar_1", 0.0, 0.0));
  map.landmarks.push_back(slot("slot_1", 0.0, 10.0, 1.5707963267948966));

  VirtualRoadNodeConfig config;
  config.pillar_slot_adjacency_max_distance_m = 8.0;
  const auto nodes = generate_virtual_road_nodes(map, config);

  int pillar_nodes = 0;
  for (const auto & node : nodes) {
    if (node.node_type == "pillar_anchor") ++pillar_nodes;
  }
  EXPECT_EQ(pillar_nodes, 4);
}

TEST(VirtualRoadNodes, RepeatedGenerationIsDeterministic)
{
  ParkingMap map;
  map.landmarks.push_back(slot("slot_1", 0.0, 6.1, 1.5707963267948966));
  map.landmarks.push_back(pillar("pillar_1", 3.0, 0.0));

  const auto first = generate_virtual_road_nodes(map);
  const auto second = generate_virtual_road_nodes(map);
  ASSERT_EQ(first.size(), second.size());
  for (std::size_t index = 0; index < first.size(); ++index) {
    EXPECT_EQ(first[index].id, second[index].id);
    EXPECT_NEAR(first[index].pose.x, second[index].pose.x, 1e-12);
    EXPECT_NEAR(first[index].pose.y, second[index].pose.y, 1e-12);
  }
}

}  // namespace
}  // namespace parking_lot_core
