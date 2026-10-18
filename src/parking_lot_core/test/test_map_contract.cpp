#include <fstream>
#include <limits>
#include <string>

#include "gtest/gtest.h"

#include "parking_lot_core/map.hpp"

namespace parking_lot_core
{
namespace
{

ParkingMap make_map()
{
  ParkingMap map;
  LandmarkNode landmark;
  landmark.id = "B-PS-2-1";
  landmark.type = "parking_slot";
  landmark.zone = "B";
  landmark.row = 2;
  landmark.column = 1;
  landmark.pose = Pose2D{1.0, 2.0, 0.0};
  landmark.geometry = {{"length", 5.8}, {"width", 2.8}};
  landmark.entrance_pose = Pose2D{3.0, 2.0, 0.0};
  landmark.confidence = 0.95;
  landmark.observation_count = 4;
  landmark.status = "confirmed";
  landmark.source = "test";
  map.landmarks.push_back(landmark);

  RoadNode road;
  road.id = "road-0001";
  road.node_type = "road";
  road.pose = Pose2D{3.5, 2.0, 0.0};
  road.anchor_id = landmark.id;
  road.confidence = 0.9;
  road.source = "test";
  road.neighbors.push_back(Neighbor{"road-0002", Pose2D{1.0, 0.0, 0.0}, "forward", 1.0});
  map.road_nodes.push_back(road);

  RoadNode next_road;
  next_road.id = "road-0002";
  next_road.node_type = "road";
  next_road.pose = Pose2D{4.5, 2.0, 0.0};
  next_road.confidence = 0.9;
  next_road.source = "test";
  map.road_nodes.push_back(next_road);

  map.edges.push_back(Edge{"edge-0001", "road-0001", "road-0002", Pose2D{1.0, 0.0, 0.0}, "forward", 1.0});
  return map;
}

std::string temporary_path(const std::string & suffix)
{
  return "/tmp/parking_lot_core_" + suffix;
}

TEST(MapContract, RoundTripsJson)
{
  const auto path = temporary_path("round_trip.json");
  const auto source = make_map();
  save_map(source, path);
  const auto loaded = load_map(path);

  ASSERT_EQ(loaded.landmarks.size(), 1U);
  ASSERT_EQ(loaded.road_nodes.size(), 2U);
  ASSERT_EQ(loaded.edges.size(), 1U);
  EXPECT_EQ(loaded.landmarks.front().id, "B-PS-2-1");
  EXPECT_EQ(loaded.road_nodes.front().neighbors.front().neighbor_id, "road-0002");
  EXPECT_EQ(loaded.metadata.frame_id, "map");
}

TEST(MapContract, RejectsDuplicateNodeIds)
{
  auto map = make_map();
  map.road_nodes.push_back(RoadNode{"B-PS-2-1", "road", Pose2D{0.0, 0.0, 0.0}});
  EXPECT_THROW(validate_map(map), MapValidationError);
}

TEST(MapContract, RejectsInvalidNodeType)
{
  auto map = make_map();
  map.landmarks.front().type = "wall";
  EXPECT_THROW(validate_map(map), MapValidationError);
}

TEST(MapContract, RejectsDanglingEdge)
{
  auto map = make_map();
  map.edges.front().to_id = "missing-road-node";
  EXPECT_THROW(validate_map(map), MapValidationError);
}

TEST(MapContract, RejectsInvalidCostAndPose)
{
  auto map = make_map();
  map.edges.front().cost = -1.0;
  EXPECT_THROW(validate_map(map), MapValidationError);

  map = make_map();
  map.landmarks.front().pose.x = std::numeric_limits<double>::infinity();
  EXPECT_THROW(validate_map(map), MapValidationError);
}

TEST(MapContract, RejectsSelfAndDuplicateNeighbors)
{
  auto map = make_map();
  map.road_nodes.front().neighbors.front().neighbor_id = "road-0001";
  EXPECT_THROW(validate_map(map), MapValidationError);

  map = make_map();
  map.road_nodes.front().neighbors.push_back(
    Neighbor{"road-0002", Pose2D{1.0, 0.0, 0.0}, "forward", 1.0});
  EXPECT_THROW(validate_map(map), MapValidationError);
}

TEST(MapContract, MigratesLegacyEmptyMap)
{
  const auto path = temporary_path("legacy.json");
  std::ofstream stream(path);
  stream << R"({
    "version": "1.0",
    "frame_id": "map",
    "vehicle_model_name": "my_orange_car",
    "current_area": null,
    "areas": [],
    "transitions": [],
    "visits": [],
    "telemetry": {"odom_samples": 0}
  })";
  stream.close();

  const auto loaded = load_map(path);
  EXPECT_TRUE(loaded.landmarks.empty());
  EXPECT_TRUE(loaded.road_nodes.empty());
  EXPECT_TRUE(loaded.edges.empty());
  ASSERT_TRUE(loaded.metadata.extra.count("legacy_format"));
  EXPECT_EQ(loaded.metadata.extra.at("legacy_format"), "parking_lot_area_map");
}

}  // namespace
}  // namespace parking_lot_core
