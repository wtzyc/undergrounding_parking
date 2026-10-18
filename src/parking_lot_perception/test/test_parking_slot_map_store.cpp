#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "parking_lot_perception/parking_slot_map_store.hpp"

namespace parking_lot_perception
{
namespace
{

ParkingSlotObservation observation(double x = 2.0, double y = -1.0, double yaw = 0.25)
{
  return ParkingSlotObservation{x, y, yaw, 5.7, 2.75, 0.9};
}

ParkingSlotMapFusionConfig config()
{
  ParkingSlotMapFusionConfig result;
  result.confirm_hits = 10;
  result.stability_position_tolerance_m = 0.15;
  result.stability_angle_tolerance_deg = 5.0;
  result.stability_dimension_tolerance_m = 0.15;
  return result;
}

std::filesystem::path temporary_path(const std::string & suffix)
{
  const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
  return std::filesystem::temp_directory_path() /
         ("parking_lot_slot_store_" + std::to_string(tick) + suffix);
}

TEST(ParkingSlotMapStore, PublishesOnlyConfirmedSlotsInUnifiedMap)
{
  ParkingSlotMapStore store(config(), "map", "my_orange_car");
  for (int frame = 0; frame < 9; ++frame) {
    store.update({observation(2.0 + 0.005 * (frame % 2), -1.0)});
    EXPECT_TRUE(store.snapshot().landmarks.empty());
  }
  store.update({observation(2.0, -1.0)});
  const auto map = store.snapshot();
  ASSERT_EQ(map.landmarks.size(), 1U);
  EXPECT_EQ(map.landmarks.front().id, "parking_slot_1");
  EXPECT_EQ(map.landmarks.front().type, "parking_slot");
  EXPECT_EQ(map.metadata.frame_id, "map");
  EXPECT_EQ(map.landmarks.front().observation_count, 10);
  EXPECT_EQ(map.landmarks.front().status, "confirmed");
  EXPECT_NEAR(map.landmarks.front().pose.x, 2.0, 0.03);
  EXPECT_NEAR(map.landmarks.front().pose.y, -1.0, 0.03);
  EXPECT_NEAR(map.landmarks.front().geometry.at("length"), 5.7, 1e-9);
  EXPECT_NEAR(map.landmarks.front().geometry.at("width"), 2.75, 1e-9);
  EXPECT_TRUE(map.road_nodes.empty());
  EXPECT_TRUE(map.edges.empty());

  const auto json = parking_lot_core::serialize_map(map);
  EXPECT_NE(json.find("\"version\""), std::string::npos);
  EXPECT_NE(json.find("\"landmarks\""), std::string::npos);
  EXPECT_NE(json.find("\"road_nodes\": []"), std::string::npos);
  EXPECT_NE(json.find("\"edges\": []"), std::string::npos);
}

TEST(ParkingSlotMapStore, MissingFrameResetsUnconfirmedStabilityCount)
{
  ParkingSlotMapStore store(config(), "map", "my_orange_car");
  store.update({observation()});
  store.update({observation()});
  store.update({observation()});
  store.update({});
  EXPECT_TRUE(store.snapshot().landmarks.empty());

  for (int frame = 0; frame < 9; ++frame) {
    store.update({observation()});
    EXPECT_TRUE(store.snapshot().landmarks.empty());
  }
  store.update({observation()});
  EXPECT_EQ(store.snapshot().landmarks.size(), 1U);
}

TEST(ParkingSlotMapStore, PositionJumpStartsANewStabilityCount)
{
  ParkingSlotMapStore store(config(), "map", "my_orange_car");
  for (int frame = 0; frame < 4; ++frame) {
    store.update({observation(2.00 + 0.02 * frame, -1.0)});
  }

  // This observation still matches the loose association window, but exceeds
  // the stricter 0.15 m stability window and therefore starts a fresh streak.
  store.update({observation(2.30, -1.0)});
  EXPECT_TRUE(store.snapshot().landmarks.empty());
  for (int frame = 0; frame < 8; ++frame) {
    store.update({observation(2.31 + 0.005 * frame, -1.0)});
  }
  EXPECT_TRUE(store.snapshot().landmarks.empty());

  store.update({observation(2.35, -1.0)});
  EXPECT_EQ(store.snapshot().landmarks.size(), 1U);
}

TEST(ParkingSlotMapStore, HeadingJumpStartsANewStabilityCount)
{
  ParkingSlotMapStore store(config(), "map", "my_orange_car");
  for (int frame = 0; frame < 4; ++frame) {
    store.update({observation()});
  }

  auto changed_heading = observation();
  changed_heading.yaw = 0.40;
  store.update({changed_heading});
  EXPECT_TRUE(store.snapshot().landmarks.empty());

  for (int frame = 0; frame < 8; ++frame) {
    auto stable_heading = observation();
    stable_heading.yaw = 0.41 + 0.005 * frame;
    store.update({stable_heading});
  }
  EXPECT_TRUE(store.snapshot().landmarks.empty());

  auto final_heading = observation();
  final_heading.yaw = 0.45;
  store.update({final_heading});
  EXPECT_EQ(store.snapshot().landmarks.size(), 1U);
}

TEST(ParkingSlotMapStore, DimensionJumpStartsANewStabilityCount)
{
  ParkingSlotMapStore store(config(), "map", "my_orange_car");
  for (int frame = 0; frame < 4; ++frame) {
    store.update({observation()});
  }

  auto changed_size = observation();
  changed_size.length += 0.20;
  store.update({changed_size});
  EXPECT_TRUE(store.snapshot().landmarks.empty());

  for (int frame = 0; frame < 8; ++frame) {
    store.update({changed_size});
  }
  EXPECT_TRUE(store.snapshot().landmarks.empty());
  store.update({changed_size});
  EXPECT_EQ(store.snapshot().landmarks.size(), 1U);
}

TEST(ParkingSlotMapStore, RetainsConfirmedSlotAfterVehicleLeavesItsView)
{
  ParkingSlotMapStore store(config(), "map", "my_orange_car");
  for (int frame = 0; frame < 10; ++frame) {
    store.update({observation()});
  }
  ASSERT_EQ(store.snapshot().landmarks.size(), 1U);

  for (int frame = 0; frame < config().max_missed_frames + 2; ++frame) {
    store.update({});
  }

  const auto map = store.snapshot();
  ASSERT_EQ(map.landmarks.size(), 1U);
  EXPECT_EQ(map.landmarks.front().id, "parking_slot_1");
}

TEST(ParkingSlotMapStore, TreatsYawReversedByPiAsTheSameParkingSlot)
{
  ParkingSlotMapStore store(config(), "map", "my_orange_car");
  for (int frame = 0; frame < config().confirm_hits; ++frame) {
    store.update({observation()});
  }
  ASSERT_EQ(store.snapshot().landmarks.size(), 1U);

  store.update({observation(2.03, -0.98, 0.25 + 3.14159265358979323846)});
  const auto map = store.snapshot();
  ASSERT_EQ(map.landmarks.size(), 1U);
  EXPECT_EQ(map.landmarks.front().id, "parking_slot_1");
  EXPECT_EQ(map.landmarks.front().observation_count, 11);
  EXPECT_NEAR(map.landmarks.front().pose.x, 2.0, 0.05);
  EXPECT_NEAR(map.landmarks.front().pose.yaw.value(), 0.25, 0.02);
}

TEST(ParkingSlotMapStore, AssociatesBorderCompletionOffsetWithConfirmedSlot)
{
  ParkingSlotMapStore store(config(), "map", "my_orange_car");
  for (int frame = 0; frame < config().confirm_hits; ++frame) {
    store.update({observation()});
  }

  // This exceeds the normal 0.5 m association gate but remains inside the
  // duplicate gate used for full-view versus border-completed observations.
  store.update({observation(2.8, -1.0)});
  const auto map = store.snapshot();
  ASSERT_EQ(map.landmarks.size(), 1U);
  EXPECT_EQ(map.landmarks.front().id, "parking_slot_1");
  EXPECT_EQ(map.landmarks.front().observation_count, 11);
}

TEST(ParkingSlotMapStore, KeepsAdjacentPhysicalSlotsSeparate)
{
  auto one_hit = config();
  one_hit.confirm_hits = 1;
  ParkingSlotMapStore store(one_hit, "map", "my_orange_car");
  store.update({observation(0.0, 0.0), observation(0.0, 3.0)});

  const auto map = store.snapshot();
  ASSERT_EQ(map.landmarks.size(), 2U);
  EXPECT_EQ(map.landmarks[0].id, "parking_slot_1");
  EXPECT_EQ(map.landmarks[1].id, "parking_slot_2");
}

TEST(ParkingSlotMapStore, CollapsesDuplicateConfirmedSlotsWhenLoadingOldMap)
{
  ParkingSlotMapStore store(config(), "map", "my_orange_car");
  auto map = store.snapshot();

  parking_lot_core::LandmarkNode first;
  first.id = "parking_slot_1";
  first.type = "parking_slot";
  first.pose = parking_lot_core::Pose2D{2.0, -1.0, 0.25};
  first.geometry = {{"length", 5.8}, {"width", 2.8}};
  first.confidence = 0.9;
  first.observation_count = 20;
  first.status = "confirmed";
  map.landmarks.push_back(first);

  auto duplicate = first;
  duplicate.id = "parking_slot_2";
  duplicate.pose = parking_lot_core::Pose2D{
    2.6, -0.95, 0.25 + 3.14159265358979323846};
  duplicate.observation_count = 12;
  map.landmarks.push_back(duplicate);

  store.load(map);
  const auto cleaned = store.snapshot();
  ASSERT_EQ(cleaned.landmarks.size(), 1U);
  EXPECT_EQ(cleaned.landmarks.front().id, "parking_slot_1");
  EXPECT_EQ(cleaned.landmarks.front().observation_count, 20);
}

TEST(ParkingSlotMapStore, SavesLoadsAndKeepsSlotIdentity)
{
  const auto path = temporary_path(".json");
  ParkingSlotMapStore source(config(), "map", "my_orange_car");
  for (int frame = 0; frame < 10; ++frame) {
    source.update({observation()});
  }
  source.save(path.string());

  const auto loaded = parking_lot_core::load_map(path.string());
  ASSERT_EQ(loaded.landmarks.size(), 1U);
  EXPECT_EQ(loaded.landmarks.front().id, "parking_slot_1");
  EXPECT_EQ(loaded.landmarks.front().observation_count, 10);
  EXPECT_NEAR(loaded.landmarks.front().pose.x, 2.0, 1e-9);
  EXPECT_NEAR(loaded.landmarks.front().pose.y, -1.0, 1e-9);
  EXPECT_NEAR(loaded.landmarks.front().pose.yaw.value(), 0.25, 1e-9);
  EXPECT_NEAR(loaded.landmarks.front().geometry.at("length"), 5.7, 1e-9);
  EXPECT_NEAR(loaded.landmarks.front().geometry.at("width"), 2.75, 1e-9);
  EXPECT_NEAR(loaded.landmarks.front().confidence, 0.9, 1e-9);

  ParkingSlotMapStore restarted(config(), "map", "my_orange_car");
  restarted.load(loaded);
  restarted.update({observation(2.01, -1.0)});
  const auto resumed = restarted.snapshot();
  ASSERT_EQ(resumed.landmarks.size(), 1U);
  EXPECT_EQ(resumed.landmarks.front().id, "parking_slot_1");
  EXPECT_EQ(resumed.landmarks.front().observation_count, 11);
  EXPECT_TRUE(resumed.road_nodes.empty());
  EXPECT_TRUE(resumed.edges.empty());

  std::filesystem::remove(path);
}

TEST(ParkingSlotMapStore, AllocatesIdsNotAlreadyPresentAfterLoading)
{
  ParkingSlotMapFusionConfig one_hit = config();
  one_hit.confirm_hits = 1;
  ParkingSlotMapStore store(one_hit, "map", "my_orange_car");
  auto map = store.snapshot();
  int existing_index = 0;
  for (const auto & id : {"parking_slot_1", "parking_slot_3"}) {
    parking_lot_core::LandmarkNode landmark;
    landmark.id = id;
    landmark.type = "parking_slot";
    landmark.pose = parking_lot_core::Pose2D{
      100.0 + 20.0 * static_cast<double>(existing_index++), 100.0, 0.0};
    landmark.geometry = {{"length", 5.8}, {"width", 2.8}};
    landmark.confidence = 0.8;
    landmark.observation_count = 3;
    landmark.status = "confirmed";
    map.landmarks.push_back(landmark);
  }
  store.load(map);
  store.update({observation(0.0, 0.0), observation(20.0, 20.0)});

  const auto result = store.snapshot();
  ASSERT_EQ(result.landmarks.size(), 4U);
  EXPECT_EQ(result.landmarks[2].id, "parking_slot_2");
  EXPECT_EQ(result.landmarks[3].id, "parking_slot_4");
}

TEST(ParkingSlotMapStore, RejectsMapWithRoadNodesOrPillars)
{
  ParkingSlotMapStore store(config(), "map", "my_orange_car");
  auto map = store.snapshot();
  parking_lot_core::LandmarkNode pillar;
  pillar.id = "pillar_1";
  pillar.type = "pillar";
  pillar.pose = parking_lot_core::Pose2D{0.0, 0.0, 0.0};
  pillar.status = "confirmed";
  map.landmarks.push_back(pillar);
  EXPECT_THROW(store.load(map), parking_lot_core::MapValidationError);

  map.landmarks.clear();
  parking_lot_core::RoadNode road;
  road.id = "road_1";
  road.node_type = "road";
  road.pose = parking_lot_core::Pose2D{0.0, 0.0, 0.0};
  map.road_nodes.push_back(road);
  EXPECT_THROW(store.load(map), parking_lot_core::MapValidationError);
}

TEST(ParkingSlotMapStore, SaveFailureIsReportedAsAnException)
{
  const auto blocker = temporary_path("_file");
  {
    std::ofstream stream(blocker);
    ASSERT_TRUE(stream.good());
    stream << "not a directory";
  }
  ParkingSlotMapStore store(config(), "map", "my_orange_car");
  EXPECT_THROW(
    store.save((blocker / "map.json").string()),
    std::filesystem::filesystem_error);
  std::filesystem::remove(blocker);
}

}  // namespace
}  // namespace parking_lot_perception
