#include <chrono>
#include <filesystem>

#include <gtest/gtest.h>

#include "parking_lot_perception/pillar_landmark_store.hpp"

namespace parking_lot_perception
{
namespace
{

PillarLandmarkStoreConfig config()
{
  PillarLandmarkStoreConfig result;
  result.confirm_hits = 3;
  result.max_missed_frames = 2;
  result.same_frame_merge_distance_m = 0.6;
  result.duplicate_merge_distance_m = 0.8;
  return result;
}

PillarLandmarkObservation observation(double x = 3.0, double y = 1.0, double confidence = 0.8)
{
  return PillarLandmarkObservation{x, y, confidence};
}

std::filesystem::path temporary_path()
{
  const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
  return std::filesystem::temp_directory_path() /
         ("parking_lot_pillar_store_" + std::to_string(tick) + ".json");
}

TEST(PillarLandmarkStore, ConfirmsStablePositionAndWritesPillarLandmark)
{
  PillarLandmarkStore store(config(), "map", "my_orange_car");
  store.update({observation(3.00, 1.00)});
  store.update({observation(3.04, 1.01)});
  EXPECT_TRUE(store.snapshot().landmarks.empty());
  store.update({observation(3.02, 0.99)});

  const auto map = store.snapshot();
  ASSERT_EQ(map.landmarks.size(), 1U);
  const auto & pillar = map.landmarks.front();
  EXPECT_EQ(pillar.id, "pillar_1");
  EXPECT_EQ(pillar.type, "pillar");
  EXPECT_EQ(pillar.status, "confirmed");
  EXPECT_EQ(pillar.observation_count, 3);
  EXPECT_EQ(pillar.source.value(), "laser_scan_geometry");
  EXPECT_NEAR(pillar.pose.x, 3.02, 0.03);
  EXPECT_NEAR(pillar.pose.y, 1.0, 0.03);
  EXPECT_DOUBLE_EQ(pillar.geometry.at("side_length"), 1.2);
  EXPECT_TRUE(map.road_nodes.empty());
  EXPECT_TRUE(map.edges.empty());
}

TEST(PillarLandmarkStore, ResetsConfirmationWhenPositionJumps)
{
  PillarLandmarkStore store(config(), "map", "my_orange_car");
  store.update({observation(3.00, 1.00)});
  store.update({observation(3.05, 1.00)});
  store.update({observation(3.30, 1.00)});
  EXPECT_TRUE(store.snapshot().landmarks.empty());
  store.update({observation(3.32, 1.00)});
  EXPECT_TRUE(store.snapshot().landmarks.empty());
  store.update({observation(3.34, 1.00)});
  EXPECT_EQ(store.snapshot().landmarks.size(), 1U);
}

TEST(PillarLandmarkStore, TemporaryMissesDoNotDiscardTrackButLongMissesDo)
{
  PillarLandmarkStore store(config(), "map", "my_orange_car");
  store.update({observation()});
  store.update({});
  store.update({});
  store.update({observation()});
  EXPECT_TRUE(store.snapshot().landmarks.empty());
  store.update({observation()});
  EXPECT_EQ(store.snapshot().landmarks.size(), 1U);

  PillarLandmarkStore lost(config(), "map", "my_orange_car");
  lost.update({observation()});
  lost.update({});
  lost.update({});
  lost.update({});
  lost.update({observation()});
  lost.update({observation()});
  EXPECT_TRUE(lost.snapshot().landmarks.empty());
}

TEST(PillarLandmarkStore, KeepsConfirmedPillarAfterItLeavesScanAndAcrossReload)
{
  const auto path = temporary_path();
  PillarLandmarkStore source(config(), "map", "my_orange_car");
  for (int frame = 0; frame < config().confirm_hits; ++frame) {
    source.update({observation()});
  }
  source.update({});
  ASSERT_EQ(source.snapshot().landmarks.size(), 1U);
  parking_lot_core::save_map(source.snapshot(), path.string());

  PillarLandmarkStore restored(config(), "map", "my_orange_car");
  restored.load(parking_lot_core::load_map(path.string()));
  restored.update({});
  const auto map = restored.snapshot();
  ASSERT_EQ(map.landmarks.size(), 1U);
  EXPECT_EQ(map.landmarks.front().id, "pillar_1");
  EXPECT_EQ(map.landmarks.front().observation_count, config().confirm_hits);
  std::filesystem::remove(path);
}

TEST(PillarLandmarkStore, SuppressesTwoCandidatesFromOneScan)
{
  auto one_hit = config();
  one_hit.confirm_hits = 1;
  PillarLandmarkStore store(one_hit, "map", "my_orange_car");
  store.update({observation(3.0, 1.0, 0.7), observation(3.3, 1.1, 0.9)});

  const auto map = store.snapshot();
  ASSERT_EQ(map.landmarks.size(), 1U);
  EXPECT_EQ(map.landmarks.front().id, "pillar_1");
  EXPECT_NEAR(map.landmarks.front().pose.x, 3.3, 1e-9);
  EXPECT_NEAR(map.landmarks.front().pose.y, 1.1, 1e-9);
}

TEST(PillarLandmarkStore, MergesDuplicateTracksAndLoadedLandmarks)
{
  auto one_hit = config();
  one_hit.confirm_hits = 1;
  PillarLandmarkStore store(one_hit, "map", "my_orange_car");
  store.update({observation(3.0, 1.0)});
  store.update({observation(3.7, 1.0)});
  ASSERT_EQ(store.snapshot().landmarks.size(), 1U);

  parking_lot_core::ParkingMap duplicate_map;
  duplicate_map.metadata.frame_id = "map";
  duplicate_map.metadata.vehicle_model_name = "my_orange_car";
  auto first = parking_lot_core::LandmarkNode{};
  first.id = "pillar_10";
  first.type = "pillar";
  first.pose = parking_lot_core::Pose2D{5.0, 2.0, std::nullopt};
  first.geometry = {{"side_length", 1.2}};
  first.confidence = 0.8;
  first.observation_count = 10;
  first.status = "confirmed";
  auto second = first;
  second.id = "pillar_11";
  second.pose.x = 5.7;
  duplicate_map.landmarks = {first, second};

  PillarLandmarkStore restored(one_hit, "map", "my_orange_car");
  restored.load(duplicate_map);
  const auto merged = restored.snapshot();
  ASSERT_EQ(merged.landmarks.size(), 1U);
  EXPECT_EQ(merged.landmarks.front().id, "pillar_10");
  EXPECT_NEAR(merged.landmarks.front().pose.x, 5.35, 1e-9);
}

}  // namespace
}  // namespace parking_lot_perception
