#pragma once

#include <parking_lot_core/map.hpp>

#include <string>
#include <vector>

namespace parking_lot_perception
{

struct ParkingSlotObservation
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double length{5.8};
  double width{2.8};
  double confidence{0.0};
};

struct ParkingSlotMapFusionConfig
{
  double match_distance_m{0.5};
  double match_angle_deg{15.0};
  // A border-completed observation of an already known slot can have a larger
  // center offset than the normal association gate. These limits are also used
  // to collapse duplicate tracks of the same physical rectangle.
  double duplicate_merge_distance_m{1.2};
  double duplicate_merge_angle_deg{10.0};
  double duplicate_dimension_tolerance_m{0.6};
  // An unconfirmed slot must be accepted and geometrically stable for this
  // many consecutive frames before it becomes a map landmark.
  int confirm_hits{10};
  double stability_position_tolerance_m{0.15};
  double stability_angle_tolerance_deg{5.0};
  double stability_dimension_tolerance_m{0.15};
  int max_missed_frames{5};
};

class ParkingSlotMapStore
{
public:
  ParkingSlotMapStore(
    ParkingSlotMapFusionConfig config,
    std::string map_frame,
    std::string vehicle_model_name);
  ~ParkingSlotMapStore();

  void update(const std::vector<ParkingSlotObservation> & observations);
  void load(const parking_lot_core::ParkingMap & map);

  parking_lot_core::ParkingMap snapshot() const;
  void save(const std::string & path) const;

private:
  struct Track;

  static double track_yaw(const Track & track);
  bool duplicate_geometry(
    const Track & track, const ParkingSlotObservation & observation) const;
  bool duplicate_geometry(const Track & first, const Track & second) const;
  void merge_duplicate_tracks();

  ParkingSlotMapFusionConfig config_;
  parking_lot_core::MapMetadata metadata_;
  std::vector<Track> tracks_;
  int next_id_{1};
};

}  // namespace parking_lot_perception
