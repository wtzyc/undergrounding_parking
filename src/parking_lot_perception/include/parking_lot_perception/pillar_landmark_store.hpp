#ifndef PARKING_LOT_PERCEPTION__PILLAR_LANDMARK_STORE_HPP_
#define PARKING_LOT_PERCEPTION__PILLAR_LANDMARK_STORE_HPP_

#include <parking_lot_core/map.hpp>

#include <string>
#include <vector>

namespace parking_lot_perception
{

struct PillarLandmarkObservation
{
  double x{0.0};
  double y{0.0};
  double confidence{0.0};
};

struct PillarLandmarkStoreConfig
{
  double match_distance_m{0.5};
  double same_frame_merge_distance_m{0.6};
  double duplicate_merge_distance_m{0.8};
  double stability_position_tolerance_m{0.15};
  double expected_side_length_m{1.2};
  double min_confidence{0.35};
  int confirm_hits{10};
  int max_missed_frames{5};
};

// Associates vehicle-relative pillar detections in map coordinates and retains
// confirmed landmarks when they leave the current LaserScan view.
class PillarLandmarkStore
{
public:
  PillarLandmarkStore(
    PillarLandmarkStoreConfig config, std::string map_frame,
    std::string vehicle_model_name);
  ~PillarLandmarkStore();

  void update(const std::vector<PillarLandmarkObservation> & observations);
  void load(const parking_lot_core::ParkingMap & map);
  parking_lot_core::ParkingMap snapshot() const;

private:
  struct Track;

  void merge_duplicate_tracks();

  PillarLandmarkStoreConfig config_;
  parking_lot_core::MapMetadata metadata_;
  std::vector<Track> tracks_;
  int next_id_{1};
};

}  // namespace parking_lot_perception

#endif  // PARKING_LOT_PERCEPTION__PILLAR_LANDMARK_STORE_HPP_
