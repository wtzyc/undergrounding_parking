#include "parking_lot_perception/pillar_landmark_store.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace parking_lot_perception
{
namespace
{

struct StablePosition
{
  double x;
  double y;
};

bool valid_observation(const PillarLandmarkObservation & observation)
{
  return std::isfinite(observation.x) && std::isfinite(observation.y) &&
         std::isfinite(observation.confidence) && observation.confidence >= 0.0 &&
         observation.confidence <= 1.0;
}

bool stable_with_observation(
  const std::vector<StablePosition> & samples,
  const PillarLandmarkObservation & observation,
  double tolerance_m)
{
  return std::all_of(samples.begin(), samples.end(), [&](const StablePosition & sample) {
    return std::hypot(sample.x - observation.x, sample.y - observation.y) <= tolerance_m;
  });
}

void append_sample(
  std::vector<StablePosition> & samples,
  const PillarLandmarkObservation & observation, int max_samples)
{
  samples.push_back({observation.x, observation.y});
  if (samples.size() > static_cast<std::size_t>(max_samples)) samples.erase(samples.begin());
}

}  // namespace

struct PillarLandmarkStore::Track
{
  std::string id;
  double x{0.0};
  double y{0.0};
  double weight{0.0};
  double confidence{0.0};
  int hits{0};
  int stable_hits{0};
  int missed{0};
  bool confirmed{false};
  std::vector<StablePosition> stability_samples;
};

PillarLandmarkStore::PillarLandmarkStore(
  PillarLandmarkStoreConfig config, std::string map_frame,
  std::string vehicle_model_name)
: config_(config)
{
  if (!std::isfinite(config_.match_distance_m) || config_.match_distance_m <= 0.0 ||
    !std::isfinite(config_.same_frame_merge_distance_m) ||
    config_.same_frame_merge_distance_m < 0.0 ||
    !std::isfinite(config_.duplicate_merge_distance_m) ||
    config_.duplicate_merge_distance_m <= 0.0 ||
    !std::isfinite(config_.stability_position_tolerance_m) ||
    config_.stability_position_tolerance_m < 0.0 ||
    !std::isfinite(config_.expected_side_length_m) || config_.expected_side_length_m <= 0.0 ||
    !std::isfinite(config_.min_confidence) || config_.min_confidence < 0.0 ||
    config_.min_confidence > 1.0 || config_.confirm_hits < 1 ||
    config_.max_missed_frames < 0)
  {
    throw std::invalid_argument("invalid pillar landmark tracking configuration");
  }
  if (map_frame.empty()) throw std::invalid_argument("map_frame must not be empty");
  if (vehicle_model_name.empty()) {
    throw std::invalid_argument("vehicle_model_name must not be empty");
  }
  metadata_.frame_id = std::move(map_frame);
  metadata_.vehicle_model_name = std::move(vehicle_model_name);
}

PillarLandmarkStore::~PillarLandmarkStore() = default;

void PillarLandmarkStore::update(
  const std::vector<PillarLandmarkObservation> & observations)
{
  // A single physical pillar can be split into two LaserScan clusters when a
  // few returns are invalid. Suppress those same-frame duplicates before they
  // are allowed to create separate tracks.
  std::vector<PillarLandmarkObservation> filtered_observations;
  for (const auto & observation : observations) {
    if (!valid_observation(observation) || observation.confidence < config_.min_confidence) {
      continue;
    }
    auto duplicate = std::find_if(
      filtered_observations.begin(), filtered_observations.end(), [&](const auto & kept) {
        return std::hypot(kept.x - observation.x, kept.y - observation.y) <=
               config_.same_frame_merge_distance_m;
      });
    if (duplicate == filtered_observations.end()) {
      filtered_observations.push_back(observation);
    } else if (observation.confidence > duplicate->confidence) {
      *duplicate = observation;
    }
  }

  std::vector<bool> matched(tracks_.size(), false);
  for (const auto & observation : filtered_observations) {

    std::size_t best_index = tracks_.size();
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < tracks_.size(); ++index) {
      if (matched[index]) continue;
      const auto & track = tracks_[index];
      const double distance = std::hypot(track.x - observation.x, track.y - observation.y);
      if (distance <= config_.match_distance_m && distance < best_distance) {
        best_index = index;
        best_distance = distance;
      }
    }

    if (best_index == tracks_.size()) {
      Track track;
      do {
        track.id = "pillar_" + std::to_string(next_id_++);
      } while (std::any_of(tracks_.begin(), tracks_.end(), [&track](const auto & existing) {
        return existing.id == track.id;
      }));
      track.x = observation.x;
      track.y = observation.y;
      track.weight = std::max(0.01, observation.confidence);
      track.confidence = observation.confidence;
      track.hits = 1;
      track.stable_hits = 1;
      track.confirmed = track.stable_hits >= config_.confirm_hits;
      append_sample(track.stability_samples, observation, config_.confirm_hits);
      tracks_.push_back(std::move(track));
      matched.push_back(true);
      continue;
    }

    matched[best_index] = true;
    auto & track = tracks_[best_index];
    if (!track.confirmed && !stable_with_observation(
        track.stability_samples, observation, config_.stability_position_tolerance_m))
    {
      // A jump inside the association window starts a fresh stability window.
      track.x = observation.x;
      track.y = observation.y;
      track.weight = std::max(0.01, observation.confidence);
      track.confidence = observation.confidence;
      track.stable_hits = 1;
      track.stability_samples.clear();
      append_sample(track.stability_samples, observation, config_.confirm_hits);
      ++track.hits;
      track.missed = 0;
      continue;
    }

    const double weight = std::max(0.01, observation.confidence);
    const double total_weight = track.weight + weight;
    track.x = (track.x * track.weight + observation.x * weight) / total_weight;
    track.y = (track.y * track.weight + observation.y * weight) / total_weight;
    track.weight = total_weight;
    track.confidence = std::max(track.confidence, observation.confidence);
    ++track.hits;
    ++track.stable_hits;
    track.missed = 0;
    if (!track.confirmed) append_sample(track.stability_samples, observation, config_.confirm_hits);
    track.confirmed = track.confirmed || track.stable_hits >= config_.confirm_hits;
    if (track.confirmed) track.stability_samples.clear();
  }

  for (std::size_t index = tracks_.size(); index > 0; --index) {
    auto & track = tracks_[index - 1U];
    if (index - 1U >= matched.size() || !matched[index - 1U]) ++track.missed;
    // Short scan dropouts do not discard an unconfirmed track or reset its
    // stable-hit count. A longer disappearance starts a new confirmation run.
    if (!track.confirmed && track.missed > config_.max_missed_frames) {
      tracks_.erase(tracks_.begin() + static_cast<std::ptrdiff_t>(index - 1U));
    }
  }
  merge_duplicate_tracks();
}

void PillarLandmarkStore::merge_duplicate_tracks()
{
  for (std::size_t first = 0; first < tracks_.size(); ++first) {
    for (std::size_t second = tracks_.size(); second > first + 1U; --second) {
      const std::size_t second_index = second - 1U;
      auto & primary = tracks_[first];
      auto & duplicate = tracks_[second_index];
      if (std::hypot(primary.x - duplicate.x, primary.y - duplicate.y) >
        config_.duplicate_merge_distance_m)
      {
        continue;
      }

      const double primary_weight = std::max(0.01, primary.weight);
      const double duplicate_weight = std::max(0.01, duplicate.weight);
      const double total_weight = primary_weight + duplicate_weight;
      primary.x = (primary.x * primary_weight + duplicate.x * duplicate_weight) / total_weight;
      primary.y = (primary.y * primary_weight + duplicate.y * duplicate_weight) / total_weight;
      primary.weight = total_weight;
      primary.confidence = std::max(primary.confidence, duplicate.confidence);
      primary.hits += duplicate.hits;
      primary.stable_hits = std::max(primary.stable_hits, duplicate.stable_hits);
      primary.missed = std::min(primary.missed, duplicate.missed);
      primary.confirmed = primary.confirmed || duplicate.confirmed;
      if (primary.stability_samples.empty() && !duplicate.stability_samples.empty()) {
        primary.stability_samples = duplicate.stability_samples;
      }
      tracks_.erase(tracks_.begin() + static_cast<std::ptrdiff_t>(second_index));
    }
  }
}

void PillarLandmarkStore::load(const parking_lot_core::ParkingMap & map)
{
  parking_lot_core::validate_map(map);
  if (map.metadata.frame_id != metadata_.frame_id) {
    throw parking_lot_core::MapValidationError(
            "semantic map frame_id does not match configured map_frame");
  }
  if (!map.road_nodes.empty() || !map.edges.empty()) {
    throw parking_lot_core::MapValidationError(
            "pillar landmark map must not contain road_nodes or edges");
  }
  for (const auto & landmark : map.landmarks) {
    if (landmark.type != "pillar") {
      throw parking_lot_core::MapValidationError(
              "pillar landmark map contains unsupported type: " + landmark.type);
    }
  }

  metadata_ = map.metadata;
  tracks_.clear();
  next_id_ = 1;
  for (const auto & landmark : map.landmarks) {
    Track track;
    track.id = landmark.id;
    track.x = landmark.pose.x;
    track.y = landmark.pose.y;
    track.weight = std::max(0.01, landmark.confidence);
    track.confidence = landmark.confidence;
    track.hits = landmark.observation_count;
    track.confirmed = landmark.status == "confirmed";
    tracks_.push_back(std::move(track));
  }
  merge_duplicate_tracks();
  while (std::any_of(tracks_.begin(), tracks_.end(), [this](const auto & track) {
    return track.id == "pillar_" + std::to_string(next_id_);
  })) {
    ++next_id_;
  }
}

parking_lot_core::ParkingMap PillarLandmarkStore::snapshot() const
{
  parking_lot_core::ParkingMap map;
  map.metadata = metadata_;
  for (const auto & track : tracks_) {
    if (!track.confirmed) continue;
    parking_lot_core::LandmarkNode landmark;
    landmark.id = track.id;
    landmark.type = "pillar";
    landmark.pose = parking_lot_core::Pose2D{track.x, track.y, std::nullopt};
    landmark.geometry = {{"side_length", config_.expected_side_length_m}};
    landmark.confidence = track.confidence;
    landmark.observation_count = track.hits;
    landmark.status = "confirmed";
    landmark.source = "laser_scan_geometry";
    map.landmarks.push_back(std::move(landmark));
  }
  return map;
}

}  // namespace parking_lot_perception
