#include "parking_lot_perception/parking_slot_map_store.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace parking_lot_perception
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

double normalize_angle(double angle)
{
  while (angle >= kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

double angle_difference(double first, double second)
{
  return std::abs(normalize_angle(first - second));
}

// A parking slot's long axis is an undirected line: yaw and yaw + pi describe
// the same rectangle. Treating them as different creates duplicate landmarks
// when OpenCV happens to reverse the direction of the fitted long edge.
double axis_angle_difference(double first, double second)
{
  const double directed = angle_difference(first, second);
  return std::min(directed, std::abs(kPi - directed));
}

double align_axis_angle(double angle, double reference)
{
  if (angle_difference(angle, reference) > kPi * 0.5) {
    angle = normalize_angle(angle + kPi);
  }
  return angle;
}

double geometry_value(const parking_lot_core::LandmarkNode & landmark,
  const std::string & key, double fallback)
{
  const auto it = landmark.geometry.find(key);
  return it == landmark.geometry.end() ? fallback : it->second;
}

bool valid_observation(const ParkingSlotObservation & observation)
{
  return std::isfinite(observation.x) && std::isfinite(observation.y) &&
         std::isfinite(observation.yaw) && std::isfinite(observation.length) &&
         std::isfinite(observation.width) && std::isfinite(observation.confidence) &&
         observation.length > 0.0 && observation.width > 0.0 &&
         observation.confidence >= 0.0 && observation.confidence <= 1.0;
}

struct StablePose
{
  double x;
  double y;
  double yaw;
  double length;
  double width;
};

bool stable_with_observation(
  const std::vector<StablePose> & samples,
  const ParkingSlotObservation & observation,
  const ParkingSlotMapFusionConfig & config)
{
  const double max_angle = config.stability_angle_tolerance_deg * kPi / 180.0;
  for (const auto & sample : samples) {
    if (std::hypot(sample.x - observation.x, sample.y - observation.y) >
      config.stability_position_tolerance_m ||
      axis_angle_difference(sample.yaw, observation.yaw) > max_angle)
    {
      return false;
    }
    if (std::abs(sample.length - observation.length) >
      config.stability_dimension_tolerance_m ||
      std::abs(sample.width - observation.width) >
      config.stability_dimension_tolerance_m)
    {
      return false;
    }
  }
  return true;
}

void append_stable_pose(
  std::vector<StablePose> & samples,
  const ParkingSlotObservation & observation,
  int max_samples)
{
  samples.push_back({
    observation.x, observation.y, observation.yaw,
    observation.length, observation.width});
  if (samples.size() > static_cast<std::size_t>(max_samples)) {
    samples.erase(samples.begin());
  }
}

}  // namespace

struct ParkingSlotMapStore::Track
{
  std::string id;
  double x{0.0};
  double y{0.0};
  double sum_sin{0.0};
  double sum_cos{0.0};
  double weight{0.0};
  double length{5.8};
  double width{2.8};
  double confidence{0.0};
  int hits{0};
  int consecutive_hits{0};
  int missed{0};
  bool confirmed{false};
  std::vector<StablePose> stability_samples;
};

double ParkingSlotMapStore::track_yaw(const Track & track)
{
  return std::atan2(track.sum_sin, track.sum_cos);
}

bool ParkingSlotMapStore::duplicate_geometry(
  const Track & track,
  const ParkingSlotObservation & observation) const
{
  return std::hypot(track.x - observation.x, track.y - observation.y) <=
           config_.duplicate_merge_distance_m &&
         axis_angle_difference(track_yaw(track), observation.yaw) <=
           config_.duplicate_merge_angle_deg * kPi / 180.0 &&
         std::abs(track.length - observation.length) <=
           config_.duplicate_dimension_tolerance_m &&
         std::abs(track.width - observation.width) <=
           config_.duplicate_dimension_tolerance_m;
}

bool ParkingSlotMapStore::duplicate_geometry(
  const Track & first,
  const Track & second) const
{
  ParkingSlotObservation observation;
  observation.x = second.x;
  observation.y = second.y;
  observation.yaw = track_yaw(second);
  observation.length = second.length;
  observation.width = second.width;
  return duplicate_geometry(first, observation);
}

ParkingSlotMapStore::ParkingSlotMapStore(
  ParkingSlotMapFusionConfig config,
  std::string map_frame,
  std::string vehicle_model_name)
: config_(config)
{
  if (!std::isfinite(config_.match_distance_m) || config_.match_distance_m <= 0.0) {
    throw std::invalid_argument("match_distance_m must be finite and positive");
  }
  if (!std::isfinite(config_.match_angle_deg) ||
    config_.match_angle_deg < 0.0 || config_.match_angle_deg > 180.0)
  {
    throw std::invalid_argument("match_angle_deg must be between 0 and 180");
  }
  if (!std::isfinite(config_.duplicate_merge_distance_m) ||
    config_.duplicate_merge_distance_m <= 0.0)
  {
    throw std::invalid_argument("duplicate_merge_distance_m must be finite and positive");
  }
  if (!std::isfinite(config_.duplicate_merge_angle_deg) ||
    config_.duplicate_merge_angle_deg < 0.0 || config_.duplicate_merge_angle_deg > 90.0)
  {
    throw std::invalid_argument("duplicate_merge_angle_deg must be between 0 and 90");
  }
  if (!std::isfinite(config_.duplicate_dimension_tolerance_m) ||
    config_.duplicate_dimension_tolerance_m < 0.0)
  {
    throw std::invalid_argument(
            "duplicate_dimension_tolerance_m must be finite and non-negative");
  }
  if (config_.confirm_hits < 1) {
    throw std::invalid_argument("confirm_hits must be positive");
  }
  if (!std::isfinite(config_.stability_position_tolerance_m) ||
    config_.stability_position_tolerance_m < 0.0)
  {
    throw std::invalid_argument(
            "stability_position_tolerance_m must be finite and non-negative");
  }
  if (!std::isfinite(config_.stability_angle_tolerance_deg) ||
    config_.stability_angle_tolerance_deg < 0.0 ||
    config_.stability_angle_tolerance_deg > 180.0)
  {
    throw std::invalid_argument(
            "stability_angle_tolerance_deg must be between 0 and 180");
  }
  if (!std::isfinite(config_.stability_dimension_tolerance_m) ||
    config_.stability_dimension_tolerance_m < 0.0)
  {
    throw std::invalid_argument(
            "stability_dimension_tolerance_m must be finite and non-negative");
  }
  if (config_.max_missed_frames < 0) {
    throw std::invalid_argument("max_missed_frames must be non-negative");
  }
  if (map_frame.empty()) throw std::invalid_argument("map_frame must not be empty");
  if (vehicle_model_name.empty()) {
    throw std::invalid_argument("vehicle_model_name must not be empty");
  }
  metadata_.frame_id = std::move(map_frame);
  metadata_.vehicle_model_name = std::move(vehicle_model_name);
}

ParkingSlotMapStore::~ParkingSlotMapStore() = default;

void ParkingSlotMapStore::update(
  const std::vector<ParkingSlotObservation> & observations)
{
  std::vector<bool> matched(tracks_.size(), false);
  for (const auto & observation : observations) {
    if (!valid_observation(observation)) continue;

    std::size_t best_index = tracks_.size();
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < tracks_.size(); ++index) {
      if (matched[index]) continue;
      const auto & track = tracks_[index];
      const double distance = std::hypot(track.x - observation.x, track.y - observation.y);
      const double heading = axis_angle_difference(
        std::atan2(track.sum_sin, track.sum_cos), observation.yaw);
      const bool normal_match = distance <= config_.match_distance_m &&
        heading <= config_.match_angle_deg * kPi / 180.0;
      const bool duplicate_match = duplicate_geometry(track, observation);
      if ((normal_match || duplicate_match) &&
        distance < best_distance)
      {
        best_index = index;
        best_distance = distance;
      }
    }

    if (best_index == tracks_.size()) {
      Track track;
      do {
        track.id = "parking_slot_" + std::to_string(next_id_++);
      } while (std::any_of(tracks_.begin(), tracks_.end(), [&track](const auto & existing) {
        return existing.id == track.id;
      }));
      track.x = observation.x;
      track.y = observation.y;
      track.weight = std::max(0.01, observation.confidence);
      track.sum_sin = std::sin(observation.yaw) * track.weight;
      track.sum_cos = std::cos(observation.yaw) * track.weight;
      track.length = observation.length;
      track.width = observation.width;
      track.confidence = observation.confidence;
      track.hits = 1;
      track.consecutive_hits = 1;
      append_stable_pose(track.stability_samples, observation, config_.confirm_hits);
      track.confirmed = track.consecutive_hits >= config_.confirm_hits;
      tracks_.push_back(std::move(track));
      matched.push_back(true);
      continue;
    }

    matched[best_index] = true;
    auto & track = tracks_[best_index];
    if (!track.confirmed &&
      (track.consecutive_hits == 0 ||
      !stable_with_observation(track.stability_samples, observation, config_)))
    {
      // Start a fresh consecutive stability window from this accepted frame.
      track.x = observation.x;
      track.y = observation.y;
      track.sum_sin = std::sin(observation.yaw) * std::max(0.01, observation.confidence);
      track.sum_cos = std::cos(observation.yaw) * std::max(0.01, observation.confidence);
      track.weight = std::max(0.01, observation.confidence);
      track.length = observation.length;
      track.width = observation.width;
      track.confidence = observation.confidence;
      track.consecutive_hits = 1;
      track.stability_samples.clear();
      append_stable_pose(track.stability_samples, observation, config_.confirm_hits);
      ++track.hits;
      track.missed = 0;
      continue;
    }

    const double weight = std::max(0.01, observation.confidence);
    const double aligned_yaw = align_axis_angle(observation.yaw, track_yaw(track));
    const double total_weight = track.weight + weight;
    track.x = (track.x * track.weight + observation.x * weight) / total_weight;
    track.y = (track.y * track.weight + observation.y * weight) / total_weight;
    track.sum_sin += std::sin(aligned_yaw) * weight;
    track.sum_cos += std::cos(aligned_yaw) * weight;
    track.length = (track.length * track.weight + observation.length * weight) / total_weight;
    track.width = (track.width * track.weight + observation.width * weight) / total_weight;
    track.weight = total_weight;
    track.confidence = std::max(track.confidence, observation.confidence);
    ++track.hits;
    ++track.consecutive_hits;
    track.missed = 0;
    if (!track.confirmed) {
      append_stable_pose(track.stability_samples, observation, config_.confirm_hits);
    }
    track.confirmed = track.confirmed || track.consecutive_hits >= config_.confirm_hits;
  }

  for (std::size_t index = tracks_.size(); index > 0; --index) {
    auto & track = tracks_[index - 1];
    if (index - 1 >= matched.size() || !matched[index - 1]) {
      ++track.missed;
      if (!track.confirmed) {
        track.consecutive_hits = 0;
        track.stability_samples.clear();
      }
    }
    // Unconfirmed detections are temporary tracks. Once a slot has been
    // confirmed, keep it as a map landmark even after the vehicle drives away.
    if (!track.confirmed && track.missed > config_.max_missed_frames) {
      tracks_.erase(tracks_.begin() + static_cast<std::ptrdiff_t>(index - 1));
    }
  }
  merge_duplicate_tracks();
}

void ParkingSlotMapStore::merge_duplicate_tracks()
{
  bool merged = true;
  while (merged) {
    merged = false;
    for (std::size_t first = 0; first < tracks_.size() && !merged; ++first) {
      for (std::size_t second = first + 1; second < tracks_.size(); ++second) {
        if (!duplicate_geometry(tracks_[first], tracks_[second])) continue;

        std::size_t keep = first;
        std::size_t remove = second;
        const auto stronger = [this](std::size_t lhs, std::size_t rhs) {
            if (tracks_[lhs].confirmed != tracks_[rhs].confirmed) {
              return tracks_[lhs].confirmed;
            }
            return tracks_[lhs].hits >= tracks_[rhs].hits;
          };
        if (!stronger(first, second)) {
          keep = second;
          remove = first;
        }

        Track combined = tracks_[keep];
        const Track & other = tracks_[remove];
        const double combined_yaw = track_yaw(combined);
        double other_sum_sin = other.sum_sin;
        double other_sum_cos = other.sum_cos;
        if (angle_difference(track_yaw(other), combined_yaw) > kPi * 0.5) {
          other_sum_sin = -other_sum_sin;
          other_sum_cos = -other_sum_cos;
        }
        const double total_weight = combined.weight + other.weight;
        if (total_weight > 0.0) {
          combined.x = (combined.x * combined.weight + other.x * other.weight) / total_weight;
          combined.y = (combined.y * combined.weight + other.y * other.weight) / total_weight;
          combined.length =
            (combined.length * combined.weight + other.length * other.weight) / total_weight;
          combined.width =
            (combined.width * combined.weight + other.width * other.weight) / total_weight;
        }
        combined.weight = total_weight;
        combined.sum_sin += other_sum_sin;
        combined.sum_cos += other_sum_cos;
        combined.confidence = std::max(combined.confidence, other.confidence);
        // Use max rather than sum so duplicate candidates from one image cannot
        // satisfy the multi-frame confirmation requirement early.
        combined.hits = std::max(combined.hits, other.hits);
        combined.consecutive_hits = std::max(
          combined.consecutive_hits, other.consecutive_hits);
        combined.missed = std::min(combined.missed, other.missed);
        combined.confirmed = combined.confirmed || other.confirmed;
        tracks_[keep] = std::move(combined);
        tracks_.erase(tracks_.begin() + static_cast<std::ptrdiff_t>(remove));
        merged = true;
        break;
      }
    }
  }
}

void ParkingSlotMapStore::load(const parking_lot_core::ParkingMap & map)
{
  parking_lot_core::validate_map(map);
  if (map.metadata.frame_id != metadata_.frame_id) {
    throw parking_lot_core::MapValidationError(
            "semantic map frame_id does not match configured map_frame");
  }
  if (!map.road_nodes.empty() || !map.edges.empty()) {
    throw parking_lot_core::MapValidationError(
            "parking-slot map file must not contain road_nodes or edges");
  }
  for (const auto & landmark : map.landmarks) {
    if (landmark.type != "parking_slot") {
      throw parking_lot_core::MapValidationError(
              "parking-slot map file contains unsupported landmark type: " + landmark.type);
    }
    const double length = geometry_value(landmark, "length", 5.8);
    const double width = geometry_value(landmark, "width", 2.8);
    if (!std::isfinite(length) || length <= 0.0 || !std::isfinite(width) || width <= 0.0) {
      throw parking_lot_core::MapValidationError(
              "parking-slot map has invalid dimensions: " + landmark.id);
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
    const double yaw = landmark.pose.yaw.value_or(0.0);
    track.weight = std::max(0.01, landmark.confidence);
    track.sum_sin = std::sin(yaw) * track.weight;
    track.sum_cos = std::cos(yaw) * track.weight;
    track.length = geometry_value(landmark, "length", 5.8);
    track.width = geometry_value(landmark, "width", 2.8);
    track.confidence = landmark.confidence;
    track.hits = landmark.observation_count;
    track.consecutive_hits = 0;
    track.confirmed = landmark.status == "confirmed";
    tracks_.push_back(std::move(track));
  }
  merge_duplicate_tracks();
  while (std::any_of(tracks_.begin(), tracks_.end(), [this](const auto & track) {
    return track.id == "parking_slot_" + std::to_string(next_id_);
  })) {
    ++next_id_;
  }
}

parking_lot_core::ParkingMap ParkingSlotMapStore::snapshot() const
{
  parking_lot_core::ParkingMap map;
  map.metadata = metadata_;
  for (const auto & track : tracks_) {
    if (!track.confirmed) continue;
    parking_lot_core::LandmarkNode landmark;
    landmark.id = track.id;
    landmark.type = "parking_slot";
    landmark.pose = parking_lot_core::Pose2D{
      track.x, track.y, normalize_angle(std::atan2(track.sum_sin, track.sum_cos))};
    landmark.geometry = {{"length", track.length}, {"width", track.width}};
    landmark.confidence = track.confidence;
    landmark.observation_count = track.hits;
    landmark.status = "confirmed";
    landmark.source = "bev_traditional_vision";
    map.landmarks.push_back(std::move(landmark));
  }
  return map;
}

void ParkingSlotMapStore::save(const std::string & path) const
{
  parking_lot_core::save_map(snapshot(), path);
}

}  // namespace parking_lot_perception
