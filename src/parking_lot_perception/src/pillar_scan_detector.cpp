#include "parking_lot_perception/pillar_scan_detector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace parking_lot_perception
{
namespace
{

struct Point2D
{
  double x;
  double y;
};

struct IndexedPoint
{
  std::size_t index;
  double range;
  Point2D point;
};

using Cluster = std::vector<IndexedPoint>;
constexpr double kPi = 3.14159265358979323846;

bool valid_range(float range, const sensor_msgs::msg::LaserScan & scan)
{
  return std::isfinite(range) && range >= scan.range_min && range <= scan.range_max;
}

double point_distance(const Point2D & lhs, const Point2D & rhs)
{
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

double cluster_gap_limit(
  double first_range, double second_range, double angle_increment,
  const PillarScanDetectorConfig & config)
{
  return std::max(
    config.cluster_gap_m,
    std::max(first_range, second_range) * std::abs(angle_increment) *
    config.angular_gap_scale);
}

double cluster_diameter(const Cluster & cluster)
{
  double diameter = 0.0;
  for (std::size_t i = 0; i < cluster.size(); ++i) {
    for (std::size_t j = i + 1; j < cluster.size(); ++j) {
      diameter = std::max(diameter, point_distance(cluster[i].point, cluster[j].point));
    }
  }
  return diameter;
}

PillarScanCandidate make_candidate(
  const Cluster & cluster, double observed_width,
  const PillarScanDetectorConfig & config)
{
  Point2D centroid{0.0, 0.0};
  for (const auto & point : cluster) {
    centroid.x += point.point.x;
    centroid.y += point.point.y;
  }
  centroid.x /= static_cast<double>(cluster.size());
  centroid.y /= static_cast<double>(cluster.size());

  // The scan sees the near surface. Move half the known pillar width along the
  // line of sight to estimate the pillar center without using Gazebo truth.
  const double radial_distance = std::hypot(centroid.x, centroid.y);
  if (radial_distance > 1e-9) {
    const double center_offset = config.expected_pillar_width_m * 0.5;
    centroid.x += centroid.x / radial_distance * center_offset;
    centroid.y += centroid.y / radial_distance * center_offset;
  }

  const double width_score = std::exp(
    -std::abs(observed_width - config.expected_pillar_width_m) /
    (config.expected_pillar_width_m * 1.4));
  const double point_score = std::min(1.0, static_cast<double>(cluster.size()) / 8.0);
  const bool partial = observed_width < config.expected_pillar_width_m * 0.75 ||
    cluster.size() < 6U;
  const double partial_penalty = partial ? 0.12 : 0.0;

  PillarScanCandidate candidate;
  candidate.x = centroid.x;
  candidate.y = centroid.y;
  candidate.observed_width_m = observed_width;
  candidate.confidence = std::clamp(
    0.55 * width_score + 0.45 * point_score - partial_penalty, 0.0, 0.95);
  candidate.partially_observed = partial;
  return candidate;
}

}  // namespace

PillarScanDetector::PillarScanDetector(const PillarScanDetectorConfig & config)
: config_(config)
{
  if (!std::isfinite(config_.expected_pillar_width_m) ||
    config_.expected_pillar_width_m <= 0.0)
  {
    throw std::invalid_argument("expected_pillar_width_m must be finite and positive");
  }
  if (!std::isfinite(config_.min_observed_width_m) ||
    !std::isfinite(config_.max_observed_width_m) ||
    config_.min_observed_width_m <= 0.0 ||
    config_.max_observed_width_m < config_.min_observed_width_m)
  {
    throw std::invalid_argument("observed width limits must be finite and ordered");
  }
  if (!std::isfinite(config_.cluster_gap_m) || config_.cluster_gap_m <= 0.0 ||
    !std::isfinite(config_.angular_gap_scale) || config_.angular_gap_scale <= 0.0 ||
    config_.min_cluster_points < 2U)
  {
    throw std::invalid_argument("cluster gap and minimum point settings must be positive");
  }
}

PillarScanDetection PillarScanDetector::detect(
  const sensor_msgs::msg::LaserScan & scan) const
{
  PillarScanDetection result;
  if (scan.ranges.empty() || !std::isfinite(scan.angle_min) ||
    !std::isfinite(scan.angle_increment) || std::abs(scan.angle_increment) < 1e-9 ||
    !std::isfinite(scan.range_min) || !std::isfinite(scan.range_max) ||
    scan.range_min < 0.0F || scan.range_max <= scan.range_min)
  {
    result.status = "invalid_scan";
    result.reason = "invalid_scan_metadata";
    return result;
  }

  std::vector<Cluster> clusters;
  Cluster current;
  std::size_t valid_point_count = 0;
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const float range = scan.ranges[i];
    if (!valid_range(range, scan)) {
      if (!current.empty()) {
        clusters.push_back(std::move(current));
        current.clear();
      }
      continue;
    }

    ++valid_point_count;
    const double angle = static_cast<double>(scan.angle_min) +
      static_cast<double>(i) * static_cast<double>(scan.angle_increment);
    IndexedPoint point{
      i,
      static_cast<double>(range),
      {static_cast<double>(range) * std::cos(angle),
        static_cast<double>(range) * std::sin(angle)}};

    if (!current.empty()) {
      const auto & previous = current.back();
      const double gap = point_distance(previous.point, point.point);
      if (gap > cluster_gap_limit(
          previous.range, point.range, scan.angle_increment, config_))
      {
        clusters.push_back(std::move(current));
        current.clear();
      }
    }
    current.push_back(point);
  }
  if (!current.empty()) clusters.push_back(std::move(current));

  if (valid_point_count == 0U) {
    result.status = "invalid_scan";
    result.reason = "no_valid_ranges";
    return result;
  }

  const double covered_angle = std::abs(static_cast<double>(scan.angle_increment)) *
    static_cast<double>(scan.ranges.size() - 1U);
  const double angle_step = std::abs(static_cast<double>(scan.angle_increment));
  const bool wraps_full_circle = covered_angle >= 2.0 * kPi - 2.5 * angle_step;
  if (wraps_full_circle && clusters.size() > 1U &&
    clusters.front().front().index == 0U &&
    clusters.back().back().index + 1U == scan.ranges.size())
  {
    const auto & first = clusters.front().front();
    const auto & last = clusters.back().back();
    if (point_distance(first.point, last.point) <= cluster_gap_limit(
        first.range, last.range, scan.angle_increment, config_))
    {
      Cluster merged = std::move(clusters.back());
      merged.insert(merged.end(), clusters.front().begin(), clusters.front().end());
      clusters.front() = std::move(merged);
      clusters.pop_back();
    }
  }

  for (const auto & cluster : clusters) {
    if (cluster.size() < config_.min_cluster_points) continue;
    const double width = cluster_diameter(cluster);
    if (width < config_.min_observed_width_m || width > config_.max_observed_width_m) {
      continue;
    }
    result.candidates.push_back(make_candidate(cluster, width, config_));
  }

  result.status = result.candidates.empty() ? "no_candidates" : "ok";
  if (result.candidates.empty()) result.reason = "no_cluster_matched_pillar_geometry";
  return result;
}

}  // namespace parking_lot_perception
