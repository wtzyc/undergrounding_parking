#ifndef PARKING_LOT_PERCEPTION__PILLAR_SCAN_DETECTOR_HPP_
#define PARKING_LOT_PERCEPTION__PILLAR_SCAN_DETECTOR_HPP_

#include <sensor_msgs/msg/laser_scan.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace parking_lot_perception
{

struct PillarScanCandidate
{
  // The estimated pillar center, expressed in the LaserScan frame.
  double x{0.0};
  double y{0.0};
  double observed_width_m{0.0};
  double confidence{0.0};
  bool partially_observed{false};
};

struct PillarScanDetection
{
  std::string status{"invalid_scan"};
  std::string reason;
  std::vector<PillarScanCandidate> candidates;
};

struct PillarScanDetectorConfig
{
  double expected_pillar_width_m{1.2};
  double min_observed_width_m{0.3};
  double max_observed_width_m{2.1};
  double cluster_gap_m{0.15};
  double angular_gap_scale{1.2};
  std::size_t min_cluster_points{3};
};

// Geometry-only candidate detector for a 2D LaserScan. It deliberately reports
// candidates rather than claiming semantic certainty: a short wall fragment can
// have the same 2D shape as a pillar and must be confirmed by later map fusion.
class PillarScanDetector
{
public:
  explicit PillarScanDetector(const PillarScanDetectorConfig & config = {});

  PillarScanDetection detect(const sensor_msgs::msg::LaserScan & scan) const;

private:
  PillarScanDetectorConfig config_;
};

}  // namespace parking_lot_perception

#endif  // PARKING_LOT_PERCEPTION__PILLAR_SCAN_DETECTOR_HPP_
