#include "parking_lot_core/time_sync.hpp"

#include <cmath>
#include <limits>

namespace parking_lot_core
{

std::string TimestampMatch::reason() const
{
  switch (status) {
    case TimestampMatchStatus::matched: return "matched";
    case TimestampMatchStatus::empty_samples: return "empty_samples";
    case TimestampMatchStatus::invalid_target: return "invalid_target";
    case TimestampMatchStatus::no_valid_samples: return "no_valid_samples";
    case TimestampMatchStatus::stale: return "stale";
    case TimestampMatchStatus::invalid_max_age: return "invalid_max_age";
  }
  return "unknown";
}

TimestampMatch match_timestamp(
  double target_time_sec,
  const std::vector<double> & sample_times_sec,
  double max_age_sec)
{
  if (!std::isfinite(max_age_sec) || max_age_sec < 0.0) {
    return TimestampMatch{TimestampMatchStatus::invalid_max_age, 0, 0.0, 0.0};
  }
  if (!std::isfinite(target_time_sec)) {
    return TimestampMatch{TimestampMatchStatus::invalid_target, 0, 0.0, 0.0};
  }
  if (sample_times_sec.empty()) {
    return TimestampMatch{TimestampMatchStatus::empty_samples, 0, 0.0, 0.0};
  }

  std::size_t best_index = 0;
  double best_age = std::numeric_limits<double>::infinity();
  double best_time = 0.0;
  bool found = false;
  for (std::size_t index = 0; index < sample_times_sec.size(); ++index) {
    const double sample_time = sample_times_sec[index];
    if (!std::isfinite(sample_time)) continue;
    const double age = std::abs(sample_time - target_time_sec);
    if (!found || age < best_age) {
      found = true;
      best_index = index;
      best_age = age;
      best_time = sample_time;
    }
  }
  if (!found) return TimestampMatch{TimestampMatchStatus::no_valid_samples, 0, 0.0, 0.0};
  if (best_age > max_age_sec) {
    return TimestampMatch{TimestampMatchStatus::stale, best_index, best_time, best_age};
  }
  return TimestampMatch{TimestampMatchStatus::matched, best_index, best_time, best_age};
}

}  // namespace parking_lot_core
