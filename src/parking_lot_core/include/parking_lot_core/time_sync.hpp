#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace parking_lot_core
{

enum class TimestampMatchStatus
{
  matched,
  empty_samples,
  invalid_target,
  no_valid_samples,
  stale,
  invalid_max_age,
};

struct TimestampMatch
{
  TimestampMatchStatus status{TimestampMatchStatus::empty_samples};
  std::size_t index{0};
  double sample_time{0.0};
  double age_sec{0.0};

  bool valid() const noexcept
  {
    return status == TimestampMatchStatus::matched;
  }

  std::string reason() const;
};

TimestampMatch match_timestamp(
  double target_time_sec,
  const std::vector<double> & sample_times_sec,
  double max_age_sec);

}  // namespace parking_lot_core
