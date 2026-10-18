#pragma once

#include <opencv2/core.hpp>

#include <array>
#include <string>
#include <vector>

namespace parking_lot_perception
{

struct TraditionalParkingSlotDetectorConfig
{
  double resolution_m_per_pixel{0.02};
  double x_min{-8.0};
  double y_min{-8.0};
  double expected_width_m{2.8};
  double expected_length_m{5.8};
  int min_component_area_px{5000};
  double min_rectangularity{0.55};
  double min_aspect_ratio{1.15};
  double max_aspect_ratio{3.2};
  double short_side_tolerance_fraction{0.30};
  double min_long_side_fraction{0.52};
  double max_long_side_fraction{1.31};
  int min_blue_red_difference{8};
  int min_blue_green_difference{5};
  int min_blue_value{25};
  int min_white_value{180};
  int max_white_chroma{40};
  double min_line_support{0.35};
  int min_supported_edges{2};
};

struct TraditionalParkingSlotCandidate
{
  std::string id;
  std::array<cv::Point2f, 4> corners_px{};
  std::array<double, 4> line_support{};
  double short_side_px{0.0};
  double long_side_px{0.0};
  double rectangularity{0.0};
  double aspect_ratio{0.0};
  double confidence{0.0};
  int supported_edges{0};
  bool touches_image_border{false};
  bool touches_left_border{false};
  bool touches_right_border{false};
  bool touches_top_border{false};
  bool touches_bottom_border{false};
  bool partial_or_occluded{false};
  bool accepted{false};
  std::vector<std::string> rejection_reasons;
};

struct TraditionalParkingSlotDetection
{
  std::vector<TraditionalParkingSlotCandidate> accepted;
  std::vector<TraditionalParkingSlotCandidate> rejected;
  cv::Mat slot_mask;
  cv::Mat white_line_mask;
};

class TraditionalParkingSlotDetector
{
public:
  explicit TraditionalParkingSlotDetector(
    TraditionalParkingSlotDetectorConfig config = {});

  TraditionalParkingSlotDetection detect(const cv::Mat & image) const;

  cv::Mat draw_debug(
    const cv::Mat & image,
    const TraditionalParkingSlotDetection & detection) const;

  std::string candidates_to_json(
    const std::vector<TraditionalParkingSlotCandidate> & candidates,
    int image_width,
    int image_height,
    double stamp_sec,
    const std::string & frame_id = "base_footprint") const;

private:
  TraditionalParkingSlotDetectorConfig config_;
};

}  // namespace parking_lot_perception
