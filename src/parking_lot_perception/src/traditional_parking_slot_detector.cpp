#include "parking_lot_perception/traditional_parking_slot_detector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace parking_lot_perception
{
namespace
{

std::array<cv::Point2f, 4> ordered_corners(const cv::RotatedRect & rectangle)
{
  cv::Point2f raw[4];
  rectangle.points(raw);
  std::array<cv::Point2f, 4> result{raw[0], raw[1], raw[2], raw[3]};
  const cv::Point2f center = 0.25F * (result[0] + result[1] + result[2] + result[3]);
  std::sort(result.begin(), result.end(), [&center](const auto & first, const auto & second) {
    return std::atan2(first.y - center.y, first.x - center.x) <
           std::atan2(second.y - center.y, second.x - center.x);
  });
  const auto first = std::min_element(result.begin(), result.end(),
    [](const auto & a, const auto & b) {
      if (std::abs(a.y - b.y) > 1e-4F) return a.y < b.y;
      return a.x < b.x;
    });
  std::rotate(result.begin(), first, result.end());
  return result;
}

std::array<double, 4> edge_line_support(
  const std::array<cv::Point2f, 4> & corners,
  const cv::Mat & white_mask)
{
  constexpr std::array<int, 7> offsets{-6, -4, -2, 0, 2, 4, 6};
  std::array<double, 4> support{};
  for (std::size_t edge = 0; edge < corners.size(); ++edge) {
    const cv::Point2f start = corners[edge];
    const cv::Point2f vector = corners[(edge + 1) % corners.size()] - start;
    const double length = cv::norm(vector);
    const int sample_count = std::max(12, cvRound(length));
    const cv::Point2f normal(
      static_cast<float>(-vector.y / std::max(length, 1e-6)),
      static_cast<float>(vector.x / std::max(length, 1e-6)));
    int marked = 0;
    int valid_samples = 0;
    for (int sample = 0; sample < sample_count; ++sample) {
      const float ratio = static_cast<float>(sample) / static_cast<float>(sample_count - 1);
      const cv::Point2f point = start + vector * ratio;
      bool found_marking = false;
      bool found_valid = false;
      for (const int offset : offsets) {
        const cv::Point2f shifted = point + normal * static_cast<float>(offset);
        const int x = cvRound(shifted.x);
        const int y = cvRound(shifted.y);
        if (x < 0 || x >= white_mask.cols || y < 0 || y >= white_mask.rows) continue;
        found_valid = true;
        found_marking = found_marking || white_mask.at<unsigned char>(y, x) != 0;
      }
      if (found_valid) {
        ++valid_samples;
        if (found_marking) ++marked;
      }
    }
    support[edge] = valid_samples > 0 ?
      static_cast<double>(marked) / static_cast<double>(valid_samples) : 0.0;
  }
  return support;
}

int supported_edge_count(
  const std::array<double, 4> & support,
  double threshold)
{
  return static_cast<int>(std::count_if(support.begin(), support.end(),
    [threshold](double value) {return value >= threshold;}));
}

cv::Point2f center_of(const std::array<cv::Point2f, 4> & corners)
{
  return 0.25F * (corners[0] + corners[1] + corners[2] + corners[3]);
}

cv::Point2f pixel_to_base(
  const cv::Point2f & point,
  int width,
  int height,
  double resolution,
  double x_min,
  double y_min)
{
  const double raw_column = static_cast<double>(width - 1) - point.x;
  const double raw_row = static_cast<double>(height - 1) - point.y;
  return cv::Point2f(
    static_cast<float>(x_min + raw_row * resolution),
    static_cast<float>(y_min + raw_column * resolution));
}

std::string number(double value)
{
  std::ostringstream output;
  output << std::fixed << std::setprecision(6) << value;
  return output.str();
}

std::string json_pixel_corners(const std::array<cv::Point2f, 4> & corners)
{
  std::ostringstream output;
  output << "[";
  for (std::size_t index = 0; index < corners.size(); ++index) {
    if (index > 0) output << ",";
    output << "[" << number(corners[index].x) << "," << number(corners[index].y) << "]";
  }
  output << "]";
  return output.str();
}

cv::Point2f base_to_pixel(
  const cv::Point2f & point,
  int width,
  int height,
  double resolution,
  double x_min,
  double y_min)
{
  return cv::Point2f(
    static_cast<float>(width - 1 - (point.y - y_min) / resolution),
    static_cast<float>(height - 1 - (point.x - x_min) / resolution));
}

struct EstimatedSlotGeometry
{
  std::array<cv::Point2f, 4> corners_px{};
  std::array<cv::Point2f, 4> corners_base{};
  cv::Point2f center_px{};
  cv::Point2f center_base{};
  double yaw{0.0};
  double length_m{0.0};
  double width_m{0.0};
  bool extrapolated{false};
};

EstimatedSlotGeometry estimate_slot_geometry(
  const TraditionalParkingSlotCandidate & candidate,
  int image_width,
  int image_height,
  const TraditionalParkingSlotDetectorConfig & config)
{
  EstimatedSlotGeometry result;
  std::array<cv::Point2f, 4> observed_base_corners{};
  for (std::size_t index = 0; index < candidate.corners_px.size(); ++index) {
    observed_base_corners[index] = pixel_to_base(
      candidate.corners_px[index], image_width, image_height,
      config.resolution_m_per_pixel, config.x_min, config.y_min);
  }

  double longest_edge = -1.0;
  cv::Point2f long_axis{};
  cv::Point2f long_axis_px{};
  for (std::size_t index = 0; index < observed_base_corners.size(); ++index) {
    const cv::Point2f edge =
      observed_base_corners[(index + 1) % observed_base_corners.size()] -
      observed_base_corners[index];
    const double length = cv::norm(edge);
    if (length > longest_edge) {
      longest_edge = length;
      long_axis = edge;
      long_axis_px = candidate.corners_px[(index + 1) % candidate.corners_px.size()] -
        candidate.corners_px[index];
    }
  }
  if (longest_edge > 1e-6) {
    long_axis *= static_cast<float>(1.0 / longest_edge);
    long_axis_px *= static_cast<float>(1.0 / cv::norm(long_axis_px));
  }
  const cv::Point2f short_axis(-long_axis.y, long_axis.x);

  result.center_base = center_of(observed_base_corners);
  result.length_m = candidate.long_side_px * config.resolution_m_per_pixel;
  result.width_m = candidate.short_side_px * config.resolution_m_per_pixel;
  result.corners_base = observed_base_corners;

  if (candidate.touches_image_border) {
    const int border_count = static_cast<int>(candidate.touches_left_border) +
      static_cast<int>(candidate.touches_right_border) +
      static_cast<int>(candidate.touches_top_border) +
      static_cast<int>(candidate.touches_bottom_border);
    cv::Point2f border_outward{};
    if (border_count == 1) {
      if (candidate.touches_left_border) border_outward = {-1.0F, 0.0F};
      if (candidate.touches_right_border) border_outward = {1.0F, 0.0F};
      if (candidate.touches_top_border) border_outward = {0.0F, -1.0F};
      if (candidate.touches_bottom_border) border_outward = {0.0F, 1.0F};

      const cv::Point2f short_axis_px(-long_axis_px.y, long_axis_px.x);
      const double long_alignment = border_outward.dot(long_axis_px);
      const double short_alignment = border_outward.dot(short_axis_px);
      const bool clipped_along_length = std::abs(long_alignment) >=
        std::abs(short_alignment);
      const cv::Point2f clipped_axis_px = clipped_along_length ?
        long_axis_px * static_cast<float>(long_alignment >= 0.0 ? 1.0 : -1.0) :
        short_axis_px * static_cast<float>(short_alignment >= 0.0 ? 1.0 : -1.0);
      const double observed_dimension_m = (clipped_along_length ?
        candidate.long_side_px : candidate.short_side_px) * config.resolution_m_per_pixel;
      const double missing_dimension_m = clipped_along_length ?
        config.expected_length_m : config.expected_width_m;

      std::array<std::pair<double, std::size_t>, 4> projections{};
      for (std::size_t index = 0; index < candidate.corners_px.size(); ++index) {
        projections[index] = {
          candidate.corners_px[index].dot(clipped_axis_px), index};
      }
      std::sort(projections.begin(), projections.end(),
        [](const auto & first, const auto & second) {return first.first < second.first;});
      const cv::Point2f visible_midpoint_px = 0.5F * (
        candidate.corners_px[projections[0].second] +
        candidate.corners_px[projections[1].second]);

      if (std::abs(long_alignment) + std::abs(short_alignment) > 0.0 &&
        missing_dimension_m > observed_dimension_m +
        2.0 * config.resolution_m_per_pixel)
      {
        const cv::Point2f completed_center_px = visible_midpoint_px + clipped_axis_px *
          static_cast<float>(missing_dimension_m / (2.0 * config.resolution_m_per_pixel));
        result.center_base = pixel_to_base(
          completed_center_px, image_width, image_height,
          config.resolution_m_per_pixel, config.x_min, config.y_min);
        result.length_m = config.expected_length_m;
        result.width_m = config.expected_width_m;
        result.extrapolated = true;

        const cv::Point2f half_length = long_axis *
          static_cast<float>(result.length_m * 0.5);
        const cv::Point2f half_width = short_axis *
          static_cast<float>(result.width_m * 0.5);
        result.corners_base = {
          result.center_base - half_length - half_width,
          result.center_base + half_length - half_width,
          result.center_base + half_length + half_width,
          result.center_base - half_length + half_width};
      }
    }
  }

  result.yaw = std::atan2(long_axis.y, long_axis.x);
  for (std::size_t index = 0; index < result.corners_base.size(); ++index) {
    result.corners_px[index] = base_to_pixel(
      result.corners_base[index], image_width, image_height,
      config.resolution_m_per_pixel, config.x_min, config.y_min);
  }
  result.center_px = base_to_pixel(
    result.center_base, image_width, image_height,
    config.resolution_m_per_pixel, config.x_min, config.y_min);
  return result;
}

}  // namespace

TraditionalParkingSlotDetector::TraditionalParkingSlotDetector(
  TraditionalParkingSlotDetectorConfig config)
: config_(std::move(config))
{
  if (!std::isfinite(config_.resolution_m_per_pixel) ||
    config_.resolution_m_per_pixel <= 0.0 ||
    !std::isfinite(config_.x_min) || !std::isfinite(config_.y_min) ||
    !std::isfinite(config_.expected_width_m) || config_.expected_width_m <= 0.0 ||
    !std::isfinite(config_.expected_length_m) || config_.expected_length_m <= 0.0 ||
    config_.min_component_area_px <= 0 || config_.min_rectangularity < 0.0 ||
    config_.min_rectangularity > 1.0 || config_.min_aspect_ratio <= 0.0 ||
    config_.max_aspect_ratio < config_.min_aspect_ratio ||
    config_.short_side_tolerance_fraction < 0.0 ||
    config_.short_side_tolerance_fraction >= 1.0 || config_.min_long_side_fraction <= 0.0 ||
    config_.max_long_side_fraction < config_.min_long_side_fraction ||
    config_.min_line_support < 0.0 ||
    config_.min_line_support > 1.0 || config_.min_supported_edges < 0 ||
    config_.min_supported_edges > 4)
  {
    throw std::invalid_argument("invalid traditional parking-slot detector limits");
  }
}

TraditionalParkingSlotDetection TraditionalParkingSlotDetector::detect(
  const cv::Mat & image) const
{
  if (image.empty() || image.type() != CV_8UC3) {
    throw std::invalid_argument("BEV image must be a non-empty BGR8 image");
  }

  TraditionalParkingSlotDetection result;
  result.slot_mask = cv::Mat::zeros(image.size(), CV_8UC1);
  result.white_line_mask = cv::Mat::zeros(image.size(), CV_8UC1);
  for (int row = 0; row < image.rows; ++row) {
    const auto * input = image.ptr<cv::Vec3b>(row);
    auto * slot = result.slot_mask.ptr<unsigned char>(row);
    auto * white = result.white_line_mask.ptr<unsigned char>(row);
    for (int column = 0; column < image.cols; ++column) {
      const int blue = input[column][0];
      const int green = input[column][1];
      const int red = input[column][2];
      if (blue - red > config_.min_blue_red_difference &&
        blue - green > config_.min_blue_green_difference && blue > config_.min_blue_value)
      {
        slot[column] = 255;
      }
      const int maximum = std::max({blue, green, red});
      const int minimum = std::min({blue, green, red});
      if (maximum >= config_.min_white_value && maximum - minimum <= config_.max_white_chroma) {
        white[column] = 255;
      }
    }
  }

  cv::Mat labels;
  cv::Mat stats;
  cv::Mat centroids;
  const int component_count = cv::connectedComponentsWithStats(
    result.slot_mask, labels, stats, centroids, 8, CV_32S);
  for (int label = 1; label < component_count; ++label) {
    const int x = stats.at<int>(label, cv::CC_STAT_LEFT);
    const int y = stats.at<int>(label, cv::CC_STAT_TOP);
    const int component_width = stats.at<int>(label, cv::CC_STAT_WIDTH);
    const int component_height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
    const int area = stats.at<int>(label, cv::CC_STAT_AREA);
    if (area < config_.min_component_area_px) continue;

    cv::Mat component;
    cv::compare(
      labels(cv::Rect(x, y, component_width, component_height)), label, component, cv::CMP_EQ);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(component, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) continue;
    auto contour = *std::max_element(contours.begin(), contours.end(),
      [](const auto & first, const auto & second) {
        return cv::contourArea(first) < cv::contourArea(second);
      });
    for (auto & point : contour) {
      point.x += x;
      point.y += y;
    }

    const cv::RotatedRect rectangle = cv::minAreaRect(contour);
    const double rect_width = rectangle.size.width;
    const double rect_height = rectangle.size.height;
    const double rectangle_area = rect_width * rect_height;
    if (rectangle_area <= 0.0) continue;

    TraditionalParkingSlotCandidate candidate;
    candidate.corners_px = ordered_corners(rectangle);
    candidate.short_side_px = std::min(rect_width, rect_height);
    candidate.long_side_px = std::max(rect_width, rect_height);
    candidate.aspect_ratio = candidate.long_side_px / std::max(candidate.short_side_px, 1e-6);
    candidate.rectangularity = static_cast<double>(area) / rectangle_area;
    candidate.line_support = edge_line_support(candidate.corners_px, result.white_line_mask);
    candidate.supported_edges = supported_edge_count(
      candidate.line_support, config_.min_line_support);
    candidate.touches_left_border = x <= 0;
    candidate.touches_top_border = y <= 0;
    candidate.touches_right_border = x + component_width >= image.cols;
    candidate.touches_bottom_border = y + component_height >= image.rows;
    candidate.touches_image_border = candidate.touches_left_border ||
      candidate.touches_right_border || candidate.touches_top_border ||
      candidate.touches_bottom_border;
    candidate.partial_or_occluded = candidate.touches_image_border || candidate.supported_edges < 3;

    if (candidate.aspect_ratio < config_.min_aspect_ratio ||
      candidate.aspect_ratio > config_.max_aspect_ratio)
    {
      candidate.rejection_reasons.emplace_back("aspect_ratio_out_of_range");
    }
    const double expected_short_side_px =
      config_.expected_width_m / config_.resolution_m_per_pixel;
    const double min_short_side_px = expected_short_side_px *
      (1.0 - config_.short_side_tolerance_fraction);
    const double max_short_side_px = expected_short_side_px *
      (1.0 + config_.short_side_tolerance_fraction);
    if (candidate.short_side_px < min_short_side_px ||
      candidate.short_side_px > max_short_side_px)
    {
      candidate.rejection_reasons.emplace_back("slot_width_out_of_range");
    }
    const double long_side_m = candidate.long_side_px * config_.resolution_m_per_pixel;
    if (long_side_m < config_.expected_length_m * config_.min_long_side_fraction ||
      long_side_m > config_.expected_length_m * config_.max_long_side_fraction)
    {
      candidate.rejection_reasons.emplace_back("slot_length_out_of_range");
    }
    if (candidate.rectangularity < config_.min_rectangularity) {
      candidate.rejection_reasons.emplace_back("component_not_rectangular_enough");
    }
    if (candidate.supported_edges < config_.min_supported_edges) {
      candidate.rejection_reasons.emplace_back("insufficient_white_line_support");
    }
    candidate.accepted = candidate.rejection_reasons.empty();
    const double width_score = std::clamp(
      1.0 - std::abs(candidate.short_side_px - expected_short_side_px) /
      std::max(expected_short_side_px * config_.short_side_tolerance_fraction, 1.0), 0.0, 1.0);
    candidate.confidence = std::clamp(
      0.45 * std::clamp(candidate.rectangularity, 0.0, 1.0) +
      0.35 * static_cast<double>(candidate.supported_edges) / 4.0 + 0.20 * width_score,
      0.0, 1.0);
    (candidate.accepted ? result.accepted : result.rejected).push_back(std::move(candidate));
  }

  const auto sort_by_center = [](const auto & first, const auto & second) {
      const auto a = center_of(first.corners_px);
      const auto b = center_of(second.corners_px);
      if (std::abs(a.y - b.y) > 1e-3F) return a.y < b.y;
      return a.x < b.x;
    };
  std::sort(result.accepted.begin(), result.accepted.end(), sort_by_center);
  std::sort(result.rejected.begin(), result.rejected.end(), sort_by_center);
  int candidate_id = 1;
  for (auto & candidate : result.accepted) {
    candidate.id = "slot-" + std::to_string(candidate_id++);
  }
  for (auto & candidate : result.rejected) {
    candidate.id = "reject-" + std::to_string(candidate_id++);
  }
  return result;
}

cv::Mat TraditionalParkingSlotDetector::draw_debug(
  const cv::Mat & image,
  const TraditionalParkingSlotDetection & detection) const
{
  cv::Mat debug = image.clone();
  const auto draw = [this, &debug](
      const TraditionalParkingSlotCandidate & candidate, bool accepted) {
      const auto geometry = estimate_slot_geometry(
        candidate, debug.cols, debug.rows, config_);
      const auto & corners = accepted ? geometry.corners_px : candidate.corners_px;
      std::vector<cv::Point> polygon;
      for (const auto & corner : corners) {
        polygon.emplace_back(cvRound(corner.x), cvRound(corner.y));
      }
      const cv::Scalar color = accepted ? cv::Scalar(0, 220, 0) : cv::Scalar(0, 140, 255);
      cv::polylines(debug, polygon, true, color, 2, cv::LINE_AA);
      for (const auto & corner : polygon) {
        cv::circle(debug, corner, 4, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
      }
      const auto visible_center = center_of(candidate.corners_px);
      if (accepted) {
        cv::circle(debug, cv::Point(cvRound(geometry.center_px.x), cvRound(geometry.center_px.y)), 4,
          cv::Scalar(255, 0, 255), cv::FILLED, cv::LINE_AA);
      } else {
        cv::circle(debug, cv::Point(cvRound(visible_center.x), cvRound(visible_center.y)), 4,
          cv::Scalar(255, 0, 255), cv::FILLED, cv::LINE_AA);
      }
      const auto label_anchor = accepted ? geometry.center_px : visible_center;
      cv::putText(debug, candidate.id,
        cv::Point(cvRound(label_anchor.x) + 6, cvRound(label_anchor.y)),
        cv::FONT_HERSHEY_SIMPLEX, 0.48, color, 2, cv::LINE_AA);
    };
  for (const auto & candidate : detection.accepted) draw(candidate, true);
  for (const auto & candidate : detection.rejected) draw(candidate, false);
  return debug;
}

std::string TraditionalParkingSlotDetector::candidates_to_json(
  const std::vector<TraditionalParkingSlotCandidate> & candidates,
  int image_width,
  int image_height,
  double stamp_sec,
  const std::string & frame_id) const
{
  std::ostringstream output;
  output << "{\"schema_version\":\"parking-slot-observations-v2\","
         << "\"candidate_only\":true,\"frame_id\":\"" << frame_id << "\","
         << "\"image_width\":" << image_width << ",\"image_height\":" << image_height
         << ",\"stamp\":" << number(stamp_sec) << ",\"candidates\":[";
  bool first = true;
  for (const auto & candidate : candidates) {
    if (!first) output << ",";
    first = false;
    const auto geometry = estimate_slot_geometry(candidate, image_width, image_height, config_);
    output << "{\"id\":\"" << candidate.id << "\",\"pixel_corners\":"
           << json_pixel_corners(candidate.corners_px)
           << ",\"completed_pixel_corners\":"
           << json_pixel_corners(geometry.corners_px)
           << ",\"partial_or_occluded\":"
           << (candidate.partial_or_occluded ? "true" : "false")
           << ",\"pose_extrapolated\":"
           << (geometry.extrapolated ? "true" : "false")
           << ",\"confidence\":" << number(candidate.confidence);
    output << ",\"x\":" << number(geometry.center_base.x)
           << ",\"y\":" << number(geometry.center_base.y)
           << ",\"yaw\":" << number(geometry.yaw)
           << ",\"length\":" << number(geometry.length_m)
           << ",\"width\":" << number(geometry.width_m)
           << ",\"corners\":[";
    for (std::size_t corner = 0; corner < geometry.corners_base.size(); ++corner) {
      if (corner > 0) output << ",";
      output << "[" << number(geometry.corners_base[corner].x) << ","
             << number(geometry.corners_base[corner].y) << "]";
    }
    output << "]";
    output << "}";
  }
  output << "]}";
  return output.str();
}

}  // namespace parking_lot_perception
