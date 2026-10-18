#include "parking_lot_perception/pillar_scan_detector.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace parking_lot_perception
{

class PillarScanDetectorNode final : public rclcpp::Node
{
public:
  PillarScanDetectorNode()
  : Node("pillar_scan_detector"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/scan");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/semantic/pillar_candidates");
    marker_topic_ = declare_parameter<std::string>(
      "marker_topic", "/semantic/pillar_candidate_markers");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    PillarScanDetectorConfig config;
    config.expected_pillar_width_m = declare_parameter<double>(
      "expected_pillar_width_m", config.expected_pillar_width_m);
    config.min_observed_width_m = declare_parameter<double>(
      "min_observed_width_m", config.min_observed_width_m);
    config.max_observed_width_m = declare_parameter<double>(
      "max_observed_width_m", config.max_observed_width_m);
    config.cluster_gap_m = declare_parameter<double>(
      "cluster_gap_m", config.cluster_gap_m);
    config.angular_gap_scale = declare_parameter<double>(
      "angular_gap_scale", config.angular_gap_scale);
    const int min_cluster_points = declare_parameter<int>(
      "min_cluster_points", static_cast<int>(config.min_cluster_points));
    if (min_cluster_points < 2) {
      throw std::invalid_argument("min_cluster_points must be at least 2");
    }
    config.min_cluster_points = static_cast<std::size_t>(min_cluster_points);
    detector_ = std::make_unique<PillarScanDetector>(config);
    if (base_frame_.empty()) throw std::invalid_argument("base_frame must not be empty");

    candidate_publisher_ = create_publisher<std_msgs::msg::String>(output_topic_, 10);
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic_, rclcpp::SensorDataQoS().keep_last(2),
      std::bind(&PillarScanDetectorNode::on_scan, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "pillar scan detector listening on %s; publishing candidates on %s",
      input_topic_.c_str(), output_topic_.c_str());
  }

private:
  static double stamp_to_seconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
  }

  std::string serialize(
    const sensor_msgs::msg::LaserScan & scan, const std::string & status,
    const std::string & reason,
    const std::vector<PillarScanCandidate> & candidates) const
  {
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    writer.StartObject();
    writer.Key("schema_version"); writer.String("pillar-candidates-v1");
    writer.Key("candidate_only"); writer.Bool(true);
    writer.Key("frame_id"); writer.String(base_frame_.c_str());
    writer.Key("source_frame"); writer.String(scan.header.frame_id.c_str());
    writer.Key("stamp"); writer.Double(stamp_to_seconds(scan.header.stamp));
    writer.Key("status"); writer.String(status.c_str());
    writer.Key("reason"); writer.String(reason.c_str());
    writer.Key("candidates"); writer.StartArray();
    for (std::size_t i = 0; i < candidates.size(); ++i) {
      const auto & candidate = candidates[i];
      writer.StartObject();
      const std::string id = "pillar-" + std::to_string(i + 1U);
      writer.Key("id"); writer.String(id.c_str());
      writer.Key("x"); writer.Double(candidate.x);
      writer.Key("y"); writer.Double(candidate.y);
      writer.Key("observed_width_m"); writer.Double(candidate.observed_width_m);
      writer.Key("confidence"); writer.Double(candidate.confidence);
      writer.Key("partial_or_occluded"); writer.Bool(candidate.partially_observed);
      writer.EndObject();
    }
    writer.EndArray();
    writer.EndObject();
    return buffer.GetString();
  }

  void publish_status(
    const sensor_msgs::msg::LaserScan & scan, const std::string & status,
    const std::string & reason)
  {
    std_msgs::msg::String message;
    message.data = serialize(scan, status, reason, {});
    candidate_publisher_->publish(message);
    publish_markers(scan, {});
  }

  void publish_markers(
    const sensor_msgs::msg::LaserScan & scan,
    const std::vector<PillarScanCandidate> & candidates)
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.header.stamp = scan.header.stamp;
    clear.header.frame_id = base_frame_;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    for (std::size_t i = 0; i < candidates.size(); ++i) {
      const auto & candidate = candidates[i];
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = scan.header.stamp;
      marker.header.frame_id = base_frame_;
      marker.ns = "pillar_candidate";
      marker.id = static_cast<int32_t>(i);
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = candidate.x;
      marker.pose.position.y = candidate.y;
      marker.pose.position.z = 0.35;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.65;
      marker.scale.y = 0.65;
      marker.scale.z = 0.7;
      marker.color.r = 1.0F;
      marker.color.g = 0.55F;
      marker.color.b = 0.0F;
      marker.color.a = 0.85F;
      marker.lifetime = rclcpp::Duration::from_seconds(0.35);
      marker.text = "pillar? " + std::to_string(i + 1U);
      array.markers.push_back(std::move(marker));
    }
    marker_publisher_->publish(array);
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    if (scan->header.frame_id.empty()) {
      publish_status(*scan, "invalid_scan", "scan_frame_id_empty");
      return;
    }

    const auto detection = detector_->detect(*scan);
    if (detection.status == "invalid_scan") {
      publish_status(*scan, detection.status, detection.reason);
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        base_frame_, scan->header.frame_id, rclcpp::Time(scan->header.stamp),
        rclcpp::Duration::from_seconds(0.05));
    } catch (const tf2::TransformException & error) {
      publish_status(*scan, "tf_unavailable", error.what());
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "cannot transform scan into %s: %s",
        base_frame_.c_str(), error.what());
      return;
    }

    auto candidates = detection.candidates;
    for (auto & candidate : candidates) {
      geometry_msgs::msg::PointStamped local;
      local.header = scan->header;
      local.point.x = candidate.x;
      local.point.y = candidate.y;
      local.point.z = 0.0;
      geometry_msgs::msg::PointStamped vehicle_relative;
      tf2::doTransform(local, vehicle_relative, transform);
      candidate.x = vehicle_relative.point.x;
      candidate.y = vehicle_relative.point.y;
    }

    std_msgs::msg::String message;
    message.data = serialize(*scan, detection.status, detection.reason, candidates);
    candidate_publisher_->publish(message);
    publish_markers(*scan, candidates);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string marker_topic_;
  std::string base_frame_;
  std::unique_ptr<PillarScanDetector> detector_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr candidate_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
};

}  // namespace parking_lot_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<parking_lot_perception::PillarScanDetectorNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("pillar_scan_detector"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
