#include "parking_lot_perception/parking_slot_map_store.hpp"
#include "parking_lot_perception/pillar_landmark_store.hpp"
#include "parking_lot_core/virtual_road_graph.hpp"
#include "parking_lot_core/virtual_road_nodes.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <rapidjson/document.h>
#include <rapidjson/error/en.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <array>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace parking_lot_perception
{
namespace
{

bool read_number(
  const rapidjson::Value & object,
  const char * key,
  double & result)
{
  if (!object.IsObject() || !object.HasMember(key) || !object[key].IsNumber()) return false;
  result = object[key].GetDouble();
  return std::isfinite(result);
}

bool parse_observations(
  const std::string & payload,
  const std::string & expected_frame,
  double & stamp_sec,
  std::vector<ParkingSlotObservation> & observations,
  std::string & error)
{
  rapidjson::Document document;
  document.Parse(payload.c_str());
  if (document.HasParseError()) {
    error = std::string("invalid candidate JSON: ") +
      rapidjson::GetParseError_En(document.GetParseError());
    return false;
  }
  if (!document.IsObject()) {
    error = "candidate message root must be an object";
    return false;
  }
  if (!document.HasMember("schema_version") || !document["schema_version"].IsString() ||
    std::string(document["schema_version"].GetString()) != "parking-slot-observations-v2")
  {
    error = "candidate message has an unsupported schema_version";
    return false;
  }
  if (!document.HasMember("frame_id") || !document["frame_id"].IsString() ||
    std::string(document["frame_id"].GetString()) != expected_frame)
  {
    error = "candidate message frame_id does not match configured base_frame";
    return false;
  }
  if (!read_number(document, "stamp", stamp_sec) || stamp_sec < 0.0 ||
    stamp_sec > static_cast<double>(std::numeric_limits<int64_t>::max()) / 1e9)
  {
    error = "candidate message must contain a finite, non-negative ROS timestamp";
    return false;
  }
  if (!document.HasMember("candidates") || !document["candidates"].IsArray()) {
    error = "candidate message must contain a candidates array";
    return false;
  }

  for (const auto & candidate : document["candidates"].GetArray()) {
    if (!candidate.IsObject()) continue;
    ParkingSlotObservation observation;
    if (!read_number(candidate, "x", observation.x) ||
      !read_number(candidate, "y", observation.y) ||
      !read_number(candidate, "yaw", observation.yaw) ||
      !read_number(candidate, "length", observation.length) ||
      !read_number(candidate, "width", observation.width) ||
      !read_number(candidate, "confidence", observation.confidence))
    {
      continue;
    }
    if (observation.length <= 0.0 || observation.width <= 0.0 ||
      observation.confidence < 0.0 || observation.confidence > 1.0)
    {
      continue;
    }
    observations.push_back(observation);
  }
  return true;
}

bool parse_pillar_observations(
  const std::string & payload,
  const std::string & expected_frame,
  double & stamp_sec,
  std::string & status,
  std::vector<PillarLandmarkObservation> & observations,
  std::string & error)
{
  rapidjson::Document document;
  document.Parse(payload.c_str());
  if (document.HasParseError() || !document.IsObject()) {
    error = "invalid pillar candidate JSON";
    return false;
  }
  if (!document.HasMember("schema_version") || !document["schema_version"].IsString() ||
    std::string(document["schema_version"].GetString()) != "pillar-candidates-v1")
  {
    error = "pillar candidate message has an unsupported schema_version";
    return false;
  }
  if (!document.HasMember("frame_id") || !document["frame_id"].IsString() ||
    std::string(document["frame_id"].GetString()) != expected_frame)
  {
    error = "pillar candidate frame_id does not match configured base_frame";
    return false;
  }
  if (!read_number(document, "stamp", stamp_sec) || stamp_sec < 0.0 ||
    stamp_sec > static_cast<double>(std::numeric_limits<int64_t>::max()) / 1e9)
  {
    error = "pillar candidate message must contain a finite, non-negative ROS timestamp";
    return false;
  }
  if (!document.HasMember("status") || !document["status"].IsString()) {
    error = "pillar candidate message must contain a status";
    return false;
  }
  status = document["status"].GetString();
  if (!document.HasMember("candidates") || !document["candidates"].IsArray()) {
    error = "pillar candidate message must contain a candidates array";
    return false;
  }
  if (status != "ok") return true;

  for (const auto & candidate : document["candidates"].GetArray()) {
    if (!candidate.IsObject()) continue;
    PillarLandmarkObservation observation;
    if (!read_number(candidate, "x", observation.x) ||
      !read_number(candidate, "y", observation.y) ||
      !read_number(candidate, "confidence", observation.confidence) ||
      observation.confidence < 0.0 || observation.confidence > 1.0)
    {
      continue;
    }
    observations.push_back(observation);
  }
  return true;
}

}  // namespace

class ParkingSlotMapFusion final : public rclcpp::Node
{
public:
  ParkingSlotMapFusion()
  : Node("parking_slot_map_fusion"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/semantic/parking_slot_candidates");
    pillar_input_topic_ = declare_parameter<std::string>(
      "pillar_input_topic", "/semantic/pillar_candidates");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/semantic/parking_slots_map");
    marker_topic_ = declare_parameter<std::string>(
      "marker_topic", "/semantic/parking_slot_markers");
    save_service_name_ = declare_parameter<std::string>(
      "save_service_name", "/semantic/parking_slots_map/save");
    map_path_ = declare_parameter<std::string>("map_path", "");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    tf_tolerance_sec_ = declare_parameter<double>("tf_tolerance_sec", 0.1);
    ParkingSlotMapFusionConfig config;
    config.match_distance_m = declare_parameter<double>("match_distance_m", 0.5);
    config.match_angle_deg = declare_parameter<double>("match_angle_deg", 15.0);
    config.duplicate_merge_distance_m = declare_parameter<double>(
      "duplicate_merge_distance_m", 1.2);
    config.duplicate_merge_angle_deg = declare_parameter<double>(
      "duplicate_merge_angle_deg", 10.0);
    config.duplicate_dimension_tolerance_m = declare_parameter<double>(
      "duplicate_dimension_tolerance_m", 0.6);
    config.confirm_hits = declare_parameter<int>("confirm_hits", 10);
    config.stability_position_tolerance_m = declare_parameter<double>(
      "stability_position_tolerance_m", 0.15);
    config.stability_angle_tolerance_deg = declare_parameter<double>(
      "stability_angle_tolerance_deg", 5.0);
    config.stability_dimension_tolerance_m = declare_parameter<double>(
      "stability_dimension_tolerance_m", 0.15);
    config.max_missed_frames = declare_parameter<int>("max_missed_frames", 5);
    PillarLandmarkStoreConfig pillar_config;
    pillar_config.match_distance_m = config.match_distance_m;
    pillar_config.same_frame_merge_distance_m = declare_parameter<double>(
      "pillar_same_frame_merge_distance_m", 0.6);
    pillar_config.duplicate_merge_distance_m = declare_parameter<double>(
      "pillar_duplicate_merge_distance_m", 0.8);
    pillar_config.stability_position_tolerance_m =
      config.stability_position_tolerance_m;
    pillar_config.confirm_hits = config.confirm_hits;
    pillar_config.max_missed_frames = config.max_missed_frames;
    pillar_config.expected_side_length_m = declare_parameter<double>(
      "pillar_side_length_m", 1.2);
    pillar_config.min_confidence = declare_parameter<double>(
      "pillar_min_confidence", 0.35);
    if (!std::isfinite(tf_tolerance_sec_) || tf_tolerance_sec_ < 0.0) {
      throw std::invalid_argument("tf_tolerance_sec must be finite and non-negative");
    }
    if (base_frame_.empty()) throw std::invalid_argument("base_frame must not be empty");

    slot_store_ = std::make_unique<ParkingSlotMapStore>(config, map_frame_, "my_orange_car");
    pillar_store_ = std::make_unique<PillarLandmarkStore>(
      pillar_config, map_frame_, "my_orange_car");
    load_existing_map();

    publisher_ = create_publisher<std_msgs::msg::String>(output_topic_, 10);
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_, rclcpp::QoS(10).transient_local());
    pillar_marker_topic_ = declare_parameter<std::string>(
      "pillar_marker_topic", "/semantic/pillar_landmark_markers");
    road_node_marker_topic_ = declare_parameter<std::string>(
      "road_node_marker_topic", "/semantic/virtual_road_node_markers");
    road_node_config_.slot_lane_extension_m = declare_parameter<double>(
      "slot_lane_extension_m", 2.07);
    road_node_config_.pillar_extension_m = declare_parameter<double>(
      "pillar_extension_m", 2.0);
    road_node_config_.lane_center_x = declare_parameter<double>(
      "lane_center_x", 0.0);
    road_node_config_.lane_center_y = declare_parameter<double>(
      "lane_center_y", 0.0);
    road_node_config_.pillar_slot_adjacency_max_distance_m = declare_parameter<double>(
      "pillar_slot_adjacency_max_distance_m", 8.0);
    road_node_config_.obstacle_clearance_m = declare_parameter<double>(
      "road_node_obstacle_clearance_m", 0.10);
    road_node_config_.duplicate_distance_m = declare_parameter<double>(
      "road_node_duplicate_distance_m", 0.25);
    road_graph_config_.max_neighbor_distance_m = declare_parameter<double>(
      "road_node_max_neighbor_distance_m", 5.0);
    pillar_marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      pillar_marker_topic_, rclcpp::QoS(10).transient_local());
    road_node_marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      road_node_marker_topic_, rclcpp::QoS(10).transient_local());
    subscription_ = create_subscription<std_msgs::msg::String>(
      input_topic_,
      rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&ParkingSlotMapFusion::on_observations, this, std::placeholders::_1));
    pillar_subscription_ = create_subscription<std_msgs::msg::String>(
      pillar_input_topic_, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&ParkingSlotMapFusion::on_pillar_observations, this, std::placeholders::_1));
    save_service_ = create_service<std_srvs::srv::Trigger>(
      save_service_name_,
      std::bind(
        &ParkingSlotMapFusion::on_save_request, this,
        std::placeholders::_1, std::placeholders::_2));

    publish_map();
    RCLCPP_INFO(
      get_logger(),
      "semantic fusion listening on %s and %s; publishing %s and landmark markers; map file: %s; save service: %s",
      input_topic_.c_str(), pillar_input_topic_.c_str(), output_topic_.c_str(),
      map_path_.empty() ? "<not configured>" : map_path_.c_str(),
      save_service_name_.c_str());
  }

private:
  void load_existing_map()
  {
    if (map_path_.empty()) return;
    if (!std::filesystem::exists(map_path_)) {
      RCLCPP_INFO(get_logger(), "semantic map does not exist yet; starting with an empty map: %s",
        map_path_.c_str());
      return;
    }
    try {
      const auto loaded = parking_lot_core::load_map(map_path_);
      parking_lot_core::ParkingMap slots;
      parking_lot_core::ParkingMap pillars;
      slots.metadata = loaded.metadata;
      pillars.metadata = loaded.metadata;
      for (const auto & landmark : loaded.landmarks) {
        if (landmark.type == "pillar") pillars.landmarks.push_back(landmark);
        else slots.landmarks.push_back(landmark);
      }
      slot_store_->load(slots);
      pillar_store_->load(pillars);
      RCLCPP_INFO(
        get_logger(), "loaded %zu parking-slot and %zu pillar landmark(s) from %s",
        slot_store_->snapshot().landmarks.size(),
        pillar_store_->snapshot().landmarks.size(), map_path_.c_str());
    } catch (const std::exception & error) {
      throw std::runtime_error(
              "cannot load semantic landmark map '" + map_path_ + "': " + error.what());
    }
  }

  bool lookup_transform(
    double stamp_sec,
    geometry_msgs::msg::TransformStamped & transform)
  {
    const rclcpp::Time requested(
      static_cast<int64_t>(stamp_sec * 1e9), RCL_ROS_TIME);
    try {
      transform = tf_buffer_.lookupTransform(
        map_frame_, base_frame_, requested,
        rclcpp::Duration::from_seconds(tf_tolerance_sec_));
      return true;
    } catch (const tf2::TransformException &) {
      // The odometry bridge and camera can differ by a few milliseconds.
      try {
        const auto latest = tf_buffer_.lookupTransform(
          map_frame_, base_frame_, tf2::TimePointZero);
        const rclcpp::Time latest_time(latest.header.stamp);
        if (std::abs((latest_time - requested).seconds()) <= tf_tolerance_sec_) {
          transform = latest;
          return true;
        }
      } catch (const tf2::TransformException &) {
      }
    }
    return false;
  }

  ParkingSlotObservation transform_observation(
    const ParkingSlotObservation & local,
    const geometry_msgs::msg::TransformStamped & transform) const
  {
    geometry_msgs::msg::PointStamped point;
    point.header.frame_id = base_frame_;
    point.point.x = local.x;
    point.point.y = local.y;
    point.point.z = 0.0;
    geometry_msgs::msg::PointStamped transformed;
    tf2::doTransform(point, transformed, transform);

    ParkingSlotObservation result = local;
    result.x = transformed.point.x;
    result.y = transformed.point.y;
    result.yaw = normalize_angle(
      tf2::getYaw(transform.transform.rotation) + local.yaw);
    return result;
  }

  PillarLandmarkObservation transform_pillar_observation(
    const PillarLandmarkObservation & local,
    const geometry_msgs::msg::TransformStamped & transform) const
  {
    geometry_msgs::msg::PointStamped point;
    point.header.frame_id = base_frame_;
    point.point.x = local.x;
    point.point.y = local.y;
    point.point.z = 0.0;
    geometry_msgs::msg::PointStamped transformed;
    tf2::doTransform(point, transformed, transform);
    return PillarLandmarkObservation{
      transformed.point.x, transformed.point.y, local.confidence};
  }

  static double normalize_angle(double angle)
  {
    constexpr double pi = 3.14159265358979323846;
    while (angle >= pi) angle -= 2.0 * pi;
    while (angle < -pi) angle += 2.0 * pi;
    return angle;
  }

  void on_observations(const std_msgs::msg::String::SharedPtr message)
  {
    double stamp = 0.0;
    std::vector<ParkingSlotObservation> local_observations;
    std::string error;
    if (!parse_observations(
        message->data, base_frame_, stamp, local_observations, error))
    {
      slot_store_->update({});
      publish_map();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.c_str());
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    if (!lookup_transform(stamp, transform)) {
      slot_store_->update({});
      publish_map();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "dropping parking-slot frame at %.3f: no %s -> %s TF within %.3f s",
        stamp, map_frame_.c_str(), base_frame_.c_str(), tf_tolerance_sec_);
      return;
    }

    std::vector<ParkingSlotObservation> map_observations;
    map_observations.reserve(local_observations.size());
    for (const auto & observation : local_observations) {
      map_observations.push_back(transform_observation(observation, transform));
    }
    slot_store_->update(map_observations);
    publish_map();
  }

  void on_pillar_observations(const std_msgs::msg::String::SharedPtr message)
  {
    double stamp = 0.0;
    std::string status;
    std::vector<PillarLandmarkObservation> local_observations;
    std::string error;
    if (!parse_pillar_observations(
        message->data, base_frame_, stamp, status, local_observations, error))
    {
      pillar_store_->update({});
      publish_map();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.c_str());
      return;
    }
    if (status != "ok" || local_observations.empty()) {
      pillar_store_->update({});
      publish_map();
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    if (!lookup_transform(stamp, transform)) {
      pillar_store_->update({});
      publish_map();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "dropping pillar frame at %.3f: no %s -> %s TF within %.3f s",
        stamp, map_frame_.c_str(), base_frame_.c_str(), tf_tolerance_sec_);
      return;
    }

    std::vector<PillarLandmarkObservation> map_observations;
    map_observations.reserve(local_observations.size());
    for (const auto & observation : local_observations) {
      map_observations.push_back(transform_pillar_observation(observation, transform));
    }
    pillar_store_->update(map_observations);
    publish_map();
  }

  void publish_map()
  {
    const auto map = current_map();
    std_msgs::msg::String message;
    message.data = parking_lot_core::serialize_map(map);
    publisher_->publish(message);
    publish_slot_markers(map);
    publish_pillar_markers(map);
    publish_road_node_markers(map);
  }

  parking_lot_core::ParkingMap current_map() const
  {
    auto map = slot_store_->snapshot();
    const auto pillar_map = pillar_store_->snapshot();
    map.landmarks.insert(
      map.landmarks.end(), pillar_map.landmarks.begin(), pillar_map.landmarks.end());
    map.road_nodes = parking_lot_core::generate_virtual_road_nodes(map, road_node_config_);
    parking_lot_core::build_virtual_road_node_neighbors(map.road_nodes, road_graph_config_);
    map.edges.clear();
    return map;
  }

  void publish_slot_markers(const parking_lot_core::ParkingMap & map)
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = map_frame_;
    clear.header.stamp = now();
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    constexpr double fallback_length = 5.8;
    constexpr double fallback_width = 2.8;
    const auto geometry_value = [](
      const parking_lot_core::LandmarkNode & landmark,
      const std::string & key,
      double fallback) {
        const auto found = landmark.geometry.find(key);
        return found == landmark.geometry.end() ? fallback : found->second;
      };

    for (std::size_t index = 0; index < map.landmarks.size(); ++index) {
      const auto & landmark = map.landmarks[index];
      if (landmark.type != "parking_slot") continue;
      const double length = geometry_value(landmark, "length", fallback_length);
      const double width = geometry_value(landmark, "width", fallback_width);
      const double yaw = landmark.pose.yaw.value_or(0.0);
      const double cosine = std::cos(yaw);
      const double sine = std::sin(yaw);
      const std::array<std::pair<double, double>, 4> local_corners{{
        {-length * 0.5, -width * 0.5},
        {length * 0.5, -width * 0.5},
        {length * 0.5, width * 0.5},
        {-length * 0.5, width * 0.5}}};

      visualization_msgs::msg::Marker outline;
      outline.header.frame_id = map_frame_;
      outline.header.stamp = now();
      outline.ns = "parking_slot_outline";
      outline.id = static_cast<int32_t>(index);
      outline.type = visualization_msgs::msg::Marker::LINE_STRIP;
      outline.action = visualization_msgs::msg::Marker::ADD;
      outline.pose.orientation.w = 1.0;
      outline.scale.x = 0.06;
      outline.color.r = 0.05F;
      outline.color.g = 0.95F;
      outline.color.b = 1.0F;
      outline.color.a = 1.0F;
      for (const auto & corner : local_corners) {
        geometry_msgs::msg::Point point;
        point.x = landmark.pose.x + cosine * corner.first - sine * corner.second;
        point.y = landmark.pose.y + sine * corner.first + cosine * corner.second;
        point.z = 0.08;
        outline.points.push_back(point);
      }
      outline.points.push_back(outline.points.front());
      markers.markers.push_back(std::move(outline));

      visualization_msgs::msg::Marker center;
      center.header.frame_id = map_frame_;
      center.header.stamp = now();
      center.ns = "parking_slot_center";
      center.id = static_cast<int32_t>(index);
      center.type = visualization_msgs::msg::Marker::SPHERE;
      center.action = visualization_msgs::msg::Marker::ADD;
      center.pose.position.x = landmark.pose.x;
      center.pose.position.y = landmark.pose.y;
      center.pose.position.z = 0.16;
      center.pose.orientation.w = 1.0;
      center.scale.x = 0.20;
      center.scale.y = 0.20;
      center.scale.z = 0.20;
      center.color.r = 1.0F;
      center.color.g = 0.10F;
      center.color.b = 0.85F;
      center.color.a = 1.0F;
      markers.markers.push_back(std::move(center));

      visualization_msgs::msg::Marker label;
      label.header.frame_id = map_frame_;
      label.header.stamp = now();
      label.ns = "parking_slot_label";
      label.id = static_cast<int32_t>(index);
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position.x = landmark.pose.x + 0.15;
      label.pose.position.y = landmark.pose.y + 0.15;
      label.pose.position.z = 0.45;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.35;
      label.color.r = 1.0F;
      label.color.g = 1.0F;
      label.color.b = 1.0F;
      label.color.a = 1.0F;
      label.text = landmark.id;
      markers.markers.push_back(std::move(label));
    }

    marker_publisher_->publish(markers);
  }

  void publish_pillar_markers(const parking_lot_core::ParkingMap & map)
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = map_frame_;
    clear.header.stamp = now();
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    std::size_t marker_id = 0U;
    for (const auto & landmark : map.landmarks) {
      if (landmark.type != "pillar") continue;
      const auto size_it = landmark.geometry.find("side_length");
      const double side_length = size_it == landmark.geometry.end() ? 1.2 : size_it->second;

      visualization_msgs::msg::Marker body;
      body.header.frame_id = map_frame_;
      body.header.stamp = now();
      body.ns = "pillar_landmark";
      body.id = static_cast<int32_t>(marker_id);
      body.type = visualization_msgs::msg::Marker::CUBE;
      body.action = visualization_msgs::msg::Marker::ADD;
      body.pose.position.x = landmark.pose.x;
      body.pose.position.y = landmark.pose.y;
      body.pose.position.z = 0.45;
      body.pose.orientation.w = 1.0;
      body.scale.x = side_length;
      body.scale.y = side_length;
      body.scale.z = 0.9;
      body.color.r = 0.10F;
      body.color.g = 0.75F;
      body.color.b = 1.0F;
      body.color.a = 0.65F;
      markers.markers.push_back(std::move(body));

      visualization_msgs::msg::Marker label;
      label.header.frame_id = map_frame_;
      label.header.stamp = now();
      label.ns = "pillar_landmark_label";
      label.id = static_cast<int32_t>(marker_id);
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position.x = landmark.pose.x;
      label.pose.position.y = landmark.pose.y;
      label.pose.position.z = 1.1;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.30;
      label.color.r = 1.0F;
      label.color.g = 1.0F;
      label.color.b = 1.0F;
      label.color.a = 1.0F;
      label.text = landmark.id;
      markers.markers.push_back(std::move(label));
      ++marker_id;
    }

    pillar_marker_publisher_->publish(markers);
  }

  void publish_road_node_markers(const parking_lot_core::ParkingMap & map)
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = map_frame_;
    clear.header.stamp = now();
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    for (std::size_t index = 0; index < map.road_nodes.size(); ++index) {
      const auto & node = map.road_nodes[index];
      visualization_msgs::msg::Marker body;
      body.header.frame_id = map_frame_;
      body.header.stamp = now();
      body.ns = "virtual_road_node";
      body.id = static_cast<int32_t>(index);
      body.type = visualization_msgs::msg::Marker::SPHERE;
      body.action = visualization_msgs::msg::Marker::ADD;
      body.pose.position.x = node.pose.x;
      body.pose.position.y = node.pose.y;
      body.pose.position.z = 0.25;
      body.pose.orientation.w = 1.0;
      body.scale.x = 0.28;
      body.scale.y = 0.28;
      body.scale.z = 0.28;
      if (node.node_type == "parking_anchor") {
        body.color.r = 1.0F;
        body.color.g = 0.65F;
        body.color.b = 0.05F;
      } else {
        body.color.r = 0.75F;
        body.color.g = 0.15F;
        body.color.b = 1.0F;
      }
      body.color.a = 1.0F;
      markers.markers.push_back(std::move(body));

      visualization_msgs::msg::Marker label;
      label.header.frame_id = map_frame_;
      label.header.stamp = now();
      label.ns = "virtual_road_node_label";
      label.id = static_cast<int32_t>(index);
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position.x = node.pose.x + 0.15;
      label.pose.position.y = node.pose.y + 0.15;
      label.pose.position.z = 0.55;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.24;
      label.color.r = 1.0F;
      label.color.g = 1.0F;
      label.color.b = 1.0F;
      label.color.a = 1.0F;
      label.text = node.id;
      markers.markers.push_back(std::move(label));
    }
    road_node_marker_publisher_->publish(markers);
  }

  void on_save_request(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (map_path_.empty()) {
      response->success = false;
      response->message = "map_path is empty; configure a semantic map JSON path first";
      return;
    }
    try {
      const auto map = current_map();
      parking_lot_core::save_map(map, map_path_);
      response->success = true;
      const auto pillar_count = std::count_if(
        map.landmarks.begin(), map.landmarks.end(), [](const auto & landmark) {
          return landmark.type == "pillar";
        });
      const auto slot_count = map.landmarks.size() - pillar_count;
      response->message = "Saved " + std::to_string(map.landmarks.size()) +
        " semantic landmark(s) and " + std::to_string(map.road_nodes.size()) +
        " virtual road node(s): " + std::to_string(slot_count) + " parking slot(s), " +
        std::to_string(pillar_count) + " pillar(s) to " + map_path_;
    } catch (const std::exception & error) {
      response->success = false;
      response->message = "Failed to save semantic map to " + map_path_ + ": " + error.what();
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    }
  }

  std::string input_topic_;
  std::string pillar_input_topic_;
  std::string output_topic_;
  std::string marker_topic_;
  std::string pillar_marker_topic_;
  std::string road_node_marker_topic_;
  std::string save_service_name_;
  std::string map_path_;
  std::string base_frame_;
  std::string map_frame_;
  double tf_tolerance_sec_{0.1};
  parking_lot_core::VirtualRoadNodeConfig road_node_config_;
  parking_lot_core::VirtualRoadGraphConfig road_graph_config_;
  std::unique_ptr<ParkingSlotMapStore> slot_store_;
  std::unique_ptr<PillarLandmarkStore> pillar_store_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pillar_marker_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr road_node_marker_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pillar_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
};

}  // namespace parking_lot_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<parking_lot_perception::ParkingSlotMapFusion>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("parking_slot_map_fusion"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
