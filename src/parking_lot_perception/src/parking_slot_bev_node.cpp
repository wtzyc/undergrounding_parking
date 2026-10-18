#include "parking_lot_perception/traditional_parking_slot_detector.hpp"

#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <stdexcept>
#include <string>

namespace parking_lot_perception
{

class ParkingSlotBevNode final : public rclcpp::Node
{
public:
  ParkingSlotBevNode()
  : Node("parking_slot_bev_detector")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/camera_god/bev_image");
    candidate_topic_ = declare_parameter<std::string>(
      "candidate_topic", "/semantic/parking_slot_candidates");
    debug_topic_ = declare_parameter<std::string>(
      "debug_topic", "/camera_god/parking_slot_debug_image");

    TraditionalParkingSlotDetectorConfig config;
    config.resolution_m_per_pixel = declare_parameter<double>(
      "resolution_m_per_pixel", config.resolution_m_per_pixel);
    config.x_min = declare_parameter<double>("x_min", config.x_min);
    config.y_min = declare_parameter<double>("y_min", config.y_min);
    config.expected_width_m = declare_parameter<double>(
      "expected_width_m", config.expected_width_m);
    config.expected_length_m = declare_parameter<double>(
      "expected_length_m", config.expected_length_m);
    config.min_component_area_px = declare_parameter<int>(
      "min_component_area_px", config.min_component_area_px);
    config.min_rectangularity = declare_parameter<double>(
      "min_rectangularity", config.min_rectangularity);
    config.min_aspect_ratio = declare_parameter<double>(
      "min_aspect_ratio", config.min_aspect_ratio);
    config.max_aspect_ratio = declare_parameter<double>(
      "max_aspect_ratio", config.max_aspect_ratio);
    config.short_side_tolerance_fraction = declare_parameter<double>(
      "short_side_tolerance_fraction", config.short_side_tolerance_fraction);
    config.min_long_side_fraction = declare_parameter<double>(
      "min_long_side_fraction", config.min_long_side_fraction);
    config.max_long_side_fraction = declare_parameter<double>(
      "max_long_side_fraction", config.max_long_side_fraction);
    config.min_blue_red_difference = declare_parameter<int>(
      "min_blue_red_difference", config.min_blue_red_difference);
    config.min_blue_green_difference = declare_parameter<int>(
      "min_blue_green_difference", config.min_blue_green_difference);
    config.min_blue_value = declare_parameter<int>(
      "min_blue_value", config.min_blue_value);
    config.min_white_value = declare_parameter<int>(
      "min_white_value", config.min_white_value);
    config.max_white_chroma = declare_parameter<int>(
      "max_white_chroma", config.max_white_chroma);
    config.min_line_support = declare_parameter<double>(
      "min_line_support", config.min_line_support);
    config.min_supported_edges = declare_parameter<int>(
      "min_supported_edges", config.min_supported_edges);
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    detector_ = std::make_unique<TraditionalParkingSlotDetector>(config);

    // Detection candidates describe the current frame; don't let stale results queue up.
    candidate_publisher_ = create_publisher<std_msgs::msg::String>(
      candidate_topic_, rclcpp::SensorDataQoS().keep_last(1));
    debug_publisher_ = create_publisher<sensor_msgs::msg::Image>(
      debug_topic_, rclcpp::SensorDataQoS().keep_last(1));
    image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic_,
      rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&ParkingSlotBevNode::on_image, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(),
      "traditional color/line detector listening on %s, output frame %s",
      input_topic_.c_str(), base_frame_.c_str());
  }

private:
  void on_image(const sensor_msgs::msg::Image::SharedPtr message)
  {
    try {
      const cv::Mat image = decode_image(*message);
      const auto detection = detector_->detect(image);
      std_msgs::msg::String output;
      // Rejected (orange) candidates stay in the debug image only; semantic
      // fusion receives accepted (green) candidates and confirms them over time.
      output.data = detector_->candidates_to_json(
        detection.accepted,
        static_cast<int>(message->width),
        static_cast<int>(message->height),
        static_cast<double>(message->header.stamp.sec) +
        static_cast<double>(message->header.stamp.nanosec) / 1e9,
        base_frame_);
      candidate_publisher_->publish(output);
      if (debug_publisher_->get_subscription_count() > 0) {
        debug_publisher_->publish(encode_image(detector_->draw_debug(image, detection), *message));
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "parking slot detection failed: %s", error.what());
    }
  }

  static cv::Mat decode_image(const sensor_msgs::msg::Image & message)
  {
    if (message.encoding == "bgr8") {
      return cv::Mat(
        static_cast<int>(message.height),
        static_cast<int>(message.width),
        CV_8UC3,
        const_cast<unsigned char *>(message.data.data()),
        message.step).clone();
    }
    if (message.encoding == "rgb8") {
      cv::Mat rgb(
        static_cast<int>(message.height),
        static_cast<int>(message.width),
        CV_8UC3,
        const_cast<unsigned char *>(message.data.data()),
        message.step);
      cv::Mat bgr;
      cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
      return bgr;
    }
    if (message.encoding == "mono8") {
      cv::Mat gray(
        static_cast<int>(message.height),
        static_cast<int>(message.width),
        CV_8UC1,
        const_cast<unsigned char *>(message.data.data()),
        message.step);
      cv::Mat bgr;
      cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
      return bgr;
    }
    throw std::invalid_argument("supported image encodings are bgr8, rgb8 and mono8");
  }

  static sensor_msgs::msg::Image encode_image(
    const cv::Mat & image,
    const sensor_msgs::msg::Image & source)
  {
    sensor_msgs::msg::Image result;
    result.header = source.header;
    result.encoding = "bgr8";
    result.height = static_cast<uint32_t>(image.rows);
    result.width = static_cast<uint32_t>(image.cols);
    result.is_bigendian = false;
    result.step = static_cast<uint32_t>(image.cols * 3);
    result.data.assign(
      image.data,
      image.data + static_cast<std::size_t>(image.rows) * result.step);
    return result;
  }

  std::string input_topic_;
  std::string candidate_topic_;
  std::string debug_topic_;
  std::string base_frame_;
  std::unique_ptr<TraditionalParkingSlotDetector> detector_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr candidate_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
};

}  // namespace parking_lot_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<parking_lot_perception::ParkingSlotBevNode>());
  rclcpp::shutdown();
  return 0;
}
