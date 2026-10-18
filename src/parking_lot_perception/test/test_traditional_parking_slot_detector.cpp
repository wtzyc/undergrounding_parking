#include "parking_lot_perception/traditional_parking_slot_detector.hpp"

#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>
#include <rapidjson/document.h>

namespace parking_lot_perception
{
namespace
{

void draw_slot(cv::Mat & image, cv::Point top_left, cv::Point bottom_right)
{
  cv::rectangle(image, top_left, bottom_right, cv::Scalar(150, 115, 95), cv::FILLED);
  cv::rectangle(image, top_left, bottom_right, cv::Scalar(255, 255, 255), 4);
}

void draw_rotated_slot(cv::Mat & image, const cv::RotatedRect & rectangle)
{
  cv::Point2f raw_corners[4];
  rectangle.points(raw_corners);
  std::vector<cv::Point> corners;
  for (const auto & point : raw_corners) {
    corners.emplace_back(cvRound(point.x), cvRound(point.y));
  }
  cv::fillConvexPoly(image, corners, cv::Scalar(150, 115, 95));
  cv::polylines(image, corners, true, cv::Scalar(255, 255, 255), 4, cv::LINE_AA);
}

TEST(TraditionalParkingSlotDetector, FindsBayFromColorLinesAndGeometry)
{
  cv::Mat image(800, 800, CV_8UC3, cv::Scalar(80, 130, 105));
  draw_slot(image, {255, 330}, {545, 470});

  TraditionalParkingSlotDetector detector;
  const auto detection = detector.detect(image);

  ASSERT_EQ(detection.accepted.size(), 1U);
  EXPECT_TRUE(detection.rejected.empty());
  EXPECT_GE(detection.accepted.front().supported_edges, 3);
  EXPECT_FALSE(detection.accepted.front().partial_or_occluded);
  EXPECT_GT(cv::countNonZero(detection.slot_mask), 30000);
  EXPECT_GT(cv::countNonZero(detection.white_line_mask), 1000);
}

TEST(TraditionalParkingSlotDetector, PublishesPoseForPartialCandidateAndDrawsItGreen)
{
  cv::Mat image(800, 800, CV_8UC3, cv::Scalar(80, 130, 105));
  draw_slot(image, {-20, 330}, {270, 470});

  TraditionalParkingSlotDetector detector;
  const auto detection = detector.detect(image);

  ASSERT_EQ(detection.accepted.size(), 1U);
  EXPECT_TRUE(detection.accepted.front().partial_or_occluded);
  const auto json = detector.candidates_to_json(detection.accepted, 800, 800, 12.5);
  EXPECT_NE(json.find("\"partial_or_occluded\":true"), std::string::npos);
  EXPECT_NE(json.find("\"x\":"), std::string::npos);
  EXPECT_NE(json.find("\"y\":"), std::string::npos);
  EXPECT_NE(json.find("\"yaw\":"), std::string::npos);
  rapidjson::Document document;
  ASSERT_FALSE(document.Parse(json.c_str()).HasParseError());
  const auto & candidate = document["candidates"][0];
  ASSERT_TRUE(candidate["pose_extrapolated"].GetBool());
  EXPECT_NEAR(candidate["length"].GetDouble(), 5.8, 1e-6);
  EXPECT_NEAR(candidate["width"].GetDouble(), 2.8, 1e-6);

  const auto & detected_corners = detection.accepted.front().corners_px;
  std::size_t clipped_edge = detected_corners.size();
  for (std::size_t index = 0; index < detected_corners.size(); ++index) {
    const auto & first = detected_corners[index];
    const auto & second = detected_corners[(index + 1) % detected_corners.size()];
    if (std::abs(first.x) < 1.5F && std::abs(second.x) < 1.5F) clipped_edge = index;
  }
  ASSERT_LT(clipped_edge, detected_corners.size());
  const std::size_t clipped_next = (clipped_edge + 1) % detected_corners.size();
  const std::size_t visible = (clipped_edge + 2) % detected_corners.size();
  const std::size_t visible_next = (clipped_edge + 3) % detected_corners.size();
  const cv::Point2f clipped_midpoint = 0.5F * (
    detected_corners[clipped_edge] + detected_corners[clipped_next]);
  const cv::Point2f visible_midpoint = 0.5F * (
    detected_corners[visible] + detected_corners[visible_next]);
  const cv::Point2f outward = clipped_midpoint - visible_midpoint;
  const cv::Point2f expected_center_px = visible_midpoint + outward * static_cast<float>(
    (5.8 / 0.02 * 0.5) / cv::norm(outward));
  const double expected_x = -8.0 + (799.0 - expected_center_px.y) * 0.02;
  const double expected_y = -8.0 + (799.0 - expected_center_px.x) * 0.02;
  EXPECT_NEAR(candidate["x"].GetDouble(), expected_x, 1e-5);
  EXPECT_NEAR(candidate["y"].GetDouble(), expected_y, 1e-5);
  EXPECT_EQ(candidate["completed_pixel_corners"].Size(), 4U);

  const cv::Mat debug = detector.draw_debug(image, detection);
  int green_pixels = 0;
  for (int row = 0; row < debug.rows; ++row) {
    for (int column = 0; column < debug.cols; ++column) {
      const auto pixel = debug.at<cv::Vec3b>(row, column);
      if (pixel[1] > pixel[0] + 80 && pixel[1] > pixel[2] + 80) ++green_pixels;
    }
  }
  EXPECT_GT(green_pixels, 0);
}

TEST(TraditionalParkingSlotDetector, CompletesAngledSlotClippedByImageBorder)
{
  cv::Mat image(800, 800, CV_8UC3, cv::Scalar(80, 130, 105));
  const cv::Point2f expected_center_px(700.0F, 400.0F);
  draw_rotated_slot(image, cv::RotatedRect(expected_center_px, cv::Size2f(290.0F, 140.0F), 25.0F));

  TraditionalParkingSlotDetector detector;
  const auto detection = detector.detect(image);

  ASSERT_EQ(detection.accepted.size(), 1U);
  EXPECT_TRUE(detection.accepted.front().touches_right_border);
  const auto json = detector.candidates_to_json(detection.accepted, 800, 800, 1.0);
  rapidjson::Document document;
  ASSERT_FALSE(document.Parse(json.c_str()).HasParseError());
  const auto & candidate = document["candidates"][0];
  ASSERT_TRUE(candidate["pose_extrapolated"].GetBool());
  EXPECT_NEAR(candidate["length"].GetDouble(), 5.8, 1e-6);
  EXPECT_NEAR(candidate["width"].GetDouble(), 2.8, 1e-6);

  const double expected_x = -8.0 + (799.0 - expected_center_px.y) * 0.02;
  const double expected_y = -8.0 + (799.0 - expected_center_px.x) * 0.02;
  EXPECT_NEAR(candidate["x"].GetDouble(), expected_x, 0.20);
  EXPECT_NEAR(candidate["y"].GetDouble(), expected_y, 0.20);
}

TEST(TraditionalParkingSlotDetector, RejectsWrongSizeColorRegion)
{
  cv::Mat image(800, 800, CV_8UC3, cv::Scalar(80, 130, 105));
  draw_slot(image, {350, 350}, {450, 450});

  TraditionalParkingSlotDetector detector;
  const auto detection = detector.detect(image);

  EXPECT_TRUE(detection.accepted.empty());
  ASSERT_EQ(detection.rejected.size(), 1U);
  EXPECT_FALSE(detection.rejected.front().rejection_reasons.empty());

  const auto json = detector.candidates_to_json(detection.accepted, 800, 800, 1.0);
  rapidjson::Document document;
  ASSERT_FALSE(document.Parse(json.c_str()).HasParseError());
  ASSERT_TRUE(document.HasMember("candidates"));
  EXPECT_TRUE(document["candidates"].IsArray());
  EXPECT_EQ(document["candidates"].Size(), 0U);
}

}  // namespace
}  // namespace parking_lot_perception
