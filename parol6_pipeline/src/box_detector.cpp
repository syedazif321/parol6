#include "parol6_pipeline/box_detector.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cmath>
#include <sstream>
#include <numeric>

BoxDetector::BoxDetector()
: Node("box_detector_node"),
  fx_(declare_parameter("fx", 554.3827)),
  fy_(declare_parameter("fy", 554.3827)),
  cx_(declare_parameter("cx", 320.5)),
  cy_(declare_parameter("cy", 240.5)),
  min_area_(declare_parameter("min_area", 300.0)),
  camera_frame_(declare_parameter("camera_frame", "camera_link"))
{
  RCLCPP_INFO(this->get_logger(), "Initializing BoxDetector node");

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/detected_box_pose", 10);
  info_pub_ = this->create_publisher<std_msgs::msg::String>("/detected_box_info", 10);

  rgb_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/d435/rgb/rgb/image_raw");
  depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/d435/depth/depth/depth/image_raw");

  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *rgb_sub_, *depth_sub_);
  sync_->registerCallback(std::bind(&BoxDetector::imageCallback, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Subscribers and synchronizer initialized");
}

void BoxDetector::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
{
  try {
    auto rgb_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
    auto depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);

    cv::Mat hsv;
    cv::cvtColor(rgb_ptr->image, hsv, cv::COLOR_BGR2HSV);

    cv::Mat red_mask, blue_mask;
    cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), red_mask);
    cv::inRange(hsv, cv::Scalar(100, 150, 0), cv::Scalar(140, 255, 255), blue_mask);

    detectBoxes(rgb_ptr->image, depth_ptr->image, red_mask, "Red");
    detectBoxes(rgb_ptr->image, depth_ptr->image, blue_mask, "Blue");

    cv::imshow("RGB Detection", rgb_ptr->image);
    cv::waitKey(1);

  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

void BoxDetector::detectBoxes(const cv::Mat& image, const cv::Mat& depth, const cv::Mat& mask, const std::string& color)
{
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  RCLCPP_INFO(this->get_logger(), "[%s] Contours found: %lu", color.c_str(), contours.size());

  for (const auto& cnt : contours)
  {
    double area = cv::contourArea(cnt);
    if (area < min_area_) continue;

    cv::RotatedRect rect = cv::minAreaRect(cnt);
    cv::Point2f center = rect.center;
    float angle = rect.angle;
    float w = rect.size.width;
    float h = rect.size.height;

    float size_px = std::max(w, h);
    int cx = static_cast<int>(center.x);
    int cy = static_cast<int>(center.y);

    if (cx < 0 || cx >= depth.cols || cy < 0 || cy >= depth.rows) {
      RCLCPP_WARN(this->get_logger(), "Invalid pixel index: (%d, %d)", cx, cy);
      continue;
    }

    float depth_val = depth.at<float>(cy, cx);

    // Try to recover invalid depth using surrounding window
    if (depth_val <= 0.0f || std::isnan(depth_val) || std::isinf(depth_val)) {
      const int window = 2;
      std::vector<float> valid_depths;

      for (int dy = -window; dy <= window; ++dy) {
        for (int dx = -window; dx <= window; ++dx) {
          int nx = cx + dx;
          int ny = cy + dy;
          if (nx >= 0 && nx < depth.cols && ny >= 0 && ny < depth.rows) {
            float val = depth.at<float>(ny, nx);
            if (val > 0.0f && !std::isnan(val) && !std::isinf(val)) {
              valid_depths.push_back(val);
            }
          }
        }
      }

      if (!valid_depths.empty()) {
        depth_val = std::accumulate(valid_depths.begin(), valid_depths.end(), 0.0f) / valid_depths.size();
        RCLCPP_WARN(this->get_logger(), "[%s] Recovered depth at (%d,%d): %f", color.c_str(), cx, cy, depth_val);
      } else {
        RCLCPP_WARN(this->get_logger(), "[%s] No valid depth found near (%d,%d)", color.c_str(), cx, cy);
        continue;
      }
    }

    float X = (center.x - cx_) * depth_val / fx_;
    float Y = (center.y - cy_) * depth_val / fy_;
    float Z = depth_val;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->now();
    pose.header.frame_id = camera_frame_;
    pose.pose.position.x = X;
    pose.pose.position.y = Y;
    pose.pose.position.z = Z;

    yawToQuaternion(-angle * M_PI / 180.0f, pose.pose.orientation);
    pose_pub_->publish(pose);

    std_msgs::msg::String info;
    std::ostringstream oss;
    oss << "{color:" << color
        << ", size:" << estimateSize(size_px)
        << ", depth:" << Z
        << ", yaw:" << angle << "}";
    info.data = oss.str();
    info_pub_->publish(info);

    cv::drawContours(image, std::vector<std::vector<cv::Point>>{cnt}, -1, cv::Scalar(0,255,0), 2);
    cv::putText(image, color, center, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255), 1);
  }
}

float BoxDetector::estimateSize(float px)
{
  if (px > 90) return 0.12f;
  else if (px > 60) return 0.10f;
  else if (px > 40) return 0.09f;
  else return 0.08f;
}

void BoxDetector::yawToQuaternion(float yaw, geometry_msgs::msg::Quaternion &q)
{
  q.x = 0.0;
  q.y = 0.0;
  q.z = sin(yaw / 2.0);
  q.w = cos(yaw / 2.0);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BoxDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
