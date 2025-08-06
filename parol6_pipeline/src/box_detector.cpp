#include "parol6_pipeline/box_detector.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iomanip>
#include <cmath>

BoxDetector::BoxDetector()
: Node("box_detector_node"),
  fx_(declare_parameter("fx", 554.3827)),
  fy_(declare_parameter("fy", 554.3827)),
  cx_(declare_parameter("cx", 320.5)),
  cy_(declare_parameter("cy", 240.5)),
  min_area_(declare_parameter("min_area", 300.0)),
  camera_frame_(declare_parameter("camera_frame", "realsense_rgb_frame"))
{
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/detected_box_pose", 10);
  info_pub_ = create_publisher<std_msgs::msg::String>("/detected_box_info", 10);

  rgb_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/camera/image_raw");
  depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/camera/depth/image_raw");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *rgb_sub_, *depth_sub_);
  sync_->registerCallback(std::bind(&BoxDetector::imageCallback, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "BoxDetector initialized");
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

    detectBoxes(rgb_ptr->image, depth_ptr->image, createRedMask(hsv), "Red");
    detectBoxes(rgb_ptr->image, depth_ptr->image, createBlueMask(hsv), "Blue");

    // Show debug views
    cv::imshow("RGB Detection", rgb_ptr->image);
    cv::Mat depth_vis;
    depth_ptr->image.convertTo(depth_vis, CV_8UC1, 255.0 / 5.0);
    cv::applyColorMap(depth_vis, depth_vis, cv::COLORMAP_JET);
    cv::imshow("Depth View", depth_vis);
    cv::waitKey(1);
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
  }
}

void BoxDetector::detectBoxes(const cv::Mat& image, const cv::Mat& depth, const cv::Mat& mask, const std::string& color)
{
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  RCLCPP_INFO(get_logger(), "[%s] %lu contours", color.c_str(), contours.size());

  for (const auto& cnt : contours)
  {
    if (cv::contourArea(cnt) < min_area_) continue;

    cv::RotatedRect rect = cv::minAreaRect(cnt);
    cv::Point2f center = rect.center;
    int cx = static_cast<int>(center.x);
    int cy = static_cast<int>(center.y);

    if (cx < 0 || cx >= depth.cols || cy < 0 || cy >= depth.rows) continue;

    float depth_val = depth.at<float>(cy, cx);
    if (!std::isfinite(depth_val) || depth_val <= 0.0f) {
      const int window = 5;
      std::vector<float> valid;

      for (int dy = -window; dy <= window; ++dy) {
        for (int dx = -window; dx <= window; ++dx) {
          int nx = cx + dx, ny = cy + dy;
          if (nx >= 0 && nx < depth.cols && ny >= 0 && ny < depth.rows) {
            float val = depth.at<float>(ny, nx);
            if (std::isfinite(val) && val > 0.0f)
              valid.push_back(val);
          }
        }
      }

      if (!valid.empty()) {
        std::nth_element(valid.begin(), valid.begin() + valid.size()/2, valid.end());
        depth_val = valid[valid.size()/2];
      } else {
        RCLCPP_WARN(get_logger(), "[%s] No valid depth around (%d, %d)", color.c_str(), cx, cy);
        continue;
      }
    }

    float X = (center.x - cx_) * depth_val / fx_;
    float Y = (center.y - cy_) * depth_val / fy_;
    float Z = depth_val;
    float size = estimateSize(std::max(rect.size.width, rect.size.height));

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = camera_frame_;
    pose.pose.position.x = X;
    pose.pose.position.y = Y;
    pose.pose.position.z = Z;
    yawToQuaternion(-rect.angle * M_PI / 180.0f, pose.pose.orientation);

    try {
      auto base_pose = tf_buffer_->transform(pose, "base_link", tf2::durationFromSec(0.1));
      pose_pub_->publish(base_pose);

      std_msgs::msg::String msg;
      std::ostringstream ss;
      ss << "{color:" << color << ",size:" << size << ",depth:" << Z << ",yaw:" << rect.angle << "}";
      msg.data = ss.str();
      info_pub_->publish(msg);

      RCLCPP_INFO(get_logger(), "[%s] BASE [%.2f, %.2f, %.2f], %.2f m, yaw=%.1f", color.c_str(),
        base_pose.pose.position.x, base_pose.pose.position.y, base_pose.pose.position.z, size, rect.angle);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(), "[%s] TF error: %s", color.c_str(), ex.what());
    }

    if (std::isfinite(center.x) && std::isfinite(center.y)) {
      if (center.x >= 0 && center.x < image.cols && center.y >= 0 && center.y < image.rows) {
        cv::drawContours(image, std::vector<std::vector<cv::Point>>{cnt}, -1, {0,255,0}, 2);
        std::ostringstream label;
        label << color << " " << std::fixed << std::setprecision(2) << size << "m @" << Z << "m";
        cv::putText(image, label.str(), center, cv::FONT_HERSHEY_SIMPLEX, 0.5, {255,255,255}, 1);
      }
    }
  }
}

float BoxDetector::estimateSize(float px)
{
  if (px > 90) return 0.12f;
  if (px > 60) return 0.10f;
  if (px > 40) return 0.09f;
  return 0.08f;
}

void BoxDetector::yawToQuaternion(float yaw, geometry_msgs::msg::Quaternion &q)
{
  q.x = 0.0;
  q.y = 0.0;
  q.z = sin(yaw / 2.0);
  q.w = cos(yaw / 2.0);
}
cv::Mat BoxDetector::createRedMask(const cv::Mat& hsv)
{
  cv::Mat mask1, mask2, red_mask;
  cv::inRange(hsv, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), mask1);
  cv::inRange(hsv, cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), mask2);
  cv::bitwise_or(mask1, mask2, red_mask);
  return red_mask;
}

cv::Mat BoxDetector::createBlueMask(const cv::Mat& hsv)
{
  cv::Mat blue_mask;
  cv::inRange(hsv, cv::Scalar(90, 50, 50), cv::Scalar(135, 255, 255), blue_mask);
  return blue_mask;
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BoxDetector>());
  rclcpp::shutdown();
  return 0;
}
