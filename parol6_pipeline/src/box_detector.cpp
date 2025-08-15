// File: src/box_detector.cpp

#include "parol6_pipeline/box_detector.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <iomanip>
#include <cmath>
#include <memory>
#include <optional>

BoxDetector::BoxDetector()
: Node("box_detector_node"),
  fx_(declare_parameter("fx", 554.3827)),
  fy_(declare_parameter("fy", 554.3827)),
  cx_(declare_parameter("cx", 320.5)),
  cy_(declare_parameter("cy", 240.5)),
  min_area_(declare_parameter("min_area", 300.0)),
  camera_frame_(declare_parameter("camera_frame", "realsense_rgb_frame")),
  detection_active_(false)
{
  info_pub_ = create_publisher<std_msgs::msg::String>("/detected_box_info", 10);
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/detected_box_pose", 10);

  rgb_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/camera/image_raw");
  depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/camera/depth/image_raw");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  sync_policy_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *rgb_sub_, *depth_sub_);

  // Create services
  start_service_ = create_service<std_srvs::srv::Trigger>(
    "/start_detection",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>&,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response
    ) {
      std::lock_guard<std::mutex> lock(detect_mutex_);
      if (detection_active_) {
        response->success = false;
        response->message = "Detection already running.";
        return;
      }
      detection_active_ = true;
      sync_policy_->registerCallback(
        std::bind(&BoxDetector::imageCallback, this, std::placeholders::_1, std::placeholders::_2));
      response->success = true;
      response->message = "Detection started.";
      RCLCPP_INFO(get_logger(), " Detection started.");
    });

  stop_service_ = create_service<std_srvs::srv::Trigger>(
    "/stop_detection",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>&,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response
    ) {
      std::lock_guard<std::mutex> lock(detect_mutex_);
      if (!detection_active_) {
        response->success = false;
        response->message = "No active detection to stop.";
        return;
      }
      detection_active_ = false;
      sync_policy_.reset();
      sync_policy_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *rgb_sub_, *depth_sub_);
      response->success = true;
      response->message = "Detection stopped.";
      RCLCPP_INFO(get_logger(), " Detection stopped.");
    });

  RCLCPP_INFO(get_logger(), "BoxDetector ready. Use /start_detection to begin.");
}

void BoxDetector::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
{
  if (!detection_active_.load()) return;

  try {
    auto rgb_ptr   = cv_bridge::toCvCopy(rgb_msg,   sensor_msgs::image_encodings::BGR8);
    auto depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);

    cv::Mat &rgb = rgb_ptr->image;
    const cv::Mat &depth = depth_ptr->image;

    // --- Build HSV + masks ---
    cv::Mat hsv;
    cv::cvtColor(rgb, hsv, cv::COLOR_BGR2HSV);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));

    cv::Mat red_mask  = createRedMask(hsv);
    cv::morphologyEx(red_mask,  red_mask,  cv::MORPH_OPEN,  kernel);
    cv::morphologyEx(red_mask,  red_mask,  cv::MORPH_CLOSE, kernel);

    cv::Mat blue_mask = createBlueMask(hsv);
    cv::morphologyEx(blue_mask, blue_mask, cv::MORPH_OPEN,  kernel);
    cv::morphologyEx(blue_mask, blue_mask, cv::MORPH_CLOSE, kernel);

    // --- Collect the single nearest candidate (in base_link) ---
    struct Candidate {
      geometry_msgs::msg::PoseStamped base_pose;
      std::string color;
      double dist;
      cv::RotatedRect rect;
    };
    std::optional<Candidate> best;

    auto processMask = [&](const cv::Mat& mask, const std::string& color) {
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      for (const auto& cnt : contours) {
        double area = cv::contourArea(cnt);
        if (area < min_area_) continue;

        cv::RotatedRect rect = cv::minAreaRect(cnt);
        cv::Point2f c = rect.center;
        int u = static_cast<int>(c.x);
        int v = static_cast<int>(c.y);
        if (u < 0 || u >= depth.cols || v < 0 || v >= depth.rows) continue;

        // Depth at (u,v) with small median fallback
        float z = depth.at<float>(v, u);
        if (!std::isfinite(z) || z <= 0.f) {
          const int w = 5;
          std::vector<float> vals;
          vals.reserve((2*w+1)*(2*w+1));
          for (int dy=-w; dy<=w; ++dy) {
            for (int dx=-w; dx<=w; ++dx) {
              int nx=u+dx, ny=v+dy;
              if (0<=nx && nx<depth.cols && 0<=ny && ny<depth.rows) {
                float val = depth.at<float>(ny, nx);
                if (std::isfinite(val) && val>0.f) vals.push_back(val);
              }
            }
          }
          if (!vals.empty()) {
            std::nth_element(vals.begin(), vals.begin()+vals.size()/2, vals.end());
            z = vals[vals.size()/2];
          } else {
            continue; // no valid depth nearby
          }
        }

        // Range check
        if (z < 0.3 || z > 3.0) continue;

        float X = (u - cx_) * z / fx_;
        float Y = (v - cy_) * z / fy_;
        float Z = z;

        tf2::Quaternion q;
        double yaw = -rect.angle * M_PI / 180.0;
        if (yaw >  M_PI/2) yaw -= M_PI;
        if (yaw < -M_PI/2) yaw += M_PI;
        tf2::Quaternion q_cam;
        q_cam.setRPY(0.0, 0.0, yaw); 

        tf2::Quaternion grasp_offset;
        grasp_offset.setRPY(0.0, 0.0, 0.0); 

        q_cam = grasp_offset * q_cam;
        q_cam.normalize();

        geometry_msgs::msg::PoseStamped cam_pose;
        cam_pose.header = rgb_msg->header;
        cam_pose.header.frame_id = camera_frame_;
        cam_pose.pose.position.x = X;
        cam_pose.pose.position.y = Y;
        cam_pose.pose.position.z = Z;
        cam_pose.pose.orientation = tf2::toMsg(q_cam);

        try {
            auto base_pose = tf_buffer_->transform(cam_pose, "base_link", tf2::durationFromSec(0.1));

            tf2::Quaternion q_base;
            tf2::fromMsg(base_pose.pose.orientation, q_base);
            double roll_b, pitch_b, yaw_b;
            tf2::Matrix3x3(q_base).getRPY(roll_b, pitch_b, yaw_b);
            tf2::Quaternion q_fixed;
            q_fixed.setRPY(M_PI, 0.0, 0.0);
            q_fixed.normalize();
            base_pose.pose.orientation = tf2::toMsg(q_fixed);

            double d = std::sqrt(
                base_pose.pose.position.x * base_pose.pose.position.x +
                base_pose.pose.position.y * base_pose.pose.position.y +
                base_pose.pose.position.z * base_pose.pose.position.z);

            if (!best.has_value() || d < best->dist) {
                best = Candidate{base_pose, color, d, rect};
            }
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[%s] TF error: %s", color.c_str(), ex.what());
            continue;
        }
      }
    };

    processMask(red_mask,  "Red");
    processMask(blue_mask, "Blue");

    if (best.has_value()) {
      auto base_pose = best->base_pose;
      base_pose.header.frame_id = "base_link";
      pose_pub_->publish(base_pose);

      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header = base_pose.header;
      tf_msg.child_frame_id = "detected_box";
      tf_msg.transform.translation.x = base_pose.pose.position.x;
      tf_msg.transform.translation.y = base_pose.pose.position.y;
      tf_msg.transform.translation.z = base_pose.pose.position.z;
      tf_msg.transform.rotation     = base_pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf_msg);

      std::ostringstream ss;
      ss << std::fixed << std::setprecision(3)
         << "{color:" << best->color
         << ", pos:[" << base_pose.pose.position.x << ", "
         << base_pose.pose.position.y << ", "
         << base_pose.pose.position.z << "], "
         << "dist:" << best->dist << "}";
      std_msgs::msg::String info;
      info.data = ss.str();
      info_pub_->publish(info);

      RCLCPP_INFO(get_logger(), "[%s] Nearest box at %.3f m: (%.3f, %.3f, %.3f)",
                  best->color.c_str(), best->dist,
                  base_pose.pose.position.x, base_pose.pose.position.y, base_pose.pose.position.z);

      // Draw rectangle & center on RGB for quick debug
      cv::Point2f verts[4];
      best->rect.points(verts);
      for (int i = 0; i < 4; i++) {
        cv::line(rgb, verts[i], verts[(i+1)%4], cv::Scalar(0,255,0), 2);
      }
      cv::circle(rgb, best->rect.center, 4, cv::Scalar(0,255,0), -1);
      cv::putText(rgb,
                  "Nearest " + best->color + " (" + std::to_string(best->dist).substr(0,4) + " m)",
                  verts[0], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 2);
    } else {
      RCLCPP_DEBUG(get_logger(), "No valid boxes found this frame.");
    }

    // Show images
    cv::imshow("RGB Detection", rgb);
    cv::Mat depth_vis;
    depth.convertTo(depth_vis, CV_8UC1, 255.0 / 5.0);
    cv::applyColorMap(depth_vis, depth_vis, cv::COLORMAP_JET);
    cv::imshow("Depth View", depth_vis);
    cv::waitKey(1);

  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
  }
}

void BoxDetector::detectBoxes(
    const cv::Mat& image,
    const cv::Mat& depth,
    const cv::Mat& mask,
    const std::string& color,
    const std_msgs::msg::Header& header)
{
  // (Unused now for publishing; kept intact for compatibility or future multi-publish use)
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  RCLCPP_DEBUG(get_logger(), "[%s] Found %zu contours", color.c_str(), contours.size());

  for (const auto& cnt : contours)
  {
    double area = cv::contourArea(cnt);
    if (area < min_area_) {
      RCLCPP_DEBUG(get_logger(), "[%s] Contour skipped (area=%.1f < %.1f)", color.c_str(), area, min_area_);
      continue;
    }

    // kept for visualization/backward compatibility only
    cv::RotatedRect rect = cv::minAreaRect(cnt);
    cv::Point2f verts[4];
    rect.points(verts);
    for (int i = 0; i < 4; i++) {
      cv::line(const_cast<cv::Mat&>(image), verts[i], verts[(i+1)%4], cv::Scalar(255, 0, 255), 1);
    }
  }
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
