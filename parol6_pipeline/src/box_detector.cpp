// file: box_detector_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

#include <atomic>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <vector>
#include <iomanip>
#include <sstream>
#include <cmath>

class BoxDetector : public rclcpp::Node
{
public:
  BoxDetector()
  : Node("box_detector_node"),
    fx_(declare_parameter("fx", 554.3827)),
    fy_(declare_parameter("fy", 554.3827)),
    cx_(declare_parameter("cx", 320.5)),
    cy_(declare_parameter("cy", 240.5)),
    min_area_px_(declare_parameter("min_area_px", 300.0)),
    square_tolerance_(declare_parameter("square_tolerance", 0.12)),
    up_normal_cos_thresh_(declare_parameter("up_normal_cos_thresh", 0.90)),
    plane_inlier_thresh_(declare_parameter("plane_inlier_thresh", 0.006)),
    ransac_iters_(declare_parameter("ransac_iters", 120)),
    max_sample_points_(declare_parameter("max_sample_points", 4000)),
    camera_frame_(declare_parameter("camera_frame", "realsense_rgb_frame")),
    base_frame_(declare_parameter("base_frame", "base_link")),
    detection_active_(false)
  {
    info_pub_ = create_publisher<std_msgs::msg::String>("/detected_box_info", 10);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/detected_box_pose", 10);

    rgb_sub_   = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/camera/image_raw");
    depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/camera/depth/image_raw");

    tf_buffer_      = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *rgb_sub_, *depth_sub_);

    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "/start_detection",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>&,
             std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
        std::lock_guard<std::mutex> lk(mutex_);
        if (detection_active_) {
          resp->success = false;
          resp->message = "Detection already running.";
          return;
        }
        // --- per-cycle reset ---
        has_published_ = false;                            // <<< CHANGED: reset the "publish once" gate
        cycle_id_++;                                       // <<< CHANGED: for logging
        detection_active_ = true;

        // (Re)register callback for this cycle
        sync_->registerCallback(std::bind(&BoxDetector::imageCb, this, std::placeholders::_1, std::placeholders::_2));

        resp->success = true; resp->message = "Detection started.";
        RCLCPP_INFO(get_logger(), "Detection started (cycle %lu). State reset: has_published_=false", static_cast<unsigned long>(cycle_id_));
      });

    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "/stop_detection",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>&,
             std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
        std::lock_guard<std::mutex> lk(mutex_);
        if (!detection_active_) { resp->success = false; resp->message = "No active detection."; return; }
        detection_active_ = false;

        // Recreate synchronizer to drop any queued pairs from previous cycle
        sync_.reset();
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *rgb_sub_, *depth_sub_);

        resp->success = true; resp->message = "Detection stopped.";
        RCLCPP_INFO(get_logger(), "Detection stopped (cycle %lu).", static_cast<unsigned long>(cycle_id_));
      });

    RCLCPP_INFO(get_logger(), "BoxDetector ready. Call /start_detection.");
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

  struct PlaneModel {
    cv::Vec3f n;   // unit normal
    float d;       // plane: n·x + d = 0
  };

  static bool planeFrom3(const cv::Vec3f& a, const cv::Vec3f& b, const cv::Vec3f& c, PlaneModel& out)
  {
    cv::Vec3f n = (b - a).cross(c - a);
    float nn = std::sqrt(n.dot(n));
    if (nn < 1e-6f) return false;
    n = n * (1.0f / nn);
    float d = -n.dot(a);
    out.n = n; out.d = d;
    return true;
  }

  static inline float pointPlaneDist(const cv::Vec3f& p, const PlaneModel& pl)
  { return std::fabs(pl.n.dot(p) + pl.d); }

  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
               const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
  {
    if (!detection_active_.load()) return;

    cv::Mat rgb, depth32;
    try {
      rgb     = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8)->image;
      auto d  = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
      depth32 = d->image;
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
      return;
    }

    cv::Mat hsv; cv::cvtColor(rgb, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask = createBoxMask(hsv);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::optional<geometry_msgs::msg::PoseStamped> best_pose;
    std::optional<std::string> best_info_str;
    double best_dist = 1e9;

    geometry_msgs::msg::TransformStamped T_cb;
    try {
      T_cb = tf_buffer_->lookupTransform(base_frame_, camera_frame_, rclcpp::Time(0), tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF lookup failed: %s", ex.what());
      return;
    }
    tf2::Transform tf_cb; tf2::fromMsg(T_cb.transform, tf_cb);

    static thread_local std::mt19937 rng{std::random_device{}()};

    for (const auto& cnt : contours) {
      double area_px = cv::contourArea(cnt);
      if (area_px < min_area_px_) continue;

      cv::Rect bb = cv::boundingRect(cnt);
      std::vector<cv::Point> pxs;
      for (int y = bb.y; y < bb.y + bb.height; ++y) {
        const uint8_t* mrow = mask.ptr<uint8_t>(y);
        for (int x = bb.x; x < bb.x + bb.width; ++x) {
          if (mrow[x]) pxs.emplace_back(x, y);
        }
      }
      if (pxs.size() < 400) continue;

      std::vector<cv::Vec3f> pts_base;
      std::vector<cv::Point> pxs_valid;
      pts_base.reserve(pxs.size());
      pxs_valid.reserve(pxs.size());
      for (const auto& p : pxs) {
        float z = depth32.at<float>(p.y, p.x);
        if (!std::isfinite(z) || z < 0.2f || z > 5.0f) continue;
        float X = (p.x - cx_) * z / fx_;
        float Y = (p.y - cy_) * z / fy_;
        tf2::Vector3 vb = tf_cb * tf2::Vector3(X, Y, z);
        pts_base.emplace_back(static_cast<float>(vb.x()), static_cast<float>(vb.y()), static_cast<float>(vb.z()));
        pxs_valid.emplace_back(p);
      }
      if (pts_base.size() < 400) continue;

      PlaneModel best_plane;
      size_t best_inliers = 0;
      std::vector<int> best_idxs;
      std::uniform_int_distribution<int> uni(0, static_cast<int>(pts_base.size()) - 1);

      for (int it = 0; it < ransac_iters_; ++it) {
        int i1 = uni(rng), i2 = uni(rng), i3 = uni(rng);
        if (i1 == i2 || i1 == i3 || i2 == i3) { --it; continue; }

        PlaneModel pl;
        if (!planeFrom3(pts_base[i1], pts_base[i2], pts_base[i3], pl)) continue;

        cv::Vec3f z_axis(0.f, 0.f, 1.f);
        float cos_up = std::fabs(pl.n.dot(z_axis));
        if (cos_up < static_cast<float>(up_normal_cos_thresh_)) continue;

        std::vector<int> inliers;
        inliers.reserve(pts_base.size());
        for (int idx = 0; idx < static_cast<int>(pts_base.size()); ++idx) {
          if (pointPlaneDist(pts_base[idx], pl) <= static_cast<float>(plane_inlier_thresh_)) {
            inliers.push_back(idx);
          }
        }
        if (inliers.size() > best_inliers) {
          best_inliers = inliers.size();
          best_plane = pl;
          best_idxs = inliers;
        }
      }

      if (best_inliers < 200) continue;

      std::vector<cv::Vec3f> inliers3;
      inliers3.reserve(best_idxs.size());
      for (int idx : best_idxs) inliers3.push_back(pts_base[idx]);

      cv::Mat data(static_cast<int>(inliers3.size()), 3, CV_32F);
      for (size_t i = 0; i < inliers3.size(); ++i) {
        data.at<float>(static_cast<int>(i), 0) = inliers3[i][0];
        data.at<float>(static_cast<int>(i), 1) = inliers3[i][1];
        data.at<float>(static_cast<int>(i), 2) = inliers3[i][2];
      }
      cv::PCA pca(data, cv::Mat(), cv::PCA::DATA_AS_ROW);

      cv::Vec3f e2 = pca.eigenvectors.row(2);
      auto nrm = [](cv::Vec3f v) {
        float n = std::sqrt(v.dot(v));
        return (n > 1e-9f) ? v * (1.f / n) : v;
      };
      e2 = nrm(e2);
      if (e2[2] > 0) e2 = -e2;

      cv::Vec3f e0p = nrm(pca.eigenvectors.row(0));
      e0p = nrm(e0p - e2 * (e0p.dot(e2)));
      cv::Vec3f e1p = nrm(e2.cross(e0p));

      cv::Vec3f C = pca.mean;
      std::vector<cv::Point2f> uv;
      uv.reserve(inliers3.size());
      for (const auto& P : inliers3) {
        cv::Vec3f d = P - C;
        uv.emplace_back(d.dot(e0p), d.dot(e1p));
      }

      std::vector<int> hull_idx;
      cv::convexHull(uv, hull_idx, true, false);
      std::vector<cv::Point2f> hull;
      hull.reserve(hull_idx.size());
      for (int i : hull_idx) hull.push_back(uv[i]);

      if (hull.size() < 4) continue;

      double peri = cv::arcLength(hull, true);
      std::vector<cv::Point2f> poly;
      cv::approxPolyDP(hull, poly, 0.02 * peri, true);
      if (poly.size() != 4) continue;

      auto angle_ok = [](const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c) {
        cv::Point2f v1 = a - b, v2 = c - b;
        double n1 = std::hypot(v1.x, v1.y), n2 = std::hypot(v2.x, v2.y);
        if (n1 < 1e-6 || n2 < 1e-6) return false;
        double cosang = std::fabs(v1.x * v2.x + v1.y * v2.y) / (n1 * n2);
        return cosang < 0.20;
      };
      bool right = angle_ok(poly[3], poly[0], poly[1]) &&
                   angle_ok(poly[0], poly[1], poly[2]) &&
                   angle_ok(poly[1], poly[2], poly[3]) &&
                   angle_ok(poly[2], poly[3], poly[0]);
      if (!right) continue;

      auto seglen = [](const cv::Point2f& a, const cv::Point2f& b) {
        return std::hypot(a.x - b.x, a.y - b.y);
      };
      double s0 = seglen(poly[0], poly[1]);
      double s1 = seglen(poly[1], poly[2]);
      double s2 = seglen(poly[2], poly[3]);
      double s3 = seglen(poly[3], poly[0]);
      double L = 0.5 * (s0 + s2);
      double W = 0.5 * (s1 + s3);
      double max_side = std::max(L, W);
      if (max_side < 1e-6) continue;
      if (std::fabs(L - W) / max_side > square_tolerance_) continue;

      // Estimate height
      std::vector<float> z_vals;
      z_vals.reserve(pxs_valid.size());
      for (const auto& p : pxs_valid) {
        float z = depth32.at<float>(p.y, p.x);
        if (std::isfinite(z)) z_vals.push_back(z);
      }
      std::sort(z_vals.begin(), z_vals.end());
      float height = 0.0f;
      if (z_vals.size() > 10) {
        float z_top = C[2];
        float z_bottom = z_vals.front(); // raw depth
        tf2::Vector3 z_bottom_vec(0, 0, z_bottom);
        float z_world = (tf_cb * z_bottom_vec).z();
        height = std::max(0.0f, z_world - z_top);
      }

      // Align X to longer side
      cv::Point2f e_uv;
      if (L >= W) {
        e_uv = poly[1] - poly[0];  // use longer edge
      } else {
        e_uv = poly[2] - poly[1];  // or switch
      }

      double e_norm = std::hypot(e_uv.x, e_uv.y);
      if (e_norm < 1e-9) continue;
      e_uv.x /= e_norm; e_uv.y /= e_norm;

      cv::Vec3f e0 = nrm(e0p * static_cast<float>(e_uv.x) + e1p * static_cast<float>(e_uv.y));
      cv::Vec3f e1 = nrm(e2.cross(e0));

      // Optional: flip to avoid negative X/Y
      tf2::Vector3 e0_tf(e0[0], e0[1], e0[2]);
      if (e0_tf.x() < -0.1) {  // strongly negative X
        e0 = -e0; e1 = -e1;
      } else if (std::abs(e0_tf.x()) < 0.2 && e0_tf.y() < -0.1) {  // weak X, negative Y
        e0 = -e0; e1 = -e1;
      }

      tf2::Matrix3x3 R(
        e0[0], e1[0], e2[0],
        e0[1], e1[1], e2[1],
        e0[2], e1[2], e2[2]
      );
      if (R.determinant() < 0) {
        e1 = -e1;
        R = tf2::Matrix3x3(e0[0], e1[0], e2[0],
                           e0[1], e1[1], e2[1],
                           e0[2], e1[2], e2[2]);
      }

      tf2::Quaternion q; R.getRotation(q); q.normalize();

      geometry_msgs::msg::PoseStamped pose;
      pose.header = rgb_msg->header;
      pose.header.frame_id = base_frame_;
      pose.pose.position.x = C[0];
      pose.pose.position.y = C[1];
      pose.pose.position.z = C[2];
      pose.pose.orientation = tf2::toMsg(q);

      double roll, pitch, yaw;
      tf2::Matrix3x3(tf2::Quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
                                     pose.pose.orientation.z, pose.pose.orientation.w))
        .getRPY(roll, pitch, yaw);

      std::string color = "unknown";
      cv::Rect roi = cv::boundingRect(cnt);
      cv::Scalar mean_hsv = cv::mean(hsv(roi), mask(roi));
      if (mean_hsv[0] < 20 || mean_hsv[0] > 170) {
        color = "red";
      } else if (mean_hsv[0] > 90 && mean_hsv[0] < 135) {
        color = "blue";
      }

      std::ostringstream oss;
      oss << std::fixed << std::setprecision(3)
          << "Box Detected:\n"
          << "  Color: " << color << "\n"
          << "  Size (L x W x H): " << L << "m x " << W << "m x " << height << "m\n"
          << "  Position (X, Y, Z): " << C[0] << ", " << C[1] << ", " << C[2] << " [m]\n"
          << "  Orientation (R, P, Y): "
          << roll * 180 / M_PI << ", "
          << pitch * 180 / M_PI << ", "
          << yaw * 180 / M_PI << " [deg]\n"
          << "  Area (pixels): " << area_px << "\n"
          << "  Distance: " << std::sqrt(C[0]*C[0] + C[1]*C[1] + C[2]*C[2]) << " [m]\n"
          << "  Timestamp: " << rgb_msg->header.stamp.sec << "."
          << std::setfill('0') << std::setw(9) << rgb_msg->header.stamp.nanosec;

      double dist = std::sqrt(C[0]*C[0] + C[1]*C[1] + C[2]*C[2]);
      if (dist < best_dist) {
        best_dist = dist;
        best_pose = pose;
        best_info_str = oss.str();
      }

      cv::polylines(rgb, {cnt}, true, cv::Scalar(0, 255, 0), 2);
    }

    if (best_pose && best_info_str) {
      if (!has_published_.load()) {
        pose_pub_->publish(*best_pose);
        auto info_msg = std::make_shared<std_msgs::msg::String>();
        info_msg->data = *best_info_str;
        info_pub_->publish(*info_msg);
        RCLCPP_INFO_STREAM(get_logger(), "\n" << *best_info_str);

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header = best_pose->header;
        tf_msg.child_frame_id = "detected_box";
        tf_msg.transform.translation.x = best_pose->pose.position.x;
        tf_msg.transform.translation.y = best_pose->pose.position.y;
        tf_msg.transform.translation.z = best_pose->pose.position.z;
        tf_msg.transform.rotation = best_pose->pose.orientation;
        tf_broadcaster_->sendTransform(tf_msg);

        has_published_ = true;
        RCLCPP_INFO(get_logger(), "Published detection once (cycle %lu). Waiting for box to disappear or for next cycle.",
                    static_cast<unsigned long>(cycle_id_));
      }
    } else {
      if (has_published_.load()) {
        RCLCPP_INFO(get_logger(), "Box no longer visible (cycle %lu). Ready to publish again within this cycle.",
                    static_cast<unsigned long>(cycle_id_));
        has_published_ = false;
      }
      RCLCPP_DEBUG(get_logger(), "No valid square top surface found.");
    }

    // Viewers
    cv::imshow("Detection", rgb);
    if (!depth32.empty()) {
      cv::Mat depth_vis;
      depth32.convertTo(depth_vis, CV_8UC1, 255.0 / 5.0);
      cv::applyColorMap(depth_vis, depth_vis, cv::COLORMAP_JET);
      cv::imshow("Depth View", depth_vis);
    }
    cv::waitKey(1);
  }

  cv::Mat createBoxMask(const cv::Mat& hsv) {
    cv::Mat red1, red2, blue, out;
    cv::inRange(hsv, cv::Scalar(0,70,50),   cv::Scalar(10,255,255), red1);
    cv::inRange(hsv, cv::Scalar(170,70,50), cv::Scalar(180,255,255), red2);
    cv::inRange(hsv, cv::Scalar(90,50,50),  cv::Scalar(135,255,255), blue);
    cv::bitwise_or(red1, red2, out);
    cv::bitwise_or(out, blue, out);
    return out;
  }

  double fx_, fy_, cx_, cy_;
  double min_area_px_;
  double square_tolerance_;
  double up_normal_cos_thresh_;
  double plane_inlier_thresh_;
  int ransac_iters_;
  int max_sample_points_;
  std::string camera_frame_;
  std::string base_frame_;

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rgb_sub_, depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr info_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_, stop_srv_;

  std::atomic<bool> detection_active_;
  std::atomic<bool> has_published_{false};
  std::mutex mutex_;
  uint64_t cycle_id_ = 0;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BoxDetector>());
  rclcpp::shutdown();
  return 0;
}
