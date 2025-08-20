#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <conveyorbelt_msgs/srv/move_distance.hpp>
#include <msg_gazebo/srv/attach_detach.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <nlohmann/json.hpp>
#include <fstream>
#include <mutex>
#include <atomic>
#include <optional>
#include <chrono>
#include <unordered_map>
#include <cctype>
#include <cmath>
#include <algorithm>  // std::clamp

using nlohmann::json;
using namespace std::chrono_literals;

// -------- helper --------
static inline double pos_distance(const geometry_msgs::msg::Pose &a,
                                  const geometry_msgs::msg::Pose &b) {
  const double dx = a.position.x - b.position.x;
  const double dy = a.position.y - b.position.y;
  const double dz = a.position.z - b.position.z;
  return std::sqrt(dx*dx + dy*dy + dz*dz);
}

// -------- TargetDB --------
class TargetDB {
public:
  explicit TargetDB(rclcpp::Logger logger) : logger_(logger) {}

  bool load(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
      RCLCPP_ERROR(logger_, "Targets.json not found: %s", path.c_str());
      return false;
    }
    try {
      f >> data_;
      loaded_ = true;
      for (auto it = data_.begin(); it != data_.end(); ++it)
        RCLCPP_INFO(logger_, " Target key: %s", it.key().c_str());
      if (!hasJointTarget("up"))
        RCLCPP_WARN(logger_, "Target 'up' missing (recommended).");
      return true;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(logger_, "Failed to parse Targets.json: %s", e.what());
      return false;
    }
  }

  bool loaded() const { return loaded_; }

  bool getJointTarget(const std::string& name, std::vector<double>& joints_out) const {
    joints_out.clear();
    const json* j = nullptr;
    if (data_.contains("joints") && data_["joints"].is_object()) {
      const auto& jointsObj = data_["joints"];
      auto it = jointsObj.find(name);
      if (it != jointsObj.end() && it->is_array()) j = &(*it);
    }
    if (!j) {
      auto itFlat = data_.find(name);
      if (itFlat != data_.end() && itFlat->is_array()) j = &(*itFlat);
    }
    if (!j) return false;
    if (j->size() != 6) {
      RCLCPP_ERROR(logger_, "Joint target '%s' has %zu values; expected 6", name.c_str(), j->size());
      return false;
    }
    try {
      joints_out.reserve(6);
      for (const auto& v : *j) joints_out.push_back(v.get<double>());
      return true;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(logger_, "Parse joints '%s' failed: %s", name.c_str(), e.what());
      joints_out.clear();
      return false;
    }
  }

  bool hasJointTarget(const std::string& name) const {
    std::vector<double> dummy;
    return getJointTarget(name, dummy);
  }

private:
  rclcpp::Logger logger_;
  json data_;
  bool loaded_{false};
};

class MotionController {
public:
  MotionController(rclcpp::Node* node, const std::string& planning_group)
  : node_(node), logger_(node->get_logger()), planning_group_(planning_group) {}

  void requestStop() {
    stop_flag_.store(true);
    std::lock_guard<std::mutex> lk(mgi_mutex_);
    if (mgi_) {
      RCLCPP_WARN(logger_, "MotionController: stop() requested");
      try { mgi_->stop(); } catch (...) {}
    }
  }
  void clearStop() { stop_flag_.store(false); }

  void setSpeedScale(double s) {
    const double clamped = std::clamp(s, 0.1, 1.0);
    speed_scale_.store(clamped);
    std::lock_guard<std::mutex> lk(mgi_mutex_);
    if (mgi_) {
      mgi_->setMaxVelocityScalingFactor(clamped);
      mgi_->setMaxAccelerationScalingFactor(clamped);
    }
    RCLCPP_INFO(logger_, "Motion speed scale set to %.2fx", clamped);
  }

  bool planAndExecuteJoints(const std::vector<double>& joints, const std::string& label) {
    if (stop_flag_.load()) return false;
    auto mgi = ensureMoveGroup();
    if (!mgi) return false;
    try {
      const double s = speed_scale_.load();
      mgi->setMaxVelocityScalingFactor(s);
      mgi->setMaxAccelerationScalingFactor(s);

      mgi->setJointValueTarget(joints);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto pc = mgi->plan(plan);
      if (pc != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger_, "[%s] planning failed", label.c_str());
        return false;
      }
      if (stop_flag_.load()) return false;
      auto ec = mgi->execute(plan);
      if (ec != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger_, "[%s] execution failed", label.c_str());
        return false;
      }
      return true;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(logger_, "[%s] exception: %s", label.c_str(), e.what());
      return false;
    }
  }

  bool planAndExecutePose(const geometry_msgs::msg::Pose& pose, const std::string& label) {
    if (stop_flag_.load()) return false;
    auto mgi = ensureMoveGroup();
    if (!mgi) return false;
    try {
      const double s = speed_scale_.load();   
      mgi->setMaxVelocityScalingFactor(s);
      mgi->setMaxAccelerationScalingFactor(s);

      mgi->setPoseTarget(pose);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto pc = mgi->plan(plan);
      if (pc != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger_, "[%s] planning failed", label.c_str());
        return false;
      }
      if (stop_flag_.load()) return false;
      auto ec = mgi->execute(plan);
      if (ec != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger_, "[%s] execution failed", label.c_str());
        return false;
      }
      return true;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(logger_, "[%s] exception: %s", label.c_str(), e.what());
      return false;
    }
  }

private:
  moveit::planning_interface::MoveGroupInterface* ensureMoveGroup() {
    std::lock_guard<std::mutex> lk(mgi_mutex_);
    if (!mgi_) {
      mgi_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        node_->shared_from_this(), planning_group_);
      const double s = speed_scale_.load();
      mgi_->setMaxVelocityScalingFactor(s);
      mgi_->setMaxAccelerationScalingFactor(s);
      RCLCPP_INFO(logger_, "MoveGroupInterface created for group '%s' (speed %.2fx)",
                  planning_group_.c_str(), s);
    }
    return mgi_.get();
  }

  rclcpp::Node* node_;
  rclcpp::Logger logger_;
  std::string planning_group_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> mgi_;
  std::mutex mgi_mutex_;
  std::atomic<bool> stop_flag_{false};
  std::atomic<double> speed_scale_{1.0}; 
};


class Gripper {
public:
  explicit Gripper(rclcpp::Node* node)
  : node_(node), logger_(node->get_logger()) {
    attach_client_ = node_->create_client<msg_gazebo::srv::AttachDetach>("/AttachDetach");
    start_pick_client_ = node_->create_client<std_srvs::srv::Trigger>("/start_picking");
  }

  void requestStop() {}

  bool startPicking(int wait_sec=30) {
    return callTrigger(start_pick_client_, "/start_picking", wait_sec);
  }

  bool attach(const std::string& model_name) {
    if (!attach_client_->wait_for_service(5s)) {
      RCLCPP_ERROR(logger_, "Service /AttachDetach not available.");
      return false;
    }
    auto req = std::make_shared<msg_gazebo::srv::AttachDetach::Request>();
    req->model1 = "parol6";
    req->link1  = "L6";
    req->model2 = model_name;
    req->link2  = "link";
    req->attach = true;

    auto fut = attach_client_->async_send_request(req);
    auto rc  = rclcpp::spin_until_future_complete(
                  node_->get_node_base_interface(), fut, 5s);
    if (rc != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(logger_, "Attach call did not complete (timeout or error)");
      return false;
    }
    auto res = fut.get();
    if (!res->success) {
      RCLCPP_ERROR(logger_, "Attach failed for %s", model_name.c_str());
      return false;
    }
    RCLCPP_INFO(logger_, "Attached OK: %s", model_name.c_str());
    return true;
  }

  bool detach(const std::string& model_name) {
    if (!attach_client_->wait_for_service(5s)) {
      RCLCPP_ERROR(logger_, "Service /AttachDetach not available.");
      return false;
    }
    auto req = std::make_shared<msg_gazebo::srv::AttachDetach::Request>();
    req->model1 = "parol6";
    req->link1  = "L6";
    req->model2 = model_name;
    req->link2  = "link";
    req->attach = false;

    auto fut = attach_client_->async_send_request(req);
    auto rc  = rclcpp::spin_until_future_complete(
                  node_->get_node_base_interface(), fut, 5s);
    if (rc != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(logger_, "Detach call did not complete (timeout or error)");
      return false;
    }
    auto res = fut.get();
    if (!res->success) {
      RCLCPP_ERROR(logger_, "Detach failed for %s", model_name.c_str());
      return false;
    }
    RCLCPP_INFO(logger_, "Detached OK: %s", model_name.c_str());
    return true;
  }


private:
  bool callTrigger(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &cli,
                   const char *name, int wait_sec) {
    if (!cli->wait_for_service(std::chrono::seconds(wait_sec))) {
      RCLCPP_ERROR(logger_, "Service %s not available.", name);
      return false;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto fut = cli->async_send_request(req);
    auto rc = rclcpp::spin_until_future_complete(node_->get_node_base_interface(), fut);
    if (rc != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(logger_, "Call to %s failed.", name);
      return false;
    }
    auto res = fut.get();
    if (!res->success) {
      RCLCPP_WARN(logger_, "%s returned failure: %s", name, res->message.c_str());
      return false;
    }
    return true;
  }

  rclcpp::Node* node_;
  rclcpp::Logger logger_;
  rclcpp::Client<msg_gazebo::srv::AttachDetach>::SharedPtr attach_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_pick_client_;
};

class Conveyor {
public:
  explicit Conveyor(rclcpp::Node* node) : node_(node), logger_(node->get_logger()) {}

  
  bool move(const std::string &srv, double distance) {
    auto client = node_->create_client<conveyorbelt_msgs::srv::MoveDistance>(srv);
    if (!client->wait_for_service(5s)) {
      RCLCPP_ERROR(logger_, "Service %s not available.", srv.c_str());
      return false;
    }
    auto req = std::make_shared<conveyorbelt_msgs::srv::MoveDistance::Request>();
    req->distance = distance;
    auto fut = client->async_send_request(req);
    auto rc = rclcpp::spin_until_future_complete(node_->get_node_base_interface(), fut);
    if (rc != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(logger_, "Call to %s failed.", srv.c_str());
      return false;
    }
    auto res = fut.get();
    if (!res->success) {
      RCLCPP_ERROR(logger_, "Conveyor move %s reported failure.", srv.c_str());
      return false;
    }
    RCLCPP_INFO(logger_, "Moved %s by %.3f", srv.c_str(), distance);
    return true;
  }

private:
  rclcpp::Node* node_;
  rclcpp::Logger logger_;
};


class VisionManager {
public:
  explicit VisionManager(rclcpp::Node* node)
  : node_(node), logger_(node->get_logger()) {
    pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/detected_box_pose", rclcpp::QoS(20),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!vision_enabled_.load()) return;
        std::lock_guard<std::mutex> lk(m_);
        latest_pose_ = *msg;
        latest_stamp_ = latest_pose_.header.stamp;
        got_pose_ = true;
      });
    info_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/detected_box_info", rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (!vision_enabled_.load()) return;
        std::lock_guard<std::mutex> lk(m_);
        std::string s = msg->data;
        for (char &c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        latest_color_.clear();
        if (s.find("red") != std::string::npos) latest_color_ = "red";
        else if (s.find("blue") != std::string::npos) latest_color_ = "blue";
        got_color_ = !latest_color_.empty();
      });

    start_det_client_ = node_->create_client<std_srvs::srv::Trigger>("/start_detection");
    stop_det_client_  = node_->create_client<std_srvs::srv::Trigger>("/stop_detection");
    spawn_client_     = node_->create_client<std_srvs::srv::Trigger>("/spawn_box");
  }

  void requestStop() {
    stop_flag_.store(true);
    vision_enabled_.store(false);
    (void)callTrigger(stop_det_client_, "/stop_detection", 2);
  }
  void clearStop() { stop_flag_.store(false); }

  bool spawnBox() { return callTrigger(spawn_client_, "/spawn_box", 5); }

  std::optional<std::pair<geometry_msgs::msg::PoseStamped,std::string>>
  runFixed(double start_delay_sec, double vision_sec) {
    if (stop_flag_.load()) return std::nullopt;

    (void)callTrigger(stop_det_client_, "/stop_detection", 2);
    resetCache();

    RCLCPP_INFO(logger_, " Waiting %.1fs before starting fixed vision window...", start_delay_sec);
    if (!sleepWithAbort(start_delay_sec)) return std::nullopt;

    if (!callTrigger(start_det_client_, "/start_detection", 5)) return std::nullopt;
    vision_enabled_.store(true);

    RCLCPP_INFO(logger_, "Fixed vision running for %.1fs ...", vision_sec);
    const rclcpp::Time t_start = node_->now();
    while ((node_->now() - t_start).seconds() < vision_sec) {
      if (stop_flag_.load()) break;
      rclcpp::spin_some(node_->get_node_base_interface());
      std::this_thread::sleep_for(20ms);
    }

    (void)callTrigger(stop_det_client_, "/stop_detection", 3);
    vision_enabled_.store(false);

    std::lock_guard<std::mutex> lk(m_);
    if (!got_pose_)  { RCLCPP_ERROR(logger_, "No pose in fixed window.");  return std::nullopt; }
    if (!got_color_) { RCLCPP_ERROR(logger_, "No color in fixed window."); return std::nullopt; }
    return std::make_pair(latest_pose_, latest_color_);
  }

  std::optional<std::pair<geometry_msgs::msg::PoseStamped,std::string>>
  runWindow(double warmup_sec, double window_sec, double stable_tol_m, int stable_count_needed, double color_grace_sec) {
    if (stop_flag_.load()) return std::nullopt;

    resetCache();
    if (!callTrigger(start_det_client_, "/start_detection", 5)) return std::nullopt;

    vision_enabled_.store(true);
    const rclcpp::Time t_start  = node_->now();
    const rclcpp::Time t_accept = t_start + rclcpp::Duration::from_seconds(warmup_sec);

    RCLCPP_INFO(logger_, " Vision started for %.1fs (warm-up %.2fs)...", window_sec, warmup_sec);

    bool have_prev=false;
    geometry_msgs::msg::Pose prev_pose{};
    int stable_count=0;

    std::optional<geometry_msgs::msg::PoseStamped> stable_pose;
    rclcpp::Time t_stable;

    while ((node_->now() - t_start).seconds() < window_sec) {
      if (stop_flag_.load()) break;
      rclcpp::spin_some(node_->get_node_base_interface());
      std::this_thread::sleep_for(50ms);

      geometry_msgs::msg::PoseStamped cur_pose_msg;
      std::string cur_color;
      bool have_pose=false, have_color=false;

      {
        std::lock_guard<std::mutex> lk(m_);
        have_pose  = got_pose_;
        have_color = got_color_;
        if (have_pose)  cur_pose_msg = latest_pose_;
        if (have_color) cur_color    = latest_color_;
      }

      const rclcpp::Time t_now = node_->now();
      if (t_now < t_accept) continue;

      if (!stable_pose && have_pose) {
        if (!have_prev) {
          prev_pose = cur_pose_msg.pose; have_prev = true; stable_count = 1;
        } else {
          const double dp = pos_distance(cur_pose_msg.pose, prev_pose);
          if (dp <= stable_tol_m) stable_count++; else { prev_pose = cur_pose_msg.pose; stable_count = 1; }
        }
        if (stable_count >= stable_count_needed) {
          stable_pose = cur_pose_msg;
          t_stable = t_now;
          RCLCPP_INFO(logger_, " Pose stabilized (count=%d, tol=%.3fm). Waiting for color...", stable_count, stable_tol_m);
        }
      }

      if (stable_pose) {
        if (have_color) {
          (void)callTrigger(stop_det_client_, "/stop_detection", 3);
          vision_enabled_.store(false);
          RCLCPP_INFO(logger_, " Vision stopped (stable pose + color).");
          return std::make_pair(*stable_pose, cur_color);
        }
        if ((t_now - t_stable).seconds() >= color_grace_sec) {
          RCLCPP_WARN(logger_, "Color not received within %.1fs after stable pose.", color_grace_sec);
          break;
        }
      }
    }

    (void)callTrigger(stop_det_client_, "/stop_detection", 3);
    vision_enabled_.store(false);
    RCLCPP_WARN(logger_, " Vision stopped (timeout).");
    return std::nullopt;
  }

private:
  void resetCache() {
    std::lock_guard<std::mutex> lk(m_);
    got_pose_ = false;
    got_color_ = false;
    latest_color_.clear();
    latest_pose_ = geometry_msgs::msg::PoseStamped();
    latest_stamp_ = builtin_interfaces::msg::Time();
  }

  bool sleepWithAbort(double seconds) {
    const int ms = static_cast<int>(seconds * 1000.0);
    int left = ms;
    while (left > 0) {
      if (stop_flag_.load()) return false;
      const int step = std::min(50, left);
      std::this_thread::sleep_for(std::chrono::milliseconds(step));
      left -= step;
    }
    return true;
  }

  bool callTrigger(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &cli,
                   const char *name, int wait_sec) {
    if (!cli->wait_for_service(std::chrono::seconds(wait_sec))) {
      RCLCPP_ERROR(logger_, "Service %s not available.", name);
      return false;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto fut = cli->async_send_request(req);
    auto rc = rclcpp::spin_until_future_complete(node_->get_node_base_interface(), fut);
    if (rc != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(logger_, "Call to %s failed.", name);
      return false;
    }
    auto res = fut.get();
    if (!res->success) {
      RCLCPP_WARN(logger_, "%s returned failure: %s", name, res->message.c_str());
      return false;
    }
    return true;
  }

  rclcpp::Node* node_;
  rclcpp::Logger logger_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr info_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_det_client_, stop_det_client_, spawn_client_;

  std::mutex m_;
  geometry_msgs::msg::PoseStamped latest_pose_;
  builtin_interfaces::msg::Time   latest_stamp_;
  std::string latest_color_;
  bool got_pose_{false};
  bool got_color_{false};

  std::atomic<bool> vision_enabled_{false};
  std::atomic<bool> stop_flag_{false};
};
