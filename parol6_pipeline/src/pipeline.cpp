#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <conveyorbelt_msgs/srv/move_distance.hpp>
#include <msg_gazebo/srv/attach_detach.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>

#include <fstream>
#include <string>
#include <thread>
#include <mutex>
#include <unordered_map>
#include <optional>
#include <chrono>
#include <atomic>
#include <cctype>
#include <cmath>

using nlohmann::json;
using namespace std::chrono_literals;

// (kept for potential debug, not used for picking now)
static double pos_distance(const geometry_msgs::msg::Pose &a,
                           const geometry_msgs::msg::Pose &b)
{
  const double dx = a.position.x - b.position.x;
  const double dy = a.position.y - b.position.y;
  const double dz = a.position.z - b.position.z;
  return std::sqrt(dx*dx + dy*dy + dz*dz);
}

class PipelineNode : public rclcpp::Node {
public:
  PipelineNode() : rclcpp::Node("pipeline_node") {
    // ---------- Subscribers (gated by vision_enabled_) ----------
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/detected_box_pose", rclcpp::QoS(20),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!vision_enabled_.load()) return;
        std::lock_guard<std::mutex> lk(vision_mutex_);
        latest_pose_       = *msg;
        latest_pose_stamp_ = latest_pose_.header.stamp;
        got_pose_ = true;
      });

    info_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/detected_box_info", rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (!vision_enabled_.load()) return;
        std::lock_guard<std::mutex> lk(vision_mutex_);
        std::string s = msg->data;
        for (char &c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        latest_color_.clear();
        if (s.find("red")  != std::string::npos) latest_color_ = "red";
        else if (s.find("blue") != std::string::npos) latest_color_ = "blue";
        got_color_ = !latest_color_.empty();
      });

    // ---------- Service clients ----------
    attach_client_     = this->create_client<msg_gazebo::srv::AttachDetach>("/AttachDetach");
    start_det_client_  = this->create_client<std_srvs::srv::Trigger>("/start_detection");
    stop_det_client_   = this->create_client<std_srvs::srv::Trigger>("/stop_detection");
    spawn_client_      = this->create_client<std_srvs::srv::Trigger>("/spawn_box");
    start_pick_client_ = this->create_client<std_srvs::srv::Trigger>("/start_picking"); // external pick controller
  }

  void run() {
    RCLCPP_INFO(get_logger(), "Pipeline node ready. Press Enter to begin...");
    std::cin.get();

    loadTargetsJson();            // optional fallback for named poses
    stopDetectionIfRunning();     // make sure detection is off
    vision_enabled_.store(false); // gate callbacks during initial feed

    // ===== Initial feed: (spawn -> 1s -> conveyor2 move) ×3; 3s between first two =====
    if (!doInitialFeed(3, 3s)) {
      RCLCPP_ERROR(get_logger(), "Initial feed failed.");
      return;
    }

    // ==== NEW: Fixed-timing vision right after the 3rd conveyor2 move ====
    // Wait 4s, then run vision for 3s, stop, and use that result.
    auto first_vd = runFixedVisionWindow(6.0 /*start delay sec*/, 3.0 /*vision sec*/);
    if (!first_vd) {
      RCLCPP_ERROR(get_logger(), "No detection in fixed vision window. Stopping.");
      return;
    }

    int cycle = 0;

    // Process the first detection immediately
    {
      auto [det_pose, det_color] = *first_vd;
      if (!processPickDropCycle(det_pose, det_color)) {
        RCLCPP_ERROR(get_logger(), "Cycle %d failed.", cycle + 1);
        return;
      }
      ++cycle;
    }

    // Subsequent cycles: keep existing stabilization-based vision window
    while (rclcpp::ok() && (kMaxCycles < 0 || cycle < kMaxCycles)) {
      RCLCPP_INFO(get_logger(), "==== Cycle %d ====", cycle + 1);

      auto vd = runVisionWindow(kVisionWindowSec);
      if (!vd) {
        RCLCPP_ERROR(get_logger(), "No detection in vision window. Stopping.");
        break;
      }
      auto [det_pose, det_color] = *vd;
      if (!processPickDropCycle(det_pose, det_color)) {
        RCLCPP_ERROR(get_logger(), "Cycle %d failed.", cycle + 1);
        break;
      }
      ++cycle;
    }

    RCLCPP_INFO(get_logger(), "✅ Pipeline finished.");
  }

private:
  // ---------- Constants (no ROS params) ----------
  static constexpr int    kMaxCycles          = -1;     // -1 => infinite
  static constexpr double kVisionWindowSec    = 15.0;   // seconds (legacy stabilized vision)
  static constexpr double kVisionWarmupSec    = 5.0;    // seconds (legacy stabilized vision)
  static constexpr double kStablePosTol       = 0.01;   // meters (1 cm)
  static constexpr int    kStableMinCount     = 10;     // consecutive samples
  static constexpr bool   kEnforceMsgStamp    = false;  // ignore message stamp gate by default
  static constexpr double kColorGraceSec      = 12.0;   // seconds to wait for color after pose stable

  static constexpr double kConveyor2Step      = 0.24;   // meters
  static constexpr double kConveyor1Step      = 0.215;
  static constexpr double kConveyor3Step      = 0.215;

  // motion scaling
  static constexpr double kVelScale           = 0.2;
  static constexpr double kAccScale           = 0.2;

  // ---------- State ----------
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Client<msg_gazebo::srv::AttachDetach>::SharedPtr attach_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_det_client_, stop_det_client_, spawn_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_pick_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr info_sub_;

  std::mutex vision_mutex_;
  geometry_msgs::msg::PoseStamped latest_pose_;
  builtin_interfaces::msg::Time   latest_pose_stamp_;
  std::string latest_color_;
  bool got_pose_{false};
  bool got_color_{false};
  std::atomic<bool> vision_enabled_{false};

  std::unordered_map<std::string, int> color_counts_{{"red",0},{"blue",0}};

  json targets_json_;
  bool targets_json_loaded_{false};

  // ---------- Core cycle ----------
  bool processPickDropCycle(const geometry_msgs::msg::PoseStamped& det_pose_in, const std::string& det_color) {
    RCLCPP_INFO(get_logger(), "==== Executing cycle for color: %s ====", det_color.c_str());

    // 1) Let your external node do the picking
    if (!callTrigger(start_pick_client_, "/start_picking", 30)) {
      RCLCPP_ERROR(get_logger(), "External picking failed or service unavailable.");
      return false;
    }
    RCLCPP_INFO(get_logger(), "External picking completed. Proceeding to place.");

    // 2) Background FEED while we move to home
    std::thread bg_feed([this]() {
      if (!spawnBox()) return;
      std::this_thread::sleep_for(1s);
      moveConveyor("/conveyor2/MoveDistance", kConveyor2Step);
    });

    // 3) Transit to home (ensure MoveIt is ready)
    if (!goNamed("home")) { if (bg_feed.joinable()) bg_feed.join(); return false; }
    if (bg_feed.joinable()) bg_feed.join();

    // 4) Route by color to drop location
    if (det_color == "red") {
      if (!goNamed("conveyor1_pre")) return false;
      if (!goNamed("conveyor_1")) return false;
    } else {
      if (!goNamed("conveyor3_pre")) return false;
      if (!goNamed("conveyor_3")) return false;
    }

    // 5) Detach at drop, then nudge that conveyor
    std::this_thread::sleep_for(200ms);
    if (!attachDetach("ignored_model_name", false)) {
      RCLCPP_WARN(get_logger(), "Detach reported failure (maybe already detached). Continuing.");
    }
    if (det_color == "red") {
      moveConveyor("/conveyor1/MoveDistance", kConveyor1Step);
    } else {
      moveConveyor("/conveyor3/MoveDistance", kConveyor3Step);
    }

    // 6) Return along the same side, back to home/init_pose
    if (det_color == "red") {
      if (!goNamed("conveyor_1")) return false;
      if (!goNamed("conveyor1_pre")) return false;
    } else {
      if (!goNamed("conveyor_3")) return false;
      if (!goNamed("conveyor3_pre")) return false;
    }
    if (!goNamed("home")) return false;
    if (!goNamedOrJson("init_pose")) {
      RCLCPP_WARN(get_logger(), "No init_pose named/JSON target; staying at home.");
    }

    // 7) Tail feed before next vision
    if (!spawnBox()) { RCLCPP_WARN(get_logger(), "Spawn failed in tail feed."); }
    std::this_thread::sleep_for(1s);
    moveConveyor("/conveyor2/MoveDistance", kConveyor2Step);

    return true;
  }

  // ---------- MoveIt bringup wait + construct lazily (robust) ----------
  bool ensureMoveGroup(double timeout_sec = 30.0) {
    if (move_group_) return true;

    const rclcpp::Time t0 = now();
    while (rclcpp::ok() && (now() - t0).seconds() < timeout_sec) {
      try {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
        move_group_->setPlanningTime(5.0);
        move_group_->setMaxVelocityScalingFactor(kVelScale);
        move_group_->setMaxAccelerationScalingFactor(kAccScale);
        auto named = move_group_->getNamedTargets();
        RCLCPP_INFO(get_logger(), "MoveGroup ready. Named targets: %zu", named.size());
        return true;
      } catch (const std::exception &e) {
        RCLCPP_WARN(get_logger(), "Waiting for MoveIt... (%s)", e.what());
        std::this_thread::sleep_for(500ms);
      }
    }

    RCLCPP_ERROR(get_logger(), "Timed out (%.1fs) waiting for MoveIt (move_group). Start your MoveIt bringup.",
                 timeout_sec);
    return false;
  }

  // ---------- NEW: Fixed-timing vision (no stabilization): wait -> run -> stop ----------
  // After the 3rd spawn+conveyor2 move: wait `start_delay_sec`, run vision for `vision_sec`, stop.
  // Returns last pose+color received during that window (if both arrived).
  std::optional<std::pair<geometry_msgs::msg::PoseStamped,std::string>>
  runFixedVisionWindow(double start_delay_sec, double vision_sec) {
    // Make sure detection is off and reset buffers
    (void)callTrigger(stop_det_client_, "/stop_detection", 2);
    {
      std::lock_guard<std::mutex> lk(vision_mutex_);
      got_pose_ = false;
      got_color_ = false;
      latest_color_.clear();
      latest_pose_ = geometry_msgs::msg::PoseStamped{};
      latest_pose_stamp_ = builtin_interfaces::msg::Time{};
    }

    // Start delay
    RCLCPP_INFO(get_logger(), "⏳ Waiting %.1fs before starting fixed vision window...", start_delay_sec);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(start_delay_sec * 1000)));

    // Start detection
    if (!callTrigger(start_det_client_, "/start_detection")) {
      RCLCPP_ERROR(get_logger(), "Failed to start detection.");
      return std::nullopt;
    }
    vision_enabled_.store(true);

    // Run for fixed duration
    RCLCPP_INFO(get_logger(), "👁️ Fixed vision running for %.1fs ...", vision_sec);
    const rclcpp::Time t_start = now();
    while (rclcpp::ok() && (now() - t_start).seconds() < vision_sec) {
      rclcpp::spin_some(this->get_node_base_interface());
      std::this_thread::sleep_for(20ms);
    }

    // Stop detection
    (void)callTrigger(stop_det_client_, "/stop_detection", 3);
    vision_enabled_.store(false);
    RCLCPP_INFO(get_logger(), "🛑 Fixed vision stopped.");

    // Read whatever we got during the window
    geometry_msgs::msg::PoseStamped pose;
    std::string color;
    bool have_pose=false, have_color=false;
    {
      std::lock_guard<std::mutex> lk(vision_mutex_);
      have_pose  = got_pose_;
      have_color = got_color_;
      if (have_pose)  pose  = latest_pose_;
      if (have_color) color = latest_color_;
    }

    if (!have_pose) {
      RCLCPP_ERROR(get_logger(), "No pose received during fixed vision window.");
      return std::nullopt;
    }
    if (!have_color) {
      RCLCPP_ERROR(get_logger(), "No color received during fixed vision window.");
      return std::nullopt;
    }
    return std::make_pair(pose, color);
  }

  // ---------- Legacy: Vision window with warm-up + pose stabilization ----------
  std::optional<std::pair<geometry_msgs::msg::PoseStamped,std::string>>
  runVisionWindow(double seconds) {
    // Reset & enable
    {
      std::lock_guard<std::mutex> lk(vision_mutex_);
      got_pose_ = false;
      got_color_ = false;
      latest_color_.clear();
      latest_pose_ = geometry_msgs::msg::PoseStamped{};
      latest_pose_stamp_ = builtin_interfaces::msg::Time{};
    }

    if (!callTrigger(start_det_client_, "/start_detection")) return std::nullopt;

    vision_enabled_.store(true);
    const rclcpp::Time t_start  = now();
    const rclcpp::Time t_accept = t_start + rclcpp::Duration::from_seconds(kVisionWarmupSec);

    RCLCPP_INFO(get_logger(), "👁️ Vision started for %.1fs (warm-up %.2fs)...",
                seconds, kVisionWarmupSec);

    bool have_prev = false;
    geometry_msgs::msg::Pose prev_pose;
    int stable_count = 0;

    std::optional<geometry_msgs::msg::PoseStamped> stable_pose;
    rclcpp::Time t_stable = rclcpp::Time(0,0,this->get_clock()->get_clock_type());

    while (rclcpp::ok() && (now() - t_start).seconds() < seconds) {
      rclcpp::spin_some(this->get_node_base_interface());
      std::this_thread::sleep_for(50ms);

      geometry_msgs::msg::PoseStamped cur_pose_msg;
      builtin_interfaces::msg::Time   cur_stamp;
      std::string cur_color;
      bool have_pose=false, have_color=false;

      {
        std::lock_guard<std::mutex> lk(vision_mutex_);
        have_pose  = got_pose_;
        have_color = got_color_;
        if (have_pose) {
          cur_pose_msg = latest_pose_;
          cur_stamp    = latest_pose_stamp_;
        }
        if (have_color) cur_color = latest_color_;
      }

      const rclcpp::Time t_now = now();

      if (t_now < t_accept) continue;
      if (kEnforceMsgStamp && have_pose) {
        if (rclcpp::Time(cur_stamp) < t_accept) continue;
      }

      if (!stable_pose && have_pose) {
        if (!have_prev) {
          prev_pose = cur_pose_msg.pose;
          have_prev = true;
          stable_count = 1;
        } else {
          const double dp = pos_distance(cur_pose_msg.pose, prev_pose);
          if (dp <= kStablePosTol) stable_count++;
          else { prev_pose = cur_pose_msg.pose; stable_count = 1; }
        }

        if (stable_count >= kStableMinCount) {
          stable_pose = cur_pose_msg;
          t_stable = t_now;
          RCLCPP_INFO(get_logger(), "✅ Pose stabilized (count=%d, tol=%.3fm). Waiting for color...",
                      stable_count, kStablePosTol);
        }
      }

      if (stable_pose) {
        if (have_color) {
          (void)callTrigger(stop_det_client_, "/stop_detection", 3);
          vision_enabled_.store(false);
          RCLCPP_INFO(get_logger(), "🛑 Vision stopped (stable pose + color).");
          return std::make_pair(*stable_pose, cur_color);
        }
        if ((t_now - t_stable).seconds() >= kColorGraceSec) {
          RCLCPP_WARN(get_logger(), "Color not received within %.1fs after stable pose.", kColorGraceSec);
          break;
        }
      }
    }

    (void)callTrigger(stop_det_client_, "/stop_detection", 4);
    vision_enabled_.store(false);
    RCLCPP_WARN(get_logger(), "🛑 Vision stopped (timeout without stable pose+color).");
    return std::nullopt;
  }

  void stopDetectionIfRunning() {
    if (stop_det_client_->wait_for_service(1s)) {
      (void)callTrigger(stop_det_client_, "/stop_detection", 2);
      RCLCPP_INFO(get_logger(), "Ensured detection is stopped before initial feed.");
    }
  }

  // ---------- JSON + naming helpers ----------
  void loadTargetsJson() {
    const std::string a = "/home/azif/projetcs/parol6/robot_data/Targets.json";
    const std::string b = "/home/azif/projects/parol6/robot_data/Targets.json";
    for (auto &p : {a, b}) {
      std::ifstream f(p);
      if (f.is_open()) {
        try {
          f >> targets_json_;
          targets_json_loaded_ = true;
          RCLCPP_INFO(get_logger(), "Loaded Targets.json from %s", p.c_str());
          return;
        } catch (const std::exception &e) {
          RCLCPP_WARN(get_logger(), "Failed parsing %s: %s", p.c_str(), e.what());
        }
      }
    }
    RCLCPP_WARN(get_logger(), "Targets.json not found. Named targets only will be used.");
  }

  std::string nextModelName(const std::string &color) {
    auto it = color_counts_.find(color);
    if (it == color_counts_.end()) {
      RCLCPP_WARN(get_logger(), "Unknown color '%s', defaulting to red counter.", color.c_str());
      return "Red_1";
    }
    it->second += 1;
    std::string cap = color;
    cap[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(cap[0])));
    std::string name = cap + "_" + std::to_string(it->second);
    RCLCPP_INFO(get_logger(), "Model name chosen: %s", name.c_str());
    return name;
  }

  // ---------- Motion helpers ----------
  bool goNamed(const std::string &name) {
    if (!ensureMoveGroup()) return false;

    move_group_->setNamedTarget(name);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto ok = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!ok) {
      RCLCPP_ERROR(get_logger(), "Plan to named target '%s' failed.", name.c_str());
      return false;
    }
    auto exec = move_group_->execute(plan);
    if (exec != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Exec to named target '%s' failed.", name.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "Moved to named target: %s", name.c_str());
    return true;
  }

  bool goNamedOrJson(const std::string &name) {
    if (goNamed(name)) return true;

    if (!targets_json_loaded_) {
      RCLCPP_ERROR(get_logger(), "Target '%s' not available and no JSON loaded.", name.c_str());
      return false;
    }
    if (!targets_json_.contains(name) || !targets_json_[name].is_array()) {
      RCLCPP_ERROR(get_logger(), "Targets.json has no array for '%s'.", name.c_str());
      return false;
    }
    if (!ensureMoveGroup()) return false;

    std::vector<double> joints;
    for (auto &v : targets_json_[name]) joints.push_back(static_cast<double>(v));

    move_group_->setJointValueTarget(joints);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto ok = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!ok) {
      RCLCPP_ERROR(get_logger(), "Plan to JSON target '%s' failed.", name.c_str());
      return false;
    }
    auto exec = move_group_->execute(plan);
    if (exec != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Exec to JSON target '%s' failed.", name.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "Moved to JSON target: %s", name.c_str());
    return true;
  }

  bool attachDetach(const std::string &model_name, bool attach) {
    if (!attach_client_->wait_for_service(5s)) {
      RCLCPP_ERROR(get_logger(), "Service /AttachDetach not available.");
      return false;
    }
    auto req = std::make_shared<msg_gazebo::srv::AttachDetach::Request>();
    req->model1 = "parol6";
    req->link1  = "L6";
    req->model2 = model_name; // name unused if external node already attached/detached
    req->link2  = "link";
    req->attach = attach;

    auto fut = attach_client_->async_send_request(req);
    auto rc = rclcpp::spin_until_future_complete(this->get_node_base_interface(), fut);
    if (rc != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "AttachDetach call failed.");
      return false;
    }
    auto res = fut.get();
    if (!res->success) {
      RCLCPP_ERROR(get_logger(), "%s failed for %s.", attach ? "Attach" : "Detach", model_name.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "%s OK: %s", attach ? "Attached" : "Detached", model_name.c_str());
    return true;
  }

  // ---------- Conveyor + initial feed ----------
  bool moveConveyor(const std::string &srv, double distance) {
    auto client = this->create_client<conveyorbelt_msgs::srv::MoveDistance>(srv);
    if (!client->wait_for_service(5s)) {
      RCLCPP_ERROR(get_logger(), "Service %s not available.", srv.c_str());
      return false;
    }
    auto req = std::make_shared<conveyorbelt_msgs::srv::MoveDistance::Request>();
    req->distance = distance;
    auto fut = client->async_send_request(req);
    auto rc = rclcpp::spin_until_future_complete(this->get_node_base_interface(), fut);
    if (rc != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Call to %s failed.", srv.c_str());
      return false;
    }
    auto res = fut.get();
    if (!res->success) {
      RCLCPP_ERROR(get_logger(), "Conveyor move %s reported failure.", srv.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "Moved %s by %.3f", srv.c_str(), distance);
    return true;
  }

  bool spawnBox() { return callTrigger(spawn_client_, "/spawn_box"); }

  bool doInitialFeed(int times, std::chrono::seconds inter_cycle_delay = 3s) {
    for (int i = 0; i < times; ++i) {
      if (!spawnBox()) return false;          // spawn
      std::this_thread::sleep_for(1s);        // 1s delay after spawn
      if (!moveConveyor("/conveyor2/MoveDistance", kConveyor2Step)) return false; // move
      if (i < times - 1) std::this_thread::sleep_for(inter_cycle_delay);
    }
    return true;
  }

  // ---------- Service helper ----------
  bool callTrigger(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &cli,
                   const char *name, int wait_sec=5)
  {
    if (!cli->wait_for_service(std::chrono::seconds(wait_sec))) {
      RCLCPP_ERROR(get_logger(), "Service %s not available.", name);
      return false;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto fut = cli->async_send_request(req);
    auto rc = rclcpp::spin_until_future_complete(this->get_node_base_interface(), fut);
    if (rc != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Call to %s failed to complete.", name);
      return false;
    }
    auto res = fut.get();
    if (!res->success) {
      RCLCPP_ERROR(get_logger(), "Service %s returned failure: %s", name, res->message.c_str());
      return false;
    }
    return true;
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PipelineNode>();
  node->run();   // logic runs after construction
  rclcpp::shutdown();
  return 0;
}
