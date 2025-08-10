// File: src/pipeline.cpp

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <conveyorbelt_msgs/srv/move_distance.hpp>
#include <msg_gazebo/srv/attach_detach.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <nlohmann/json.hpp>

#include <fstream>
#include <string>
#include <thread>
#include <mutex>

using json = nlohmann::json;

class PipelineNode : public rclcpp::Node
{
public:
  PipelineNode() : Node("pipeline_node")
  {
    RCLCPP_INFO(get_logger(), "Pipeline node started. Press Enter to begin...");
    std::cin.get();

    if (!spawn_box()) return;
    
    // Delay 1 sec after spawn
    RCLCPP_INFO(get_logger(), "⏳ Waiting 1 second after spawning box...");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (!move_conveyor("conveyor2", 0.65)) return;

    // Delay 2 sec before starting vision
    RCLCPP_INFO(get_logger(), "⏳ Waiting 2 seconds before starting vision...");
    std::this_thread::sleep_for(std::chrono::seconds(5));

    if (!detect_box()) return;

    if (!start_picking()) return;
    if (!attach_object(true)) return;

    std::string target_conveyor = get_target_conveyor(box_color_);
    if (target_conveyor.empty()) {
      RCLCPP_ERROR(get_logger(), "No target conveyor found for color '%s'", box_color_.c_str());
      return;
    }

    if (!move_to_named_target(target_conveyor)) return;
    if (!attach_object(false)) return;
    if (!move_conveyor(target_conveyor, 0.65)) return;

    RCLCPP_INFO(get_logger(), "✅ Pipeline completed! Box '%s' delivered to %s.",
                spawned_box_name_.c_str(), target_conveyor.c_str());
  }

private:
  std::string spawned_box_name_ = "Red_1";
  std::string box_color_ = "red";

  // Clients
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr spawn_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr detect_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pick_client_;
  rclcpp::Client<conveyorbelt_msgs::srv::MoveDistance>::SharedPtr conveyor_client_;
  rclcpp::Client<msg_gazebo::srv::AttachDetach>::SharedPtr attach_client_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  // Detection sync
  geometry_msgs::msg::PoseStamped::SharedPtr latest_pose_;
  std::mutex pose_mutex_;
  bool box_detected_ = false;

  bool spawn_box()
  {
    spawn_client_ = this->create_client<std_srvs::srv::Trigger>("/spawn_box");
    if (!spawn_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Service /spawn_box not available.");
      return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = spawn_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to call /spawn_box");
      return false;
    }

    auto result = future.get();
    if (!result->success) {
      RCLCPP_ERROR(get_logger(), "Spawn service failed.");
      return false;
    }

    box_color_ = "red";  // In real system, parse from response
    spawned_box_name_ = "Red_1";
    RCLCPP_INFO(get_logger(), "Spawned box: %s (color: %s)", spawned_box_name_.c_str(), box_color_.c_str());
    return true;
  }

  bool move_conveyor(const std::string& conveyor, double distance)
  {
    auto client = this->create_client<conveyorbelt_msgs::srv::MoveDistance>(conveyor + "/MoveDistance");
    if (!client->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Service %s/MoveDistance not available.", conveyor.c_str());
      return false;
    }

    auto request = std::make_shared<conveyorbelt_msgs::srv::MoveDistance::Request>();
    request->distance = distance;

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to call %s/MoveDistance", conveyor.c_str());
      return false;
    }

    auto result = future.get();
    if (!result->success) {
      RCLCPP_ERROR(get_logger(), "Conveyor move failed.");
      return false;
    }

    RCLCPP_INFO(get_logger(), "Moved %s by %.2f", conveyor.c_str(), distance);
    return true;
  }

  bool detect_box()
  {
    RCLCPP_INFO(get_logger(), "🔍 Starting vision phase in 2 seconds...");

    // Subscribe to /detected_box_pose
    auto sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/detected_box_pose",
      10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        latest_pose_ = msg;
        box_detected_ = true;
        RCLCPP_INFO_ONCE(get_logger(), "📸 Detected box pose in '%s'", msg->header.frame_id.c_str());
      });

    // Create detection clients
    detect_client_ = this->create_client<std_srvs::srv::Trigger>("/start_detection");
    auto stop_client = this->create_client<std_srvs::srv::Trigger>("/stop_detection");

    // Wait for /start_detection
    if (!detect_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Service /start_detection not available. Is box_detector_node running?");
      return false;
    }

    // Start detection
    auto start_req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto start_future = detect_client_->async_send_request(start_req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), start_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to call /start_detection");
      return false;
    }

    auto start_result = start_future.get();
    if (!start_result->success) {
      RCLCPP_ERROR(get_logger(), "Start detection failed: %s", start_result->message.c_str());
      return false;
    }

    RCLCPP_INFO(get_logger(), "👁️  Vision started. Waiting up to 2 seconds for a box...");

    // Wait up to 2 seconds for a box
    auto start_time = this->now();
    while (rclcpp::ok() && !box_detected_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if ((this->now() - start_time).seconds() >= 2.0) {
        break;
      }
    }

    // Always stop detection
    if (!stop_client->wait_for_service(std::chrono::seconds(3))) {
      RCLCPP_WARN(get_logger(), "Service /stop_detection not available.");
    } else {
      auto stop_req = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto stop_future = stop_client->async_send_request(stop_req);
      rclcpp::spin_until_future_complete(this->get_node_base_interface(), stop_future);
      RCLCPP_INFO(get_logger(), "🛑 Vision stopped after 2 seconds.");
    }

    // Check result
    if (!box_detected_) {
      RCLCPP_ERROR(get_logger(), "❌ No box detected during 2-second window.");
      return false;
    }

    RCLCPP_INFO(get_logger(), "✅ Box detected at (%.3f, %.3f, %.3f)",
                latest_pose_->pose.position.x,
                latest_pose_->pose.position.y,
                latest_pose_->pose.position.z);
    return true;
  }

  bool start_picking()
  {
    pick_client_ = this->create_client<std_srvs::srv::Trigger>("/start_picking");
    if (!pick_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Service /start_picking not available.");
      return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = pick_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to call /start_picking");
      return false;
    }

    auto result = future.get();
    if (!result->success) {
      RCLCPP_ERROR(get_logger(), "Picking failed: %s", result->message.c_str());
      return false;
    }

    RCLCPP_INFO(get_logger(), "Picking succeeded.");
    return true;
  }

  bool attach_object(bool attach)
  {
    if (!attach_client_) {
      attach_client_ = this->create_client<msg_gazebo::srv::AttachDetach>("/AttachDetach");
    }
    if (!attach_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Service /AttachDetach not available.");
      return false;
    }

    auto request = std::make_shared<msg_gazebo::srv::AttachDetach::Request>();
    request->model1 = "parol6";
    request->link1 = "L6";
    request->model2 = spawned_box_name_;
    request->link2 = "link";
    request->attach = attach;

    auto future = attach_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to call /AttachDetach");
      return false;
    }

    auto result = future.get();
    if (!result->success) {
      RCLCPP_ERROR(get_logger(), "Attach/Detach failed.");
      return false;
    }

    RCLCPP_INFO(get_logger(), "%s object: %s", attach ? "Attached" : "Detached", spawned_box_name_.c_str());
    return true;
  }

  std::string get_target_conveyor(const std::string& color)
  {
    std::string json_path = "/home/azif/projects/parol6/robot_data/Targets.json";  // Fixed typo: projetcs → projects
    std::ifstream f(json_path);
    if (!f.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open %s", json_path.c_str());
      return "";
    }

    json j;
    try {
      f >> j;
      if (j.contains(color)) {
        std::string target = j[color];
        RCLCPP_INFO(get_logger(), "Color '%s' → Target conveyor: %s", color.c_str(), target.c_str());
        return target;
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to parse JSON: %s", e.what());
    }

    return "";
  }

  bool move_to_named_target(const std::string& target_name)
  {
    if (!move_group_) {
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
    }

    move_group_->setNamedTarget(target_name);
    RCLCPP_INFO(get_logger(), "Planning motion to named target: %s", target_name.c_str());

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode plan_result = move_group_->plan(plan);

    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Planning to '%s' failed!", target_name.c_str());
      return false;
    }

    RCLCPP_INFO(get_logger(), "Executing motion to %s...", target_name.c_str());
    moveit::core::MoveItErrorCode exec_result = move_group_->move();

    if (exec_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Execution failed! Code: %d", exec_result.val);
      return false;
    }

    RCLCPP_INFO(get_logger(), "Successfully moved to target: %s", target_name.c_str());
    return true;
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PipelineNode>());
  rclcpp::shutdown();
  return 0;
}