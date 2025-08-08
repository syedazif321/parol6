// File: src/pipeline.cpp

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <conveyorbelt_msgs/srv/move_distance.hpp>
#include <msg_gazebo/srv/attach_detach.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <nlohmann/json.hpp>

#include <fstream>
#include <string>
#include <cstdlib>

using json = nlohmann::json;

class PipelineNode : public rclcpp::Node
{
public:
  PipelineNode() : Node("pipeline_node")
  {
    RCLCPP_INFO(get_logger(), "Pipeline node started. Press Enter to begin...");
    std::cin.get();

    if (!spawn_box()) {
      RCLCPP_ERROR(get_logger(), "Failed to spawn box.");
      return;
    }

    if (!move_conveyor("conveyor2", 0.65)) {
      RCLCPP_ERROR(get_logger(), "Failed to move conveyor2.");
      return;
    }

    if (!detect_box()) {
      RCLCPP_ERROR(get_logger(), "Box detection failed.");
      return;
    }

    if (!start_picking()) {
      RCLCPP_ERROR(get_logger(), "Picking failed.");
      return;
    }

    if (!attach_object(true)) {
      RCLCPP_ERROR(get_logger(), "Attach failed.");
      return;
    }

    std::string target_conveyor = get_target_conveyor(box_color_);
    if (target_conveyor.empty()) {
      RCLCPP_ERROR(get_logger(), "No target conveyor found for color '%s'", box_color_.c_str());
      return;
    }

    if (!move_to_named_target(target_conveyor)) {
      RCLCPP_ERROR(get_logger(), "Failed to move to %s", target_conveyor.c_str());
      return;
    }

    if (!attach_object(false)) {
      RCLCPP_ERROR(get_logger(), "Detach failed.");
      return;
    }

    if (!move_conveyor(target_conveyor, 0.65)) {
      RCLCPP_ERROR(get_logger(), "Failed to move target conveyor: %s", target_conveyor.c_str());
      return;
    }

    RCLCPP_INFO(get_logger(), "✅ Pipeline completed! Box '%s' delivered to %s.",
                spawned_box_name_.c_str(), target_conveyor.c_str());
  }

private:
  std::string spawned_box_name_;
  std::string box_color_ = "red";

  // === Service Clients ===
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr spawn_client_;
  rclcpp::Client<conveyorbelt_msgs::srv::MoveDistance>::SharedPtr conveyor_client_;
  rclcpp::Client<msg_gazebo::srv::AttachDetach>::SharedPtr attach_client_;

  // === MoveIt ===
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

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

    // In real system, parse from response. For now, hardcode.
    spawned_box_name_ = "Red_1";  // or "Blue_1"
    box_color_ = (spawned_box_name_.find("Red") != std::string::npos) ? "red" : "blue";
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
    RCLCPP_INFO(get_logger(), "Running box_detector_node...");
    int ret = std::system("ros2 run parol6_pipeline box_detector_node");
    if (ret != 0) {
      RCLCPP_ERROR(get_logger(), "box_detector_node failed!");
      return false;
    }
    RCLCPP_INFO(get_logger(), "Box detection completed.");
    return true;
  }

  bool start_picking()
  {
    auto client = this->create_client<std_srvs::srv::Trigger>("/start_picking");
    if (!client->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Service /start_picking not available.");
      return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(request);
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
    std::string json_path = "/home/azif/projetcs/parol6/robot_data/Targets.json";
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