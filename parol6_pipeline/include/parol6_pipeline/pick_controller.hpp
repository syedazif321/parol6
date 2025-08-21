// File: include/parol6_pipeline/pick_controller.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>  // ← Required for Float64
#include <std_msgs/msg/string.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

namespace parol6_pipeline
{

class PickController : public rclcpp::Node
{
public:
  explicit PickController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void initialize_move_group();
  bool move_to_pose(const geometry_msgs::msg::PoseStamped& pose_msg);

  rclcpp::Logger logger_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;  // ← Added
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;

  geometry_msgs::msg::PoseStamped latest_pose_;
  bool received_pose_;

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  bool move_group_initialized_;

  double current_speed_;  // ← Added: stores current speed from slider
};

} // namespace parol6_pipeline