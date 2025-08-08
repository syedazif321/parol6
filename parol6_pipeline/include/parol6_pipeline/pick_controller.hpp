// File: include/parol6_pipeline/pick_controller.hpp

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_srvs/srv/trigger.hpp>

namespace parol6_pipeline
{

class PickController : public rclcpp::Node
{
public:
  explicit PickController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  bool move_to_pose(const geometry_msgs::msg::PoseStamped& pose_msg);
  void initialize_move_group(); 
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  geometry_msgs::msg::PoseStamped latest_pose_;
  bool received_pose_ = false;
  bool move_group_initialized_ = false;

  rclcpp::Logger logger_ = get_logger();
};

}  