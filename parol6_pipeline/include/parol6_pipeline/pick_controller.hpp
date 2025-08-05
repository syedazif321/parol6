#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

namespace parol6_pipeline
{

class PickController : public rclcpp::Node
{
public:
  explicit PickController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  bool move_to_pose(const geometry_msgs::msg::Pose & pose);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  moveit::planning_interface::MoveGroupInterface arm_interface_;

  geometry_msgs::msg::PoseStamped latest_pose_;
  bool received_pose_ = false;
};

}  // namespace parol6_pipeline
