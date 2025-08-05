#include "parol6_pipeline/pick_controller.hpp"
#include "rclcpp/rclcpp.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <thread>
#include <chrono>
#include <cmath>

namespace parol6_pipeline
{

PickController::PickController(const rclcpp::NodeOptions & options)
: Node("pick_controller", options),
  arm_interface_(std::shared_ptr<rclcpp::Node>(this), "arm")
{
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/detected_box_pose", 10,
    std::bind(&PickController::pose_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "[PickController] Node initialized and waiting for poses on /detected_box_pose");
}

void PickController::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "[Pose Callback] Received pose:");
  RCLCPP_INFO(this->get_logger(), " - Position: [x=%.3f, y=%.3f, z=%.3f]", 
              msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  RCLCPP_INFO(this->get_logger(), " - Orientation: [x=%.3f, y=%.3f, z=%.3f, w=%.3f]",
              msg->pose.orientation.x, msg->pose.orientation.y,
              msg->pose.orientation.z, msg->pose.orientation.w);

  // Normalize the quaternion
  tf2::Quaternion q(
    msg->pose.orientation.x,
    msg->pose.orientation.y,
    msg->pose.orientation.z,
    msg->pose.orientation.w
  );
  q.normalize();

  // Log quaternion norm before using
  double norm = std::sqrt(
    std::pow(q.x(), 2) + std::pow(q.y(), 2) + std::pow(q.z(), 2) + std::pow(q.w(), 2));
  RCLCPP_INFO(this->get_logger(), "[Orientation Check] Quaternion norm = %.5f", norm);

  // Apply normalized orientation back
  geometry_msgs::msg::Pose pose = msg->pose;
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();

  latest_pose_ = *msg;
  received_pose_ = true;

  // Attempt to move
  if (move_to_pose(pose)) {
    RCLCPP_INFO(this->get_logger(), "[Motion] Arm successfully reached the target pose.");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    arm_interface_.setNamedTarget("init");
    if (!arm_interface_.move()) {
      RCLCPP_ERROR(this->get_logger(), "[Motion] Failed to return to init pose.");
    } else {
      RCLCPP_INFO(this->get_logger(), "[Motion] Arm returned to init pose.");
    }

  } else {
    RCLCPP_ERROR(this->get_logger(), "[Motion] Failed to reach the target pose.");
  }
}

bool PickController::move_to_pose(const geometry_msgs::msg::Pose & pose)
{
  RCLCPP_DEBUG(this->get_logger(), "[Planning] Setting pose target...");
  arm_interface_.setPoseTarget(pose);

  arm_interface_.setGoalPositionTolerance(0.01);
  arm_interface_.setGoalOrientationTolerance(0.01);

  RCLCPP_INFO(this->get_logger(), "[Planning] Planning trajectory...");
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(arm_interface_.plan(plan));

  if (success) {
    RCLCPP_INFO(this->get_logger(), "[Planning] Plan successful. Executing...");
    return static_cast<bool>(arm_interface_.execute(plan));
  } else {
    RCLCPP_ERROR(this->get_logger(), "[Planning] Plan failed.");
    return false;
  }
}

}  // namespace parol6_pipeline

// ------------------- MAIN -------------------

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<parol6_pipeline::PickController>();
  RCLCPP_INFO(node->get_logger(), "[Main] Starting PickController node...");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
