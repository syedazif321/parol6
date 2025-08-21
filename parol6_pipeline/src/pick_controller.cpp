

#include "parol6_pipeline/pick_controller.hpp"

// ROS & MoveIt
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/duration.hpp> 


#include <memory>

namespace parol6_pipeline
{

PickController::PickController(const rclcpp::NodeOptions & options)
: Node("pick_controller", options),
  logger_(get_logger()),
  received_pose_(false),
  move_group_initialized_(false)
{
  RCLCPP_INFO(logger_, "PickController started. Waiting for /detected_box_pose and /start_picking trigger.");

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/detected_box_pose",
    10,
    std::bind(&PickController::pose_callback, this, std::placeholders::_1)
  );

 
  trigger_srv_ = create_service<std_srvs::srv::Trigger>(
    "start_picking",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response
    )
    {

      if (!received_pose_) {
        response->success = false;
        response->message = "No box pose received yet!";
        RCLCPP_WARN(logger_, "%s", response->message.c_str());
        return;
      }

      if (!move_group_initialized_) {
        initialize_move_group();
        if (!move_group_) {
          response->success = false;
          response->message = "Failed to initialize MoveGroupInterface!";
          RCLCPP_ERROR(logger_, "%s", response->message.c_str());
          return;
        }
        move_group_initialized_ = true;
      }

      RCLCPP_INFO(logger_, "Trigger received. Executing pick to [%.3f, %.3f, %.3f] in frame '%s'",
                  latest_pose_.pose.position.x,
                  latest_pose_.pose.position.y,
                  latest_pose_.pose.position.z,
                  latest_pose_.header.frame_id.c_str());

      if (move_to_pose(latest_pose_)) {
        response->success = true;
        response->message = "Picking completed successfully!";
        RCLCPP_INFO(logger_, "Picking succeeded.");
      } else {
        response->success = false;
        response->message = "Failed to plan or execute motion!";
        RCLCPP_ERROR(logger_, "Picking failed.");
      }
    }
  );
}

void PickController::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  latest_pose_ = *msg;
  received_pose_ = true;
  RCLCPP_INFO_ONCE(logger_, "Received first detected box pose from frame '%s'",
                   msg->header.frame_id.c_str());
}

void PickController::initialize_move_group()
{
  try {
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");

    move_group_->setMaxVelocityScalingFactor(1.0);
    move_group_->setMaxAccelerationScalingFactor(1.0);

    RCLCPP_INFO(logger_, "MoveGroupInterface initialized with 1.0x velocity and acceleration scaling.");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Failed to initialize MoveGroupInterface: %s", e.what());
    move_group_.reset();
  }
}

bool PickController::move_to_pose(const geometry_msgs::msg::PoseStamped& pose_msg)
{
  if (!move_group_) {
    RCLCPP_ERROR(logger_, "MoveGroupInterface not initialized!");
    return false;
  }

  move_group_->setMaxVelocityScalingFactor(1.0);
  move_group_->setMaxAccelerationScalingFactor(1.0);

  move_group_->setPoseReferenceFrame(pose_msg.header.frame_id);
  move_group_->setPoseTarget(pose_msg.pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool planning_success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (!planning_success) {
    RCLCPP_ERROR(logger_, "Planning failed for target pose!");
    return false;
  }

  RCLCPP_INFO(logger_, "Planning successful! Executing motion...");

  auto &pts = plan.trajectory_.joint_trajectory.points;
  if (!pts.empty()) {
    double duration = rclcpp::Duration(pts.back().time_from_start).seconds();
    RCLCPP_INFO(logger_, "Planned trajectory duration: %.2f s", duration);
  }

  moveit::core::MoveItErrorCode exec_result = move_group_->execute(plan);

  if (exec_result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_INFO(logger_, "Motion executed successfully.");
    return true;
  } else {
    RCLCPP_ERROR(logger_, "Execution failed! Error code: %d", exec_result.val);
    return false;
  }
}

} 


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<parol6_pipeline::PickController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}